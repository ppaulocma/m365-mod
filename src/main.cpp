// ============================================================================
// Patinete ESP — controlador BLDC M365/Xbot com ESP32 + SimpleFOC
// ============================================================================
//
//   ESTÁGIO DE POTÊNCIA: placa original M365/Xbot com 3x FD2103S em modo
//   3-PWM (HIN+LIN unidos, dead-time interno). Halls via conversor de nível.
//   Shunt único de barramento no ADC. Toda a configuração está em
//   include/config.h — procure por "AJUSTE AQUI".
//
// ─── SEQUÊNCIA DE PRIMEIRO TESTE (resumo — detalhes no README.md) ───────────
//
//  0. GND COMUM! O GND do ESP32 tem que estar ligado ao GND da controladora
//     (mesma referência dos FD2103S, dos shunts e dos conversores de nível).
//  1. Alimente o estágio de potência por FONTE DE BANCADA com limite de
//     corrente (~2 A no início), NUNCA direto na bateria no primeiro teste.
//     Roda SUSPENSA (motor livre).
//  2. Grave e abra o serial a 115200. O boot despeja TODOS os parâmetros
//     vivos ('*' = diferente do default) — é a foto do estado, e vale colar
//     junto de qualquer relato de problema.
//  3. CALIBRE NESTA ORDEM (pelo app ou pelo console; cada passo depende do
//     anterior, e fora de ordem cada um mede em cima do erro do outro):
//       a) 'cal'  — corrente de fase. Mede o mapa de fases, o sinal e a
//                   ESCALA (contra o INA226). Sem ela o motor não liga:
//                   sinal de sensor invertido é realimentação positiva.
//                   Antes dela, confira a corrente de bateria com
//                   amperímetro ('ibat' + 'shunt <mOhm>') — ela ancora tudo.
//       b) 'zea'  — ângulo de comutação + DIREÇÃO do sensor Hall.
//       c) 'ang'  — afina o ângulo GIRANDO (mínimo de corrente). É o passo
//                   que a calibração parada não consegue fazer.
//     Motor girou áspero ou consumindo muito? Quase sempre é (c) faltando.
//  4. Sentido de giro invertido? Use o parâmetro "Inverter sentido" e
//     recalibre o ângulo — não mexa na fiação por causa disso.
//  5. Confira os ADCs no config.h (divisor do VBAT, beta do NTC) e só então
//     suba os limites pelo app.
//
// ─── ARQUITETURA ────────────────────────────────────────────────────────────
//
//   NÚCLEO 1: motorControlTask — SimpleFOC (loopFOC/move) sem bloqueios,
//             proteções a 1 kHz (sobrecorrente, subtensão, sobretemperatura),
//             foldback de corrente e rampas. Prioridade alta.
//   NÚCLEO 0: bleDashboardTask (app via Bluetooth/NimBLE), task de sensores
//             (INA226/ADCs) e link serial com o painel físico (Nano).
//   Os núcleos só conversam pelas structs Command/Telemetry protegidas por
//   spinlock (shared_state.*). Nada de global solta.
//
// ============================================================================
#include <Arduino.h>
#include "config.h"
#include "params.h"
#include "curves.h"
#include "faultlog.h"
#include "motor_control.h"
#include "phase_current.h"
#include "ble_dashboard.h"
#include "ina226.h"
#include "uart_panel.h"

void setup() {
  // Estado determinístico dos gates o mais cedo possível: nível baixo =
  // MOSFETs de baixo conduzindo (repouso seguro dos FD2103S; os inputs
  // deles têm pull-down interno, isto só reforça).
  pinMode(PIN_PWM_A, OUTPUT); digitalWrite(PIN_PWM_A, LOW);
  pinMode(PIN_PWM_B, OUTPUT); digitalWrite(PIN_PWM_B, LOW);
  pinMode(PIN_PWM_C, OUTPUT); digitalWrite(PIN_PWM_C, LOW);

  // Buffer TX grande: os prints de diagnóstico (mcPrintf) são absorvidos pelo
  // buffer em vez de BLOQUEAREM a task do motor esperando a UART drenar (115200
  // é lento — um bloqueio de ms no loop FOC vira solavanco de torque).
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=====================================================");
  Serial.println("  Patinete ESP — BLDC M365 + SimpleFOC (FD2103S 3PWM)");
  // Carimbo de build: a caixa-preta provou que "qual binário está gravado"
  // não pode ser suposição — um despejo de firmware velho custou uma rodada
  // inteira de diagnóstico. Confira esta linha após TODO upload.
  Serial.printf("  build %s %s\n", __DATE__, __TIME__);
  Serial.println("=====================================================");

  // Parâmetros, curvas, log de faltas e odômetro (NVS) — antes das tasks
  paramsInit();
  paramsDump();     // boot autodocumentado: TODO o estado vivo no paste
  curvesInit();
  faultLogInit();
  motorEnergyInit();

  // Núcleo 0: BLE + dashboard (criada primeiro — a task do motor tem
  // prioridade maior que esta loopTask e tomaria o núcleo 1 antes do
  // setup() terminar).
  xTaskCreatePinnedToCore(bleDashboardTask, "ble", WEB_TASK_STACK,
                          nullptr, WEB_TASK_PRIO, nullptr, 0);

#if CUR_SENSE_ENABLED || VBAT_SENSE_ENABLED || NTC_SENSE_ENABLED
  // Núcleo 0: task de sensores lentos (INA226 por I2C + ADCs de tensão/NTC).
  // I2C e analogRead são bloqueantes/têm seção crítica — longe do motor!
  xTaskCreatePinnedToCore(ina226Task, "sensores", 3072, nullptr, 2, nullptr, 0);
#endif

#if PANEL_UART_ENABLED
  // Núcleo 0: link com o painel do guidão (Nano) — acelerador/freio/display
  xTaskCreatePinnedToCore(panelTask, "painel", 4096, nullptr, 2, nullptr, 0);
#endif

  // Núcleo 1: controle do motor (alta prioridade, criada por último)
  xTaskCreatePinnedToCore(motorControlTask, "motor", MOTOR_TASK_STACK,
                          nullptr, MOTOR_TASK_PRIO, nullptr, 1);
}

#if IPH_DIAG_TOOLS
// ---------------------------------------------------------------------------
// CONSOLE DE BANCADA (USB) — as mesmas rotinas do app, sem precisar do app
// ---------------------------------------------------------------------------
// Existe por dois motivos: permitir um roteiro automatizado do lado do PC
// (tools/bench_test.py) e poder diagnosticar com o BLE fora do caminho — o
// rádio é justamente uma das fontes de ruído que se quer medir.
// Roda na loopTask, que estava ociosa. Os comandos só levantam as mesmas flags
// que o BLE levanta; quem executa continua sendo a task do motor.
static void benchConsole() {
  static char buf[24];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c != '\n' && c != '\r') {
      if (n < sizeof(buf) - 1) buf[n++] = c;
      continue;
    }
    if (!n) continue;
    buf[n] = 0;
    n = 0;

    if      (!strcmp(buf, "diag"))    { motorRequestIphDiag(); }
    else if (!strcmp(buf, "crash"))   { motorRequestCrash(); }
    else if (!strcmp(buf, "cal"))     { motorRequestIphCal(); }
    else if (!strcmp(buf, "zea"))     { motorRequestCalZea(); }
    else if (!strcmp(buf, "zero"))    { motorRequestIphZero(); }
    else if (!strcmp(buf, "factory")) { paramsFactory(); curvesFactory(); }
    else if (!strcmp(buf, "save"))    { paramsRequestSave(); }
    // 'gira [A] [km/h]' e 'ab [A] [km/h]' — roda LIVRE, velocidade baixa.
    else if (!strncmp(buf, "gira", 4) || !strncmp(buf, "spin", 4)) {
      float a = 3.0f, v = SPIN_TEST_KMH;
      if (buf[4] == ' ') sscanf(buf + 5, "%f %f", &a, &v);
      motorRequestSpin(a, v);
    }
    // 'ang [A] [km/h]' — varredura do angulo de comutacao sob rotacao (acha
    // o minimo de corrente de bateria; o residuo do setor que o zea estatico
    // nao enxerga). RODA LIVRE.
    else if (!strncmp(buf, "ang", 3)) {
      float a = 3.0f, v = 6.0f;
      if (buf[3] == ' ') sscanf(buf + 4, "%f %f", &a, &v);
      motorRequestAngScan(a, v);
    }
    // 'varre [km/h] [A]' — VARREDURA. Acelera do REPOUSO ate o teto com
    // CORRENTE FIXA e decompoe a ondulacao do id nas harmonicas do proprio
    // angulo (1x/2x/6x eletrica + 1x mecanica), com o piso do ruido ao lado de
    // cada numero. Corrente fixa porque numa roda livre segurar VELOCIDADE nao
    // custa torque: a inercia da roda e a unica carga da bancada, e ela so
    // aparece acelerando. Cada harmonica aponta uma causa diferente. Tambem
    // mede quantos graus tem cada um dos 6 setores de Hall. RODA LIVRE.
    else if (!strncmp(buf, "varre", 5)) {
      float v = 20.0f, a = 5.0f;
      if (buf[5] == ' ') sscanf(buf + 6, "%f %f", &v, &a);
      motorRequestSweep(v, a);
    }
    else if (!strncmp(buf, "stepv", 5)) {          // degrau de TENSAO
      motorRequestVStep((buf[5] == ' ') ? atof(buf + 6) : 1.0f);
    }
    else if (!strncmp(buf, "step", 4)) {           // degrau de CORRENTE
      motorRequestIStep((buf[4] == ' ') ? atof(buf + 5) : 3.0f);
    }
    // Ajuste dos ganhos direto pelo console: o ciclo ajustar-medir-ajustar é o
    // trabalho de fato, e passar pelo app a cada iteração (navegar, aplicar,
    // reconectar) torna isso lento demais para valer. Vale em RAM; use "save"
    // quando chegar num valor bom.
    else if (buf[0] == 'p' && buf[1] == ' ') {
      paramSet(P_CUR_P, atof(buf + 2));
      Serial.printf("[con] P_CUR_P = %.4f\n", prm(P_CUR_P));
      continue;
    }
    else if (buf[0] == 'i' && buf[1] == ' ') {
      paramSet(P_CUR_I, atof(buf + 2));
      Serial.printf("[con] P_CUR_I = %.1f\n", prm(P_CUR_I));
      continue;
    }
    // tf <s>: filtro dq da malha ao vivo (ex.: tf 0.005). Com Hall de 60° a
    // corrente REAL pulsa por setor; subir o Tf suaviza o que a malha
    // persegue (menos banda), baixar devolve banda (mais reação ao ripple).
    else if (buf[0] == 't' && buf[1] == 'f' && buf[2] == ' ') {
      paramSet(P_CUR_TF, atof(buf + 3));
      Serial.printf("[con] P_CUR_TF = %.4f s\n", prm(P_CUR_TF));
      continue;
    }
    // vp/vi: ganhos do PID de VELOCIDADE ao vivo (a NVS vence o config —
    // ajuste aqui e 'save'). Regra: zero I/P bem abaixo do cruzamento P·14.
    else if (buf[0] == 'v' && buf[1] == 'p' && buf[2] == ' ') {
      paramSet(P_PID_P, atof(buf + 3));
      Serial.printf("[con] P_PID_P = %.3f A/(rad/s)\n", prm(P_PID_P));
      continue;
    }
    else if (buf[0] == 'v' && buf[1] == 'i' && buf[2] == ' ') {
      paramSet(P_PID_I, atof(buf + 3));
      Serial.printf("[con] P_PID_I = %.2f\n", prm(P_PID_I));
      continue;
    }
    // shunt <mOhm>: shunt do INA226 de BARRAMENTO. É PARÂMETRO (NVS vence o
    // config.h) — editar INA226_SHUNT_MOHM só muda o default de fábrica.
    // Calibre comparando com um alicate/multímetro em corrente conhecida:
    //   novo_shunt = shunt_atual x (I_lida / I_real). Depois 'save'.
    else if (!strncmp(buf, "shunt ", 6)) {
      paramSet(P_SHUNT_MOHM, atof(buf + 6));
      Serial.printf("[con] P_SHUNT_MOHM = %.3f mOhm (escala ±%.0f A) — "
                    "'save' para gravar\n",
                    prm(P_SHUNT_MOHM), 81.92f / prm(P_SHUNT_MOHM));
      continue;
    }
    // lead <us>: avanço de comutação (latência ângulo→FETs). Ajuste no TOPO
    // de velocidade observando o duty vetorial do app: o ótimo é o MENOR
    // duty na mesma velocidade (menos Ud gasto com a inclinação do vetor).
    // Comece por 150-250; a caixa-preta sugeriu τ efetivo ≈370 µs.
    // dmax <fracao>: autoridade da malha do eixo d (0 = DESLIGA o eixo d).
    // O A/B decisivo quando a suspeita e "o d esta bombeando corrente": com
    // 0 o Ud vai a zero e so a comutacao age. Se a velocidade voltar e a
    // corrente cair, o problema esta na malha de d (medicao de id ou ganho);
    // se nao mudar nada, o problema e a comutacao/angulo.
    // db <A>: zona morta do INTEGRADOR das duas malhas de corrente. Tem de
    // cobrir o ruido de id/iq medidos — o 'diag' imprime o veredito e a
    // sugestao. Zona morta pequena demais = integrador perseguindo ruido =
    // corrente REAL (no eixo d aparece como "id sobe sozinho com picos").
    else if (!strncmp(buf, "db ", 3)) {
      paramSet(P_CUR_DB, atof(buf + 3));
      Serial.printf("[con] P_CUR_DB = %.2f A — 'save' para gravar\n",
                    prm(P_CUR_DB));
      continue;
    }
    else if (!strncmp(buf, "dmax ", 5)) {
      paramSet(P_UD_MAX, atof(buf + 5));
      Serial.printf("[con] P_UD_MAX = %.3f (%.1f%% do teto de tensao p/ o "
                    "eixo d; 0 = malha de d DESLIGADA) — 'save' p/ gravar\n",
                    prm(P_UD_MAX), prm(P_UD_MAX) * 100.0f);
      continue;
    }
    else if (!strncmp(buf, "lead ", 5)) {
      paramSet(P_COMM_LEAD, atof(buf + 5));
      Serial.printf("[con] P_COMM_LEAD = %.0f us — 'save' para gravar\n",
                    prm(P_COMM_LEAD));
      continue;
    }
    // fw <A>: FIELD WEAKENING por injeção de id (0 = off). Reprojetado p/ a
    // era dos sensores de fase: quando a utilização de tensão passa do
    // joelho (P_FW_START, % do teto REAL), um integrador escreve alvo
    // NEGATIVO de id na malha de d — mais topo com a mesma tensão, pagando
    // em corrente/aquecimento (o id entra em |I| e nos trips). Só TURBO,
    // nunca em regen, colapsa ao soltar. Comece com 6-8 A; máximo 15.
    // Ganho ~0,2-0,3 km/h por ampère (depende do L do motor).
    else if (!strncmp(buf, "fw ", 3)) {
      paramSet(P_FW_MAX_A, atof(buf + 3));
      Serial.printf("[con] P_FW_MAX_A = %.1f A de id (0 = FW off; so no "
                    "TURBO) — 'save' para gravar\n", prm(P_FW_MAX_A));
      continue;
    }
    else if (!strcmp(buf, "halls"))   { motorPrintHalls(); }
    else if (!strcmp(buf, "params"))  { paramsDump(); }
    // set <chave> <valor>: escreve QUALQUER parametro pela chave da tabela
    // (as chaves aparecem no 'params'). 'save' para gravar na flash.
    else if (!strncmp(buf, "set ", 4)) {
      char k[8] = {0}; float v = 0;
      if (sscanf(buf + 4, "%7s %f", k, &v) == 2) {
        int id = paramIdByKey(k);
        if (id >= 0) {
          paramSet((uint8_t)id, v);
          Serial.printf("[con] %s = %.4g — 'save' para gravar\n",
                        k, (double)prm((uint8_t)id));
        } else Serial.printf("[con] chave '%s' desconhecida (rode 'params')\n", k);
      } else Serial.println("[con] uso: set <chave> <valor>");
      continue;
    }
    // 'ibat' — corrente de BARRAMENTO ao vivo, para calibrar o shunt do
    // INA226 contra o amperimetro da FONTE (a referencia da bancada).
    else if (!strcmp(buf, "ibat")) {
#if CUR_SENSE_ENABLED
      Serial.println("[con] corrente de barramento por 5 s — compare com o "
                     "amperimetro da fonte (carga constante!):");
      for (int k = 0; k < 10; k++) {
        Serial.printf("[con]   %.3f A  (shunt em uso: %.3f mOhm)\n",
                      ina226CurrentA(), prm(P_SHUNT_MOHM));
        delay(500);
      }
      Serial.println("[con] ajuste: novo = atual x (leitura_acima / fonte) -> "
                     "'shunt <v>' + 'save' -> depois 'cal'");
#endif
    }
    else if (!strcmp(buf, "gains")) {
      Serial.printf("[con] corrente: P=%.4f I=%.1f Tf=%.4f s deadband=%.2f A | "
                    "velocidade: P=%.3f I=%.2f\n",
                    prm(P_CUR_P), prm(P_CUR_I), prm(P_CUR_TF),
                    prm(P_CUR_DB), prm(P_PID_P), prm(P_PID_I));
      continue;
    }
    else if (!strcmp(buf, "help")) {
      Serial.printf("[con] RODA LIVRE:  gira [A] [km/h]  zea  ang [A] [km/h] "
                    "(padrao %.0f km/h)\n", (double)SPIN_TEST_KMH);
      Serial.println("[con]   gira = gira e mede a ondulacao de VELOCIDADE "
                     "(a aspereza que se sente)");
      Serial.println("[con]   varre [km/h] [A] = acelera com corrente fixa e "
                     "diz de ONDE vem a aspereza (harmonicas + Halls)");
      Serial.println("[con] RODA PRESA: step [A]  stepv [V]  cal");
      Serial.println("[con] PARADO:     diag  zero  halls  ibat  crash "
                     "(caixa-preta da ultima falta)");
      Serial.println("[con] ajuste: p <v> | i <v> | tf <s> | vp <v> | vi <v> | "
                     "shunt <mOhm> | lead <us> | fw <A> | dmax <fr> | db <A> | "
                     "params | set <chave> <v> | gains | save | factory");
    }
    else { Serial.printf("[con] comando desconhecido: %s\n", buf); continue; }
    Serial.printf("[con] ok: %s\n", buf);      // eco que o roteiro espera
  }
}
#endif

// Tudo roda nas duas tasks acima; a loopTask fica ociosa.
void loop() {
#if IPH_DIAG_TOOLS
  benchConsole();
  vTaskDelay(pdMS_TO_TICKS(20));
#else
  vTaskDelay(pdMS_TO_TICKS(1000));
#endif
}
