// ============================================================================
// shared_state.h — estado compartilhado entre o núcleo 1 (motor) e o
// núcleo 0 (WiFi/dashboard), protegido por spinlock (portMUX_TYPE).
//
// Regra de ouro: NINGUÉM acessa as structs globais diretamente. Sempre
// copiar via readCommand()/writeCommand()/readTelemetry()/writeTelemetry(),
// que fazem a cópia inteira dentro de uma seção crítica curtíssima.
// ============================================================================
#pragma once
#include <Arduino.h>

// --- Modos de controle ------------------------------------------------------
enum : uint8_t {
  MODE_TORQUE   = 0,   // acelerador -> tensão (proporcional ao torque)
  MODE_VELOCITY = 1,   // acelerador -> velocidade alvo (PID fecha a malha)
};

// --- Bits de falta / estado (campo "faults" da telemetria) ------------------
#define FAULT_OVERCURRENT  0x01   // sobrecorrente no shunt (trava)
#define FAULT_UNDERVOLT    0x02   // subtensão da bateria (trava)
#define FAULT_OVERTEMP     0x04   // sobretemperatura nos MOSFETs (trava)
#define FAULT_SENSOR       0x08   // initFOC/Hall falhou (exige reboot)
#define FAULT_ESTOP        0x10   // parada de emergência acionada
#define FAULT_LINK         0x20   // sem comando da dashboard (dead-man, não trava)
#define FAULT_UV_WARN      0x40   // AVISO de bateria fraca (progressivo, não trava)
#define FAULT_STALL        0x80   // corrente de fase alta com a roda ~parada
                                  // (comutação errada / rotor travado). Corta e
                                  // TRAVA. Agora usa |I| MEDIDO pelos INA240 —
                                  // antes era um proxy por tensão x velocidade.

// Faltas que TRAVAM o motor até o usuário limpar na dashboard
#define FAULT_LATCHING_MASK (FAULT_OVERCURRENT | FAULT_UNDERVOLT | FAULT_OVERTEMP | FAULT_SENSOR | FAULT_STALL)

// --- Comando: dashboard -> motor -------------------------------------------
struct Command {
  float    throttle;      // 0..1  acelerador
  float    brake;         // 0..1  freio
  bool     enable;        // liga/desliga o motor
  bool     estop;         // parada de emergência (trava até "limpar faltas")
  uint8_t  mode;          // MODE_TORQUE / MODE_VELOCITY
  float    i_limit_a;     // [A] limite de corrente de bateria
  float    max_kmh;       // [km/h] teto da SESSÃO (0 = sem ajuste; a fonte
                          // da verdade é o parâmetro P_MAX_KMH — slider só REDUZ)
  float    v_limit;       // [V] voltage_limit do motor
  float    zea;           // [rad] ângulo zero elétrico ao vivo (NAN = usa o do config)
  uint8_t  ride;          // modo de pilotagem: 0 eco / 1 normal / 2 turbo
  bool     cruise;        // cruise control (trava a velocidade atual)
  bool     walk;          // walk mode (empurrar: ~4 km/h)
  bool     locked;        // trava eletrônica (só engata parado)
  bool     headlight;     // farol (LED no painel Nano; comandado pelo app)
  uint32_t clear_req;     // contador: incrementa a cada pedido de "limpar faltas"
  uint32_t last_cmd_ms;   // millis() do último comando recebido (dead-man)
};

// --- Telemetria: motor -> dashboard -----------------------------------------
struct Telemetry {
  float    kmh;           // velocidade (negativa = ré)
  float    ibat_a;        // corrente de bateria filtrada (negativa = regen)
  float    vbat_v;        // tensão do pack
  float    temp_c;        // temperatura dos MOSFETs
  float    motor_temp_c;  // temperatura interna do MOTOR (NTC no GPIO26)
  float    target;        // alvo aplicado (V no modo torque, rad/s no velocity)
  float    power_scale;   // 0..1 — <1 indica limitação (corrente/temperatura) ativa
  float    i_limit_a;     // eco dos limites atuais (p/ sincronizar a UI)
  float    max_kmh;
  float    v_limit;
  float    zea;           // [rad] ângulo zero elétrico em uso (p/ anotar no config)
  uint32_t loop_hz;       // frequência real do loopFOC (diagnóstico)
  uint8_t  mode;
  bool     enabled;
  uint16_t faults;        // bitmask FAULT_*
  // Diagnóstico dos sensores Hall (p/ validar fiação girando a roda na mão):
  uint8_t  hall_bits;     // estado atual: bitA<<2 | bitB<<1 | bitC
  uint8_t  hall_seen;     // máscara dos estados válidos (1..6) já vistos
  uint16_t hall_bad;      // ressincronizações — só conta com HALL_DIAG=1 (a
                          // reconciliação só existe lá); 0 na build normal
  uint16_t hall_eps;      // bordas CRUAS por segundo — >0 com roda parada = ruído PWM
  uint16_t eps_a;         // bordas aceitas/s no Hall A (os 3 devem ser iguais)
  uint16_t eps_b;         // bordas aceitas/s no Hall B
  uint16_t eps_c;         // bordas aceitas/s no Hall C
  uint16_t seq_skips;     // saltos de setor na sequência = LEITURA ERRADA comprovada
  uint16_t seq_revs;      // inversões da sequência (bounce, se a roda não inverteu)
  uint16_t seq_inval;     // estados 000/111 que entraram na lib
  uint16_t rej_conf_a;    // bordas que demoraram a assentar, por pino —
  uint16_t rej_conf_b;    //   o "medidor de saúde" do sinal de cada Hall
  uint16_t rej_conf_c;    //   (um pino >> outros = fio/canal daquele Hall)
  uint16_t rej_filt_a;    // bordas descartadas pela janela de 100 µs
  uint16_t rej_filt_b;
  uint16_t rej_filt_c;
  uint16_t link_drops;    // ativações do dead-man — tranquinho junto = queda de WiFi
  uint16_t loop_gap_us;   // maior buraco entre iterações do loop FOC no último
                          // segundo — ms em vez de µs = task de controle travando
  // --- energia / instrumentos (F2) ---
  uint32_t odo_m;         // odômetro total [m] (persistido)
  uint32_t trip_m;        // trip parcial [m]
  float    trip_wh;       // Wh consumidos na trip (líquido)
  float    trip_wh_regen; // Wh regenerados na trip
  float    wh_per_km;     // consumo médio da trip
  uint8_t  duty_pct;      // duty de modulação 0..100
  uint8_t  headroom_pct;  // headroom térmico 0..100 (curva térmica)
  uint8_t  ride;          // modo de pilotagem ativo
  bool     saturated;     // tensão no teto (limite de modulação atingido)
  bool     cruise;        // cruise ativo
  bool     walk;          // walk ativo
  bool     locked;        // trava ativa
  bool     headlight;     // farol ligado (eco p/ app e painel)
  uint8_t  cmd_status;    // resultado do último comando/escrita (0=ok, CMDERR_*)
  uint16_t cap_wh;        // capacidade APRENDIDA do pack [Wh] (0 = ainda não)
  uint8_t  soc_pct;       // SoC inteligente (contagem de energia ancorada em
                          // repouso; 0 = indisponível -> usar tensão)
  uint8_t  fw_x2;         // avanço de FW atual [° elétricos x2] (gráfico/app)
  int8_t   mtpa_x10;      // trim MTPA aprendido [° elétricos x10]
  bool     eco_auto;      // cruzeiro econômico automático ativo (modo ECO)
  // --- CORE em tempo real (data-log profundo de debug) ---
  float    vel_est;       // [rad/s mec] estimador interno de velocidade
  float    v_q;           // [V] tensão aplicada eixo q (torque)
  float    v_d;           // [V] tensão aplicada eixo d
  float    i_raw;         // [A] corrente CRUA de BATERIA (antes do filtro)
  float    eff_v_limit;   // [V] teto de tensão efetivo (só modulação x derate —
                          //     o termo de back-EMF foi removido)
  // --- CORRENTE DE FASE MEDIDA (2x INA240A1 low-side) ---
  float    iph_a, iph_b, iph_c;  // [A] correntes de fase (a 3ª é reconstruída)
  float    iph_mag;       // [A] |I| = sqrt(ialpha²+ibeta²) — a grandeza que
                          //     protege: independe do ângulo, então o Hall
                          //     grosso não a contamina
  float    iph_d;         // [A] Id medido (eixo de fluxo; ~0 fora de FW)
  float    iph_q;         // [A] Iq medido (eixo de torque)
  float    iq_target;     // [A] Iq pedido pelo controle (alvo da malha)
  uint8_t  duty_max_pct; // [%] duty de OPERAÇÃO: o que a janela low-side comporta
                         //     conversão — limita o teto de modulação
  bool     iph_ok;        // calibração da corrente de fase VÁLIDA
  uint8_t  iph_ref;       // como o módulo INA240 ligou o REF, 2 bits por canal
                          // (0 bidirecional / 1 REF no GND / 2 REF no V+ /
                          //  3 sem sinal) — canal 1 nos bits 0-1, canal 2 em 2-3
  float    iph_zero1;     // [V] zero medido do canal 1 (diagnóstico do módulo)
  float    iph_zero2;     // [V] zero medido do canal 2
  uint8_t  temp_scale;    // [%] derate térmico isolado
  uint8_t  tracv_scale;   // [%] derate por tensão/SoC isolado
  int8_t   elec_sector;   // setor elétrico atual (0..5) do Hall
  uint8_t  ovr;           // override ativo (0 nenhum,1 trava,2 walk,3 cruise,4 step)
  bool     coast_active;  // regulador de costeira ativo (roda solta)
  float    throttle;      // acelerador CRU do piloto (0..1) — a ENTRADA
  float    brake;         // freio CRU do piloto (0..1)
  float    thr_eff;       // acelerador após a curva (0..1)
  float    brk_eff;       // freio após a curva+regen×vel×tensão (0..1)
  float    vel_target;    // alvo do modo velocidade [rad/s] (cruise/walk/trava)
  float    elec_angle;    // ângulo elétrico usado na comutação [rad]
  float    dc_a, dc_b, dc_c;  // duty aplicado por fase (0..1) — vetor real
  float    shaft_angle;   // ângulo MECÂNICO do sensor [rad]
  float    shaft_vel;     // velocidade da SimpleFOC [rad/s] (vs vel_est)
  float    pscale_fold;   // foldback de corrente ISOLADO (0..1)
  float    eco_v;         // tensão-alvo do eco automático [V]
  float    regen_ilim;    // limite de corrente de regen efetivo [A]
  uint16_t ms_since_cmd;  // ms desde o último comando (dead-man do link)
};

// Erros de comando/validação reportados ao app (cmd_status)
#define CMDERR_OK          0
#define CMDERR_REJECTED    1   // valor/curva fora dos limites físicos
#define CMDERR_NEED_STOP   2   // exige veículo parado (trava, perfis...)
#define CMDERR_BAD_STATE   3   // não permitido no estado atual
// Resultados da AUTO-CALIBRAÇÃO (cmd 24)
#define CMDERR_CAL_OK      4   // auto-cal concluída OK (ZEA/Ke atualizados)
#define CMDERR_CAL_FAIL    6   // ABORTADA: motor não girou (fase aberta/tensão?)
#define CMDERR_NO_IPHASE   7   // sensor de corrente de FASE ausente/fora de faixa

// --- Acesso protegido (implementação em shared_state.cpp) -------------------
Command   readCommand();
void      writeCommand(const Command &c);
// Escrita atômica só de acelerador/freio (painel UART) — não toca nos outros
// campos, então nunca atropela uma mudança concorrente vinda do BLE.
void      writeControls(float throttle, float brake, bool stampDeadman);
void      clearCruiseCmd();  // cancelou/recusou: próximo clique tem borda
void      clearVmaxCmd();    // slider consumido (one-shot por modo)
void      cycleRideCmd();    // botão do painel: próximo modo (eco>nor>turbo)
Telemetry readTelemetry();
void      writeTelemetry(const Telemetry &t);
