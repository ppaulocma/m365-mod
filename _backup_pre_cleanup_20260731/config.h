// ============================================================================
// config.h — TODOS os pinos, limiares e calibrações do controlador
// ----------------------------------------------------------------------------
// Tudo que você precisa ajustar está NESTE arquivo. Procure por "AJUSTE AQUI".
//
// Hardware alvo:
//   - ESP32 substituindo o LKS32MC numa controladora M365/Xbot
//   - 3x FD2103S em modo 3-PWM (HIN+LIN unidos => 1 sinal por fase,
//     dead-time e anti-shoot-through são feitos DENTRO do driver)
//   - Halls 5 V passando por conversor de nível -> 3,3 V no ESP32
//   - 1 shunt único no barramento DC (corrente de bateria, NÃO por fase)
// ============================================================================
#pragma once

// ============================================================================
// 1) MODO DE TESTE EM MALHA ABERTA (primeiro teste de bancada!)
// ----------------------------------------------------------------------------
// Coloque 1, grave, e o motor roda em velocity_openloop com tensão baixa,
// SEM usar os Halls. Serve para validar potência + ordem de fases antes de
// fechar a malha. Se o motor "range"/vibra e não gira: troque duas fases
// (inverta dois PIN_PWM_* abaixo). Quando girar liso, volte para 0.
// ============================================================================
#define OPEN_LOOP_TEST            0

#define OPEN_LOOP_VOLTAGE_LIMIT   2.0f   // [V] tensão máx. no teste aberto (baixa!)
#define OPEN_LOOP_MAX_RADS        10.0f  // [rad/s mecânico] alvo com slider a 100%
#define OPEN_LOOP_SLEW            5.0f   // [rad/s²] rampa do alvo em malha aberta

// ============================================================================
// 1b) MODO BANCADA ISOLADO — caça aos congelamentos do loop (gap)
// ----------------------------------------------------------------------------
// Isola o controle do motor de TODAS as variáveis externas: alvo FIXO em
// tensão, sem dead-man, sem comandos da dashboard. Protocolo: liga, conta
// BENCH_START_S segundos no serial e habilita sozinho com BENCH_TARGET_V.
// RODA SUSPENSA! Para parar: E-STOP (se BENCH_BLE=1) ou tirar a alimentação.
//
//   BENCH_TEST 1 + BENCH_BLE 0 -> teste mais PURO: a task de BLE nem é
//        criada (sem dashboard nenhuma — só serial 1x/s).
//   BENCH_TEST 1 + BENCH_BLE 1 -> BLE/app sobem SÓ para telemetria e
//        E-STOP; todos os outros comandos são ignorados.
//
// (Histórico: este modo caçou os congeladores do stack WiFi — hoje o BLE
// no núcleo 0 não compete com o motor, mas o modo fica p/ diagnósticos.)
// ============================================================================
#define BENCH_TEST      0
#define BENCH_BLE      0
#define BENCH_START_S   5        // [s] contagem regressiva antes de ligar
#define BENCH_TARGET_V  12.0f     // [V] alvo fixo de tensão (modo torque)

// ============================================================================
// 1c) AUTOTESTE DE FASES + HALLS — diagnóstico de "não parte / treme parado"
// ----------------------------------------------------------------------------
// Com 1, logo no boot o firmware gira um vetor de tensão em MALHA ABERTA bem
// devagar (2 voltas elétricas ≈ 48° da roda, ~8 s) observando os Halls, e
// imprime no serial um VEREDITO que separa as causas:
//   - rotor não se moveu           -> FASE morta/driver/conector (não é Hall)
//   - girou mas faltam estados     -> HALL ruim (diz quais faltaram)
//   - girou irregular (vai-e-volta)-> uma fase FRACA/morta
//   - tudo certo                   -> problema é na malha fechada (calibração)
// RODA LIVRE! Depois do teste, segue a operação normal.
// ============================================================================
#define SELF_TEST       0

// Teste CRU de hardware: no boot, roda o motor com FOC básico em tensão FIXA
// (sem curvas/eco/MTPA/FW/foldback/rampas/comandos) por HW_TEST_S segundos,
// medindo comutação borda a borda + corrente. Anomalias são impressas no
// instante em que acontecem. Se o defeito aparecer AQUI, é hardware.
// RODA SUSPENSA! Desligue (0) após o diagnóstico.
#define HW_TEST         0
#define HW_TEST_V       6.0f     // [V] tensão fixa do teste (~8-10 km/h)
#define HW_TEST_S       8        // [s] duração da medição
#define SELF_TEST_I_V   1.2f     // [V] vetor DC do teste de simetria de fases
#define SELF_TEST_V     4.0f     // [V] tensão do teste (não mexeu? suba p/ 4-5)

// ============================================================================
// 2) PINOS
// ----------------------------------------------------------------------------
// PWM: 1 sinal por fase (FD2103S com HIN+LIN unidos). Pinos seguros do ESP32
//      (sem função de strapping). Para INVERTER O SENTIDO DE GIRO sem
//      ressoldar: troque dois PIN_PWM_* entre si (ex.: A<->B) E troque os
//      dois PIN_HALL_* correspondentes — ou deixe o initFOC() detectar a
//      direção sozinho (ele compensa por software).
// ADC: TODOS em ADC1 (GPIO 32..39) — ADC2 não funciona com WiFi ligado!
// ============================================================================
#define PIN_PWM_A       19    // fase A -> entrada (HIN+LIN) do FD2103S da fase A
#define PIN_PWM_B       18    // fase B
#define PIN_PWM_C       5    // fase C

// Halls em 33/32/35: pinos FÍSICOS vizinhos no DevKit (35-32-33 em sequência).
// GPIO 35 é entrada-somente e SEM pullup interno — funciona porque o conversor
// de nível empurra o sinal ativamente (por isso o firmware usa pullup EXTERNO
// no SimpleFOC; se algum Hall ficar flutuando em repouso, adicione pullup
// físico de ~10k para 3,3 V).
#define PIN_HALL_A      35    // Hall A (já em 3,3 V via conversor de nível)
#define PIN_HALL_B      32    // Hall B
#define PIN_HALL_C      33    // Hall C (entrada-somente)

// Filtro de glitch dos Halls: o PWM de potência induz pulsos de ns~µs nos
// cabos de Hall que embaralham a comutação (engasgos / trava em ponto de
// Hall / "embanana" em velocidade). Bordas REAIS ficam >= 2,5 ms afastadas
// por pino (até 30 km/h) — qualquer borda mais rápida que isto é descartada.
#define HALL_GLITCH_US  100   // [µs] janela mínima entre bordas aceitas por pino

// Confirmação de borda: pulso de ruído dura ns~poucos µs; borda REAL de Hall
// fica no nível novo. A ISR espera este tempo e RELÊ o pino — nível voltou =
// pulso falso, ignorado por inteiro (não suja o estado nem o filtro acima).
// É a defesa contra o ruído forte de BAIXA velocidade (corrente máxima).
// 0 = DESLIGA a confirmação (use p/ testar se ela atrapalha a partida).
#define HALL_CONFIRM_US 3     // [µs] espera de confirmação dentro da ISR

// Diagnóstico dos Halls: 1 = forense completa (verificador de sequência,
// mapa /hallmap, contadores por pino, reconciliação, card da dashboard);
// 0 = MODO MÍNIMO: interrupção -> filtro -> espera assentar -> biblioteca,
// e nada mais. Com o hardware saneado (pullup 1k + 1 nF) o mínimo basta;
// religue o 1 quando quiser investigar qualquer sintoma novo.
#define HALL_DIAG       0

// Diagnóstico geral via serial (1 Hz, com motor ligado ou roda girando):
// modo/override/trava, alvo vs tensão aplicada, corrente, faltas. Ligue para
// caçar comportamento estranho (aspereza, dentes, freio fantasma); desligue
// depois — é texto no núcleo 1.
#define DIAG_SERIAL     0

#define PIN_ADC_CURRENT 34    // saída amplificada do shunt de barramento (ADC1)
#define PIN_ADC_VBAT    36    // divisor da tensão do pack (ADC1, pino "VP")
#define PIN_ADC_NTC     39    // NTC junto aos MOSFETs (ADC1, pino "VN")

// ============================================================================
// 3) MOTOR / PWM
// ============================================================================
#define MOTOR_POLE_PAIRS        15       // hub M365 = 15 pares de polo
#define PWM_FREQUENCY_HZ        25000    // 25 kHz: inaudível e ok p/ FD2103S

// AJUSTE AQUI: tensão nominal do pack (10s Li-Ion: ~36 V nominal, 42 V cheio).
// Use a tensão TÍPICA de operação; é usada no cálculo do duty do PWM.
#define VOLTAGE_POWER_SUPPLY    40.0f    // [V]

// Teto de modulação (fração de Vsupply que vira tensão de fase).
// IMPORTANTE — limite de HARDWARE: os FD2103S são drivers BOOTSTRAP: o gate
// do MOSFET de cima é alimentado por um capacitor que SÓ recarrega quando o
// de baixo conduz. Modulação a 100% (0.577 = máximo do seno) deixa o
// high-side ligado tempo demais no pico da senoide -> o capacitor descarrega,
// o gate afunda e a fase falha — é o "se perde/tranca/faz barulho" perto da
// velocidade máxima. 0.55 (~95% de duty máx.) garante ~2 µs de recarga do
// bootstrap por ciclo de PWM. NÃO suba além de ~0.56 com este estágio.
#define MODULATION_CEIL_FACTOR  0.55f

// Limite de tensão de fase aplicado ao motor (controla o "torque máximo").
// Começa BAIXO por segurança; o valor é ajustável ao vivo pela dashboard
// até VOLTAGE_LIMIT_MAX. Suba aos poucos conforme ganhar confiança.
#define VOLTAGE_LIMIT_DEFAULT   36.0f     // [V] valor inicial (partida segura)
#define VOLTAGE_LIMIT_MAX       42.0f    // [V] teto absoluto da dashboard

// Tensão usada na calibração do initFOC (alinhamento do sensor).
// AJUSTE AQUI: o hub da M365 tem resistência de fase baixa (~0,2 Ω) =>
// 1,5 V já dá vários ampères. Se a calibração falhar, suba um pouco.
// Faça a 1ª calibração com fonte de bancada limitada em ~5 A e RODA SUSPENSA.
#define VOLTAGE_SENSOR_ALIGN    1.5f     // [V]

// --- Teto de corrente por FEEDFORWARD de back-EMF (P_ILIM_FF liga/desliga) ----
// ATENÇÃO: NÃO é o antigo "limitador com rampa" (removido). É o teto instantâneo
// V <= P_HEADROOM + Ke*ω aplicado no controlSlice(). O shunt do INA226 mede a
// corrente da BATERIA (barramento DC); a corrente de FASE parado/baixa rotação
// é muito maior e o foldback nem a enxerga. Este teto é o que segura o PICO de
// fase na largada. Liga/desliga pelo parâmetro P_ILIM_FF (default LIGADO).
// DESLIGAR = confiar só no foldback de bateria: mais torque bruto na arrancada,
// porém risco de pico de fase — só desligue sabendo disso.
// O firmware sempre corta voltage_limit acima de ~Vsupply/sqrt(3) (teto físico
// da modulação senoidal — acima disso é só distorção), independente deste teto.

// INTERPOLAÇÃO DO ÂNGULO DOS HALLS: entre transições, o ângulo elétrico avança
// suave pela velocidade medida em vez de pular 60°. Comutação vira senoidal =
// motor bem mais liso, menos ripple e menos "travamentinhos". Erro limitado a
// 1 setor (nunca pior que o degrau cru). SEMPRE compilada; liga/desliga AO VIVO
// pelo parâmetro P_HALL_INTERP (default LIGADO) — pode desligar na rua se sentir
// instabilidade, sem regravar.
#define HALL_INTERP_ENABLED     1
#define HALL_INTERP_MIN_VEL     2.0f   // [rad/s mec] abaixo disso não interpola
#define MOTOR_KE_V_PER_RADS     0.32f   // [V/(rad/s)] back-EMF MEDIDA na dashboard:
                                        // 3,8 V @ 4,9 km/h (12,6 rad/s) => Ke ~ 0,30.
                                        // 0,32 = medido + folga. Se aparecer
                                        // "tranquinho" rítmico em alta rotação,
                                        // suba de 0,02 em 0,02. Ke alto demais não
                                        // dá sintoma — só afrouxa o limite de
                                        // corrente escondido. Não exagere.
#define STALL_V_HEADROOM        8.0f    // [V] margem acima da back-EMF no arranque.
                                        // Com o INA226 LIGADO, o foldback real
                                        // (P_ILIM) é quem limita a corrente — este
                                        // headroom só precisa ser alto o bastante
                                        // p/ ALCANÇAR esse limite parado. 4 V dava
                                        // partida fraca; 8 V dá torque de arranque
                                        // de verdade (o foldback corta o excesso).
// COSTEIRA (roda-solta ao soltar o acelerador) — liga/desliga por P_COAST.
#define COAST_I_GAIN            3.0f    // [V/(A*s)] ganho do regulador de COSTEIRA
#define COAST_TARGET_A          0.4f    // [A] alvo de corrente ao soltar: um tico
                                        // NEGATIVO (regen levíssimo) garante ZERO
                                        // tração mesmo com offset do sensor — a
                                        // roda solta de verdade, nunca empurra

// --- Resultado da calibração do initFOC -------------------------------------
// Na primeira execução, o initFOC() gira o motor e imprime no serial o
// "zero electric angle" e a direção do sensor. Anote e cole aqui para os
// próximos boots NÃO precisarem girar o motor para calibrar.
#define MOTOR_ZERO_ELEC_ANGLE   4.19f      // AJUSTE AQUI: ex. 2.09f (NAN = auto)
#define MOTOR_SENSOR_DIR        1       // 0 = auto-detectar, 1 = CW, -1 = CCW

// ============================================================================
// 4) RODA / VELOCIDADE
// ============================================================================
#define WHEEL_DIAMETER_M        0.216f   // [m] roda 8,5" da M365. AJUSTE se outra.
#define MAX_KMH_ABS             50.0f    // [km/h] teto absoluto da dashboard
#define MAX_KMH_DEFAULT         30.0f    // [km/h] limite de velocidade (rua)

// ============================================================================
// 5) CORRENTE (shunt único de barramento)
// ----------------------------------------------------------------------------
// CHAVE GERAL: 0 = leitura de corrente DESATIVADA (shunt ainda não ligado/
// calibrado). Desliga o autozero do boot, o foldback e a proteção de
// sobrecorrente; a dashboard mostra 0,0 A. Subtensão e temperatura seguem
// ativas. ATENÇÃO: sem isso NÃO há limite de corrente por firmware — use
// SEMPRE fonte de bancada com limite de corrente e freie suave!
// Volte para 1 quando o sinal do shunt estiver no PIN_ADC_CURRENT.
// ============================================================================
#define CUR_SENSE_ENABLED       1

// --- Fonte da leitura de corrente -------------------------------------------
// 1 = INA226 via I2C (recomendado): task própria no núcleo 0, ~1 kHz.
// 0 = ADC analógico legado (amplificador do shunt no PIN_ADC_CURRENT).
// LIGAÇÃO DO INA226 (módulo): VCC->3V3, GND->GND, SDA->21, SCL->22,
// A0/A1->GND (addr 0x40), IN+/IN- POR CIMA DO SHUNT ORIGINAL da placa
// (lado baixo! IN+/IN- máx. 36 V de modo comum) e VBUS->GND (máx. 36 V,
// NUNCA no pack). NÃO use o shunt R100 (0,1 Ω) que vem no módulo: a 20 A
// ele vira fumaça — o shunt de ~1 mΩ da placa dá fundo de escala ±82 A.
#define CUR_SENSE_INA226        1

// Filtros de EXIBIÇÃO (telemetria/app/painel) — as PROTEÇÕES continuam no
// sinal rápido (foldback/OC em ms); só o que você VÊ fica calmo.
#define IBAT_DISP_TAU_S         0.8f   // [s] corrente exibida (medidor de EV)
#define TEMP_DISP_TAU_S         3.0f   // [s] temperatura exibida
#define INA226_ADDR             0x40   // A0 e A1 no GND
#define INA226_SHUNT_MOHM       4.3f   // [mΩ] AJUSTE: resistência REAL do shunt.
                                       // Calibre: compare a corrente da dashboard
                                       // com a da fonte e corrija este valor
                                       // proporcionalmente (lido alto = suba).
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22

// AJUSTE AQUI (calibração, quando reativar): com o motor sob carga conhecida,
// compare a corrente lida pela fonte/alicate com a mostrada na dashboard e
// corrija CUR_MV_PER_A. Procedimento: I_real = (mV_adc - offset) / CUR_MV_PER_A.
// O offset (leitura com 0 A) é medido automaticamente no boot — por isso
// NÃO deixe o motor girando/carregado ao ligar o ESP32.
#define CUR_MV_PER_A            13.2f    // [mV/A] AJUSTE AQUI: ganho do amp. do shunt
#define CUR_INVERT              0        // 1 se corrente positiva aparecer negativa
#define CUR_AUTOZERO            1        // 1 = mede offset no boot (motor parado!)
#define CUR_OFFSET_MV_DEFAULT   0.0f     // [mV] usado se CUR_AUTOZERO = 0

// Limite de corrente de bateria (foldback suave — reduz potência sem desligar)
#define I_LIMIT_DEFAULT_A       18.0f    // [A] limite de corrente de bateria (rua)
#define I_LIMIT_MAX_A           30.0f    // [A] teto que a dashboard permite pedir
#define REGEN_I_LIMIT_A         6.0f     // [A] corrente máx. de REGENERAÇÃO (freio)
                                         //     AJUSTE: se o amp. do shunt for
                                         //     unidirecional, regen não é medida —
                                         //     mantenha freio conservador!

// Proteção de SOBRECORRENTE dura (desliga e trava até "limpar faltas")
#define OC_TRIP_A               25.0f    // [A] AJUSTE AQUI conforme seus MOSFETs
#define OC_TRIP_MS              100      // [ms] tempo acima do limiar p/ disparar

// Dinâmica do foldback (normalmente não precisa mexer)
#define FOLDBACK_GAIN           2.0f     // [1/(A*s)] velocidade de corte
#define FOLDBACK_RECOVERY       0.5f     // [1/s] velocidade de recuperação

// ============================================================================
// 6) TENSÃO DA BATERIA (divisor resistivo)
// ----------------------------------------------------------------------------
// CHAVE GERAL: 0 = divisor AINDA NÃO LIGADO no pino. Desliga a leitura e a
// proteção de subtensão (um pino de ADC flutuando lê lixo e dispara falta
// à toa!). A dashboard mostra 0,0 V. Volte para 1 quando ligar o divisor.
// ============================================================================
#define VBAT_SENSE_ENABLED      1

// AJUSTE AQUI (quando reativar): valores REAIS do seu divisor. Calibre
// medindo o pack com multímetro e ajustando VBAT_CAL_GAIN até bater.
#define VBAT_R_TOP              100000.0f // [Ω] resistor de cima (pack -> pino)
#define VBAT_R_BOTTOM           4700.0f   // [Ω] resistor de baixo (pino -> GND)

// Filtro da leitura (amostrada a cada 25 ms): peso da amostra nova no EMA.
// 0.015 => constante de tempo ~1,7 s — display firme, sem flutuação.
// A proteção de subtensão não sofre: ela já exige afundamento SUSTENTADO
// (UV_TRIP_MS) + histerese, então filtro lento = menos falso disparo.
#define VBAT_LPF_ALPHA          0.015f
#define VBAT_CAL_GAIN           1.000f    // fator fino de calibração

#define UV_CUTOFF_V             32.0f    // [V] subtensão: corta (10s: 3,2 V/célula)
#define UV_RECOVER_V            34.0f    // [V] histerese p/ permitir limpar a falta
#define UV_TRIP_MS              800      // [ms] filtro (ignora afundamento breve)
#define UV_BOOT_GRACE_MS        3000     // [ms] carência pós-boot: não avalia
                                         // subtensão enquanto filtro/sensores
                                         // ainda estão assentando

// ============================================================================
// 7) TEMPERATURA (NTC junto aos MOSFETs)
// ----------------------------------------------------------------------------
// CHAVE GERAL: 0 = NTC AINDA NÃO LIGADO no pino. Desliga a leitura, o corte
// térmico e o derate (pino flutuando lê lixo e pode "medir" 150 °C!).
// A dashboard mostra 0 °C. Volte para 1 quando ligar o NTC.
// ============================================================================
#define NTC_SENSE_ENABLED       1

// AJUSTE AQUI (quando reativar): parâmetros do SEU NTC e topologia.
// Assumido (padrão seguro): NTC entre o pino e GND, resistor série p/ 3,3 V.
// Se for o contrário (NTC no lado de cima), mude NTC_TO_GND para 0.
#define NTC_BETA                3950.0f  // AJUSTE AQUI: beta do seu NTC
#define NTC_R25                 10000.0f // [Ω] resistência a 25 °C
#define NTC_SERIES_R            10000.0f // [Ω] resistor série do divisor
#define NTC_TO_GND              1        // 1 = NTC p/ GND; 0 = NTC p/ 3,3 V

#define TEMP_DERATE_C           70.0f    // [°C] começa a reduzir potência
#define TEMP_CUTOFF_C           90.0f    // [°C] corta e trava (falta)
#define OT_TRIP_MS              1000     // [ms] filtro do corte térmico

// ============================================================================
// 8) COMPORTAMENTO DE CONTROLE (rampas, freio, PID de velocidade)
// ============================================================================
#define TORQUE_SLEW_V_S         90.0f    // [V/s] rampa do acelerador: RESPOSTA
#define TORQUE_RELEASE_V_S      150.0f   // [V/s] rampa ao SOLTAR/reduzir: rápida

                                         // (soltar o acelerador tem que cortar
                                         // já — 48 V/s dava ~0,4 s de "delay")
#define ACCEL_RADS2             25.0f    // [rad/s²] aceleração (modo velocidade)
#define DECEL_RADS2             8.0f     // [rad/s²] "freio motor" ao soltar acelerador
#define BRAKE_DECEL_RADS2       40.0f    // [rad/s²] desaceleração com freio a 100%
#define BRAKE_MIN_SPEED_RADS    2.0f     // [rad/s] abaixo disso o freio vira alvo 0
                                         //   (evita o motor girar PARA TRÁS no fim
                                         //    da frenagem em modo torque)
// Limite de velocidade (modo torque) por TETO DE TENSÃO, em malha aberta:
// a tensão aplicada não passa de Ke*vel_máx + margem => o motor converge
// suavemente para a vel. máx. SEM realimentar velocidade (o corte proporcional
// antigo oscilava — surto rítmico ao encostar no limite). O limite fica
// APROXIMADO (depende da precisão do Ke): margem maior = passa um pouco mais
// do limite; margem menor = chega mais devagar na vel. máx.
#define SPEED_CAP_MARGIN_V      0.5f     // [V] margem acima da back-EMF da vel. máx.

// PID de velocidade (modo velocity, saída em volts). AJUSTE fino na prática.
// Halls têm baixa resolução -> filtro LPF_velocity relativamente alto.
#define VEL_PID_P               0.2f
#define VEL_PID_I               1.5f
#define VEL_PID_D               0.0f
#define VEL_PID_RAMP            100.0f   // [V/s] rampa interna da saída do PID
#define VEL_LPF_TF              0.10f    // [s] filtro da velocidade medida — Halls
                                         //     são "grossos": suba se der tranco,
                                         //     desça se a resposta ficar mole

// ============================================================================
// 8b) SIMPLEFOC STUDIO / COMMANDER (ajuste fino pela USB)
// ----------------------------------------------------------------------------
// Com 1, o firmware expõe o motor no protocolo "Commander" da SimpleFOC na
// serial USB (115200). Use o SimpleFOC Studio (GUI oficial:
// github.com/JorgeMaker/SimpleFOCStudio) para gráficos ao vivo e ajuste de
// PID/filtros em tempo real — o motor está registrado na letra 'M'.
// ATENÇÃO: alvo, enable, voltage_limit e velocity_limit são reescritos pelo
// loop de controle (a dashboard manda neles). Use o Studio para MONITORAR e
// ajustar GANHOS; use a dashboard para acelerar/frear/limites.
// ============================================================================
#define COMMANDER_ENABLED       0        // 0 = app cobre o tuning (stream +
                                         // step response) — sem polling serial
                                         // no laço de 1 kHz e menos flash.
                                         // Religue p/ usar o SimpleFOC Studio.

// ============================================================================
// 8c) MODOS DE PILOTAGEM / CRUISE / WALK / ENERGIA
// ----------------------------------------------------------------------------
// Cada modo limita corrente, velocidade e suaviza a rampa. O TURBO usa os
// limites configurados no app sem reduções (e é o único que permite field
// weakening). Valores AJUSTÁVEIS aqui; seleção em tempo real pelo app.
// ============================================================================
#define RIDE_ECO_ILIM_A      10.0f
#define RIDE_ECO_VMAX_KMH    18.0f
#define RIDE_ECO_SLEW_MUL    0.55f
#define RIDE_NORMAL_ILIM_A   18.0f
#define RIDE_NORMAL_VMAX_KMH 25.0f
#define RIDE_NORMAL_SLEW_MUL 0.85f
// TURBO: sem tetos próprios (usa os limites do app) e FW liberado

#define CRUISE_MIN_KMH       5.0f    // só engata acima disso
#define WALK_KMH             4.0f    // alvo do walk mode
#define WALK_ILIM_A          5.0f    // corrente máx. no walk
#define LOCK_ILIM_A          8.0f    // corrente máx. da trava eletrônica

#define UV_WARN_V            33.5f   // [V] aviso de bateria fraca (antes do corte)

// --- Luzes -------------------------------------------------------------------
// LUZ DE FREIO no ESP (GPIO 23 -> gate de MOSFET N -> LED- ; LED+ no 5V/12V).
// Acende no manete OU em regen forte; com o farol ligado vira lanterna fraca
// (PWM). O FAROL fica no NANO (D6) e é comandado pelo app via flag do link.
#define BRAKE_LIGHT_ENABLED     1
#define PIN_BRAKE_LIGHT         23
#define BRAKE_LIGHT_LEDC_CH     7      // canal LEDC livre (MCPWM é separado)
#define BRAKE_LIGHT_DIM_DUTY    28     // [0-255] lanterna (com farol ligado)
#define BATT_VMAX_V          42.0f   // [V] tensão de carga cheia: regen CORTA
                                     // acima disso (camada dura sobre a curva)

// Eco automático (modo ECO): em cruzeiro estável, o firmware assume o comando
// e desce a tensão até o mínimo que segura a velocidade dentro da tolerância.
#define ECO_AUTO_TOL_KMH     1.5f    // [km/h] afundamento tolerado (economia)
#define ECO_AUTO_ENGAGE_MS   3000    // [ms] de cruzeiro estável p/ engatar
#define ECO_AUTO_TRIM_V_S    0.06f   // [V/s] descida lenta da tensão
#define ECO_AUTO_RECOV_V_S   0.50f   // [V/s] recuperação quando afunda demais

// ============================================================================
// 9) BLUETOOTH (app React Native) — substituiu a dashboard web/WiFi
// ----------------------------------------------------------------------------
// Servidor BLE (NimBLE) no núcleo 0: telemetria por notify a 10 Hz e
// comandos por write (protocolo em ble_dashboard.h — os mesmos UUIDs estão
// no app, pasta "Patinente APP"). Sem WiFi no firmware: menos RAM/flash e
// picos de corrente 3x menores no 3,3 V (os Halls agradecem).
// ============================================================================
#define BLE_DEVICE_NAME         "PatineteESP"
#define BLE_TELEM_MS            100      // [ms] período do notify de telemetria

// Dead-man: sem comando (painel físico OU app) nesse intervalo, o alvo é
// zerado. Protege contra queda de link com acelerador apertado.
#define CMD_TIMEOUT_MS          600      // [ms]

// ============================================================================
// 9b) PAINEL FÍSICO NO GUIDÃO (Arduino Nano) — acelerador, freio e display
// ----------------------------------------------------------------------------
// Link de UM FIO (half-duplex, 9600 baud): o ESP32 manda telemetria a 10 Hz
// e o Nano responde com acelerador/freio. Cabeçalhos distintos por direção
// (AA55 / BB66) + CRC8 — cada lado ignora o próprio eco naturalmente.
// Com o painel ATIVO (frame fresco), o acelerador/freio da dashboard são
// ignorados (o físico manda); e-stop e limites da dashboard continuam valendo.
// Sem frames por PANEL_TIMEOUT_MS, o dead-man existente zera o alvo.
// Fiação: ver README ("Painel"). Linha em nível 5 V no cabo; canal sobrando
// do módulo BSS138 desce p/ 3,3 V no lado do ESP32.
// ============================================================================
#define PANEL_UART_ENABLED  1
#define PIN_PANEL_RX        16     // RX2 do ESP32 (liga direto na linha 3,3 V)
#define PIN_PANEL_TX        17     // TX2 (liga na linha ATRAVÉS do diodo 1N4148)
#define PANEL_BAUD          9600
#define PANEL_PERIOD_MS     100    // [ms] ciclo telemetria->resposta (10 Hz)
#define PANEL_TIMEOUT_MS    400    // [ms] painel "vivo" = frame mais novo que isso
#define PANEL_CTRL_STALE_MS 250    // [ms] sem resposta do painel por isso => zera
                                   // acelerador/freio na hora (não fica segurando
                                   // o último valor até o dead-man de 600 ms)

// ============================================================================
// 10) FREERTOS
// ============================================================================
#define MOTOR_TASK_PRIO         20       // núcleo 1 — controle. 20 = acima de
                                         // TODAS as tasks de rede (tcpip=18,
                                         // async_tcp=10): nenhuma consegue
                                         // preemptar o loop FOC, mesmo que
                                         // alguma migre de núcleo. Abaixo só
                                         // do ipc1/eventos (24, raros/breves).
#define WEB_TASK_PRIO           1        // núcleo 0 — WiFi/dashboard
#define MOTOR_TASK_STACK        8192
#define WEB_TASK_STACK          8192
