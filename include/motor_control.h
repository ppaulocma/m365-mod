// ============================================================================
// motor_control.h — task de controle do motor (fixada no NÚCLEO 1)
// ============================================================================
#pragma once

// Task FreeRTOS: inicializa SimpleFOC (driver 3-PWM + Halls) e roda
// loopFOC()/move() o mais rápido possível, com proteções a 1 kHz.
// Criar com xTaskCreatePinnedToCore(..., core 1).
void motorControlTask(void *arg);

// Calibrações remotas (disparadas pelo app via BLE; rodam na task do motor).
// RODA SUSPENSA — o app confirma com o usuário antes de chamar.
void motorRequestCalZea();   // ângulo zero + DIREÇÃO do sensor (gira o motor!)
void motorRequestCalKe();    // mede Ke com rampa até 6 V (roda livre)
void motorRequestIphZero();  // re-mede o ZERO dos INA240 (motor desabilitado e
                             // roda PARADA — corrente tem que ser zero de fato)
void motorRequestAngScan(float amps, float kmh); // varredura do ângulo de
                             // comutação sob rotação (acha o mínimo de corrente
                             // de bateria e salva no P_ZEA) — roda LIVRE
void motorRequestSweep(float kmh, float amps);   // VARREDURA: acelera do
                             // repouso ao teto com CORRENTE FIXA e decompoe a
                             // ondulacao do id em harmonicas do angulo
                             // (1x/2x/6x eletrica e 1x mecanica), cada uma com
                             // o piso do ruido ao lado, + a geometria dos 6
                             // setores de Hall. Responde POR QUE nao esta
                             // liso, nao so quanto. RODA LIVRE, ~6 s.
void motorRequestSpin(float amps, float kmh);   // gira com corrente fixa e mede a
                             // ondulação de VELOCIDADE — a grandeza que
                             // corresponde à aspereza. RODA LIVRE.
void motorPrintHalls();              // forense da leitura dos Halls
void motorRequestVStep(float volts); // degrau de TENSÃO fixa, sem a malha —
                             // separa ondulação da malha de ondulação da
                             // comutação/mecânica/medição
void motorRequestIStep(float amps);  // degrau na malha de CORRENTE, captura a
                             // 1 kHz e desenha no serial. Diz se ela oscila, se
                             // e lenta ou se nem segue o alvo. RODA PRESA.
void motorRequestIphDiag();  // despeja no serial 5 s de leitura CRUA do sensor
void motorRequestCrash();    // despeja a CAIXA-PRETA (512 ms congelados na
                             // 1ª falta que travou; re-arma após despejar)
                             // (contagens de ADC, desvio do zero, correntes)
void motorRequestIphCal();   // CALIBRA a corrente de fase: descobre por medição
                             // o instante de amostragem, o mapa de fases e o
                             // sinal de cada canal. RODA SUSPENSA. Sem isto a
                             // malha de corrente fica bloqueada.

// Energia / odômetro (persistido em NVS; tick do núcleo 0, grava parado)
void motorEnergyInit();
void motorEnergyTick(float kmhAbs);
void motorTripReset();

// Step response p/ tuning de PID (captura 1 kHz; download via BULK)
bool    motorStartStep(float ampKmh);
int     motorCaptureRead(uint16_t startSample, uint8_t *buf, int maxBytes);
void    motorCaptureFree();

// Amostra rápida p/ streaming BLE de alta taxa
void motorFastSample(float *kmh, float *ibat, float *vbat, float *target,
                     uint8_t *duty, int8_t *temp);
void motorPerfSample(uint8_t *fw2, uint8_t *scale, int8_t *mtpa10,
                     uint8_t *head);
// Corrente de fase p/ o stream de tuning: medido x pedido. Sem ver os dois
// juntos não dá para ajustar o PI de corrente.
void motorCurSample(float *iqMeas, float *iqTarget);

// Status de comando/validação p/ o app (CMDERR_* em shared_state.h)
void motorReportCmdStatus(uint8_t st);
