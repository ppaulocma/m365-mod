// ============================================================================
// motor_control.h — task de controle do motor (fixada no NÚCLEO 1)
// ============================================================================
#pragma once

// Task FreeRTOS: inicializa SimpleFOC (driver 3-PWM + Halls) e roda
// loopFOC()/move() o mais rápido possível, com proteções a 1 kHz.
// Criar com xTaskCreatePinnedToCore(..., core 1).
void motorControlTask(void *arg);

// Monta o JSON do mapa de duração de setores (90 slots por volta mecânica)
// e ZERA o mapa. Usado pelo endpoint /hallmap. Retorna o tamanho escrito.
// Buffer recomendado: >= 1400 bytes.
int hallMapReport(char *buf, size_t n);

// Calibrações remotas (disparadas pelo app via BLE; rodam na task do motor).
// RODA SUSPENSA — o app confirma com o usuário antes de chamar.
void motorRequestCalZea();   // realinha o ângulo zero (gira o motor!)
void motorRequestCalKe();    // mede Ke com 6 V fixos por 4 s

// Energia / odômetro (persistido em NVS; tick do núcleo 0, grava parado)
void motorEnergyInit();
void motorEnergyTick(float kmhAbs);
void motorTripReset();

// Step response p/ tuning de PID (captura 1 kHz; download via BULK)
bool    motorStartStep(float ampKmh);
uint8_t motorCaptureState();               // 0 livre | 1 capturando | 2 pronto
int     motorCaptureRead(uint16_t startSample, uint8_t *buf, int maxBytes);
void    motorCaptureFree();

// Amostra rápida p/ streaming BLE de alta taxa
void motorFastSample(float *kmh, float *ibat, float *vbat, float *target,
                     uint8_t *duty, int8_t *temp);
void motorPerfSample(uint8_t *fw2, uint8_t *scale, int8_t *mtpa10,
                     uint8_t *head);

// Status de comando/validação p/ o app (CMDERR_* em shared_state.h)
void motorReportCmdStatus(uint8_t st);
