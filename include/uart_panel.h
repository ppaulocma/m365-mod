// ============================================================================
// uart_panel.h — link serial de 1 fio com o painel do guidão (Arduino Nano)
// ----------------------------------------------------------------------------
// Task no NÚCLEO 0: manda telemetria a 10 Hz e recebe acelerador/freio.
// Protocolo (9600 8N1, half-duplex num fio):
//   ESP -> Nano:  AA 55 0C  [12 bytes payload]  CRC8
//     payload: i16 kmh*10 | u16 vbat*10 | i16 ibat*10 | i16 potencia_W |
//              i8 temp_C | u8 soc_% | u8 flags(b0 ligado, b1 modo-vel,
//              b2 regen) | u8 faults
//   Nano -> ESP:  BB 66 03  [th 0-255] [br 0-255] [flags(b0 acel ok,
//                            b1 freio ok)]  CRC8
// CRC8 poly 0x07 sobre LEN+payload. Cada lado só procura o cabeçalho da
// direção OPOSTA — o eco da própria transmissão é ignorado de graça.
// ============================================================================
#pragma once
#include "config.h"

#if PANEL_UART_ENABLED
// Task FreeRTOS (criar no núcleo 0)
void panelTask(void *arg);

// true = painel mandou controles há menos de PANEL_TIMEOUT_MS.
// Enquanto ativo, a dashboard NÃO controla acelerador/freio (o físico manda).
bool panelControlsActive();
#else
static inline bool panelControlsActive() { return false; }
#endif
