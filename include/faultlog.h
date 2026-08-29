// ============================================================================
// faultlog.h — log persistente de faltas (o quê, quando, com quais valores)
// ============================================================================
#pragma once
#include <Arduino.h>

struct __attribute__((packed)) FaultEntry {
  uint32_t odo_m;     // odômetro no momento [m]          (off 0)
  uint32_t uptime_s;  // tempo ligado [s]                 (off 4)
  uint8_t  faults;    // bits FAULT_* que dispararam      (off 8)
  int16_t  ibat10;    // corrente [A*10]                  (off 9)
  uint16_t vbat10;    // tensão [V*10]                    (off 11)
  int8_t   temp_c;    // temperatura [°C]                 (off 13)
  int16_t  kmh10;     // velocidade [km/h*10]             (off 14)
};                    // exatamente 16 bytes (packed)

void    faultLogInit();
void    faultLogAdd(const FaultEntry &e);
void    faultLogTick(float kmhAbs);     // commit NVS (roda parada)
bool    faultLogGet(uint8_t i, FaultEntry &out);   // 0 = mais recente
void    faultLogClear();
