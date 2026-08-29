// ============================================================================
// ina226.h — leitura de corrente de bateria via INA226 (I2C)
// ----------------------------------------------------------------------------
// Task dedicada no NÚCLEO 0: I2C é bloqueante e um barramento travado pode
// segurar a task por ms — jamais pode rodar no núcleo do motor. O núcleo 1
// só consome o último valor via ina226CurrentA() (cópia sob spinlock).
// ============================================================================
#pragma once
#include "config.h"

#if CUR_SENSE_ENABLED || VBAT_SENSE_ENABLED || NTC_SENSE_ENABLED || MOTOR_NTC_SENSE_ENABLED
// Task de sensores lentos (criar com xTaskCreatePinnedToCore no núcleo 0):
// INA226 (I2C) + ADCs de tensão/NTC — NADA de ADC/I2C no núcleo do motor.
void ina226Task(void *arg);
#endif

#if VBAT_SENSE_ENABLED
float sensVbatV();    // última tensão do pack [V]
#endif
#if NTC_SENSE_ENABLED
float sensTempC();    // última temperatura dos MOSFETs [°C]
#endif
#if MOTOR_NTC_SENSE_ENABLED
float sensMotorTempC();   // última temperatura interna do MOTOR [°C]
#endif

#if CUR_SENSE_ENABLED

// Última corrente lida [A] (positiva = tração, negativa = regen).
// Retorna 0.0 se a leitura estiver velha (>100 ms) — sensor morto não pode
// gerar foldback/proteção falsos.
float ina226CurrentA();

// true = sensor respondendo (leitura fresca)
#endif
