// ============================================================================
// ina226.cpp — driver mínimo do INA226 por registrador (sem lib externa)
// ----------------------------------------------------------------------------
// Configuração: conversão de shunt de 204 µs com média de 4 amostras, modo
// contínuo só-shunt => valor novo a cada ~0,8 ms. A task lê a ~1 kHz e
// publica; o núcleo 1 consome sem nunca tocar no I2C.
//
// Conversão: Vshunt = raw * 2,5 µV  ->  I = Vshunt / R_shunt
// Com o shunt de 1 mΩ da placa: fundo de escala ±81,92 mV = ±81,9 A.
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "ina226.h"
#include "params.h"
#include "phase_current.h"   // phaseCurrentSarPowerAssert (trava SAR/errata)
#include <math.h>

// Este módulo virou a TASK DE SENSORES LENTOS (núcleo 0): além do INA226,
// lê os ADCs de tensão e NTC. Motivo: analogReadMilliVolts() usa seções
// críticas internas (interrupções desligadas por dezenas de µs) — no núcleo
// do motor isso atrasava as ISRs dos Halls e gerava skips em baixa rotação.
// O núcleo 1 agora só consome valores prontos via accessors.

#if CUR_SENSE_ENABLED || VBAT_SENSE_ENABLED || NTC_SENSE_ENABLED || MOTOR_NTC_SENSE_ENABLED

static portMUX_TYPE s_sensMux = portMUX_INITIALIZER_UNLOCKED;

#if VBAT_SENSE_ENABLED
static float s_vbatV = 0.0f;
float sensVbatV() {
  taskENTER_CRITICAL(&s_sensMux);
  float v = s_vbatV;
  taskEXIT_CRITICAL(&s_sensMux);
  return v;
}
static float readVbatOnce() {
  float acc = 0;                       // média (ADC2 é ruidoso)
  for (int i = 0; i < 16; i++) acc += (float)analogReadMilliVolts(PIN_ADC_VBAT);
  float mv = acc / 16.0f;
  return (mv / 1000.0f) * ((VBAT_R_TOP + VBAT_R_BOTTOM) / VBAT_R_BOTTOM) * VBAT_CAL_GAIN;
}
#endif

#if NTC_SENSE_ENABLED || MOTOR_NTC_SENSE_ENABLED
// Média de N leituras — o ADC2 é mais ruidoso que o ADC1; oversampling reduz
// o tremor (o ruído cai ~√N).
static float avgMv(uint8_t pin) {
  float acc = 0;
  for (int i = 0; i < 16; i++) acc += (float)analogReadMilliVolts(pin);
  return acc / 16.0f;
}
// Converte mV do divisor NTC (beta) em °C. Cada sensor passa os SEUS parâmetros
// (beta/R25/série/topologia) — os dois NTCs são MODELOS DIFERENTES.
static float ntcMvToC(float v, float beta, float r25, float seriesR, bool toGnd) {
  v = constrain(v, 0.005f, 3.25f);
  float r = toGnd ? seriesR * v / (3.3f - v) : seriesR * (3.3f - v) / v;
  float tK = 1.0f / (1.0f / 298.15f + logf(r / r25) / beta);
  return tK - 273.15f;
}
#endif

#if NTC_SENSE_ENABLED
static float s_tempC = 0.0f;
float sensTempC() {
  taskENTER_CRITICAL(&s_sensMux);
  float t = s_tempC;
  taskEXIT_CRITICAL(&s_sensMux);
  return t;
}
static float readTempOnce() {
  return ntcMvToC(avgMv(PIN_ADC_NTC) / 1000.0f,
                  NTC_BETA, NTC_R25, NTC_SERIES_R, NTC_TO_GND);
}
#endif

#if MOTOR_NTC_SENSE_ENABLED
static float s_motorTempC = 0.0f;
float sensMotorTempC() {
  taskENTER_CRITICAL(&s_sensMux);
  float t = s_motorTempC;
  taskEXIT_CRITICAL(&s_sensMux);
  return t;
}
static float readMotorTempOnce() {
  return ntcMvToC(avgMv(PIN_ADC_MOTOR_NTC) / 1000.0f,
                  MOTOR_NTC_BETA, MOTOR_NTC_R25, MOTOR_NTC_SERIES_R, MOTOR_NTC_TO_GND);
}
#endif

#if CUR_SENSE_ENABLED

// Registradores do INA226
#define INA_REG_CONFIG  0x00
#define INA_REG_SHUNT   0x01   // 2,5 µV/LSB, com sinal

static float        s_currentA = 0.0f;
static uint32_t     s_lastOkMs = 0;

static bool writeReg(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  return Wire.endTransmission() == 0;
}

static bool readReg(uint8_t reg, uint16_t &out) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;      // repeated start
  if (Wire.requestFrom((int)INA226_ADDR, 2) != 2) return false;
  out = ((uint16_t)Wire.read() << 8) | (uint16_t)Wire.read();
  return true;
}

#if I2C_SCAN
// Diagnóstico: escaneia o barramento e loga os endereços que respondem.
// Ligue I2C_SCAN no config, grave, veja o serial e ponha o valor no INA226_ADDR.
static void i2cScan() {
  Serial.print("[i2c] scan:");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); found++; }
  }
  Serial.println(found ? "  <- ponha no INA226_ADDR" : " (nenhum!)");
}
#endif

float ina226CurrentA() {
  taskENTER_CRITICAL(&s_sensMux);
  float    a = s_currentA;
  uint32_t t = s_lastOkMs;
  taskEXIT_CRITICAL(&s_sensMux);
  // Leitura velha = sensor morto/travado: devolve 0 A para o foldback e a
  // proteção ficarem inertes (como antes do sensor), nunca dispararem à toa.
  return (millis() - t < 100) ? a : 0.0f;
}

#endif  // CUR_SENSE_ENABLED

void ina226Task(void *arg) {
#if VBAT_SENSE_ENABLED
  analogSetPinAttenuation(PIN_ADC_VBAT, ADC_11db);
#endif
#if NTC_SENSE_ENABLED
  analogSetPinAttenuation(PIN_ADC_NTC, ADC_11db);
#endif
#if MOTOR_NTC_SENSE_ENABLED
  analogSetPinAttenuation(PIN_ADC_MOTOR_NTC, ADC_11db);
#endif
#if CUR_SENSE_ENABLED
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  Wire.setTimeOut(5);           // barramento travado não pode prender a task

#if I2C_SCAN
  i2cScan();                    // só diagnóstico: loga os endereços do barramento
#endif

  // Reset + configuração:
  // [15]=RST | AVG[11:9]=001 (média de 4) | VBUSCT[8:6]=000 (mín., não usado)
  // VSHCT[5:3]=001 (204 µs) | MODE[2:0]=101 (shunt contínuo)
  writeReg(INA_REG_CONFIG, 0x8000);
  vTaskDelay(pdMS_TO_TICKS(2));
  bool ok = writeReg(INA_REG_CONFIG, (0b001 << 9) | (0b000 << 6) | (0b001 << 3) | 0b101);
  // Shunt EM USO = parâmetro P_SHUNT_MOHM (a NVS vence o config.h). O aviso
  // de divergência com o default saiu daqui: o paramsDump() do boot já marca
  // com '*' todo parâmetro fora do default, este incluído.
  const float shunt = prm(P_SHUNT_MOHM);
  Serial.printf("[ina226] init %s (addr 0x%02X, shunt EM USO %.3f mOhm, "
                "escala ±%.0f A)\n",
                ok ? "OK" : "FALHOU — confira SDA/SCL/endereco (ligue I2C_SCAN)",
                INA226_ADDR, shunt, (double)(81.92f / shunt));
  uint32_t errStreak = 0;
#endif

  uint32_t n = 0;
  for (;;) {
    n++;
#if VBAT_SENSE_ENABLED
    if (n % 25 == 0) {                  // 40 Hz — sinal lento
      float v = readVbatOnce();
      phaseCurrentSarPowerAssert();     // o read do ADC2 desfaz a trava do SAR
      taskENTER_CRITICAL(&s_sensMux); s_vbatV = v; taskEXIT_CRITICAL(&s_sensMux);
    }
#endif
#if NTC_SENSE_ENABLED
    if (n % 25 == 12) {
      float t = readTempOnce();
      phaseCurrentSarPowerAssert();     // idem: reafirma a trava pós-ADC2
      taskENTER_CRITICAL(&s_sensMux); s_tempC = t; taskEXIT_CRITICAL(&s_sensMux);
    }
#endif
#if MOTOR_NTC_SENSE_ENABLED
    if (n % 25 == 18) {
      float t = readMotorTempOnce();
      phaseCurrentSarPowerAssert();     // idem: reafirma a trava pós-ADC2
      taskENTER_CRITICAL(&s_sensMux); s_motorTempC = t; taskEXIT_CRITICAL(&s_sensMux);
    }
#endif
#if CUR_SENSE_ENABLED
    uint16_t raw;
    if (readReg(INA_REG_SHUNT, raw)) {
      errStreak = 0;
      float a = (float)(int16_t)raw * 2.5e-3f / prm(P_SHUNT_MOHM); // 2,5 µV/LSB
#if CUR_INVERT
      a = -a;
#endif
      taskENTER_CRITICAL(&s_sensMux);
      s_currentA = a;
      s_lastOkMs = millis();
      taskEXIT_CRITICAL(&s_sensMux);
    } else if (++errStreak == 500) {   // ~0,5 s sem responder: loga uma vez
      Serial.println("[ina226] sem resposta no I2C — corrente indisponivel");
      // tenta reconfigurar (sensor pode ter resetado por brownout)
      writeReg(INA_REG_CONFIG, (0b001 << 9) | (0b000 << 6) | (0b001 << 3) | 0b101);
    }
#endif  // CUR_SENSE_ENABLED
    vTaskDelay(1);                     // ~1 kHz (INA) / 40 Hz (ADCs)
  }
}

#endif  // CUR_SENSE_ENABLED || VBAT_SENSE_ENABLED || NTC_SENSE_ENABLED
