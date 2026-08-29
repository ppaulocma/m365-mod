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
//     (mesma referência dos FD2103S, do shunt e dos conversores de nível).
//  1. Alimente o estágio de potência por FONTE DE BANCADA com limite de
//     corrente (~2 A no início), NUNCA direto na bateria no primeiro teste.
//     Roda SUSPENSA (motor livre).
//  2. Em config.h, coloque OPEN_LOOP_TEST 1 e grave. Conecte no WiFi
//     "Patinete-ESP" (senha "patinete123"), abra http://192.168.4.1,
//     clique LIGAR MOTOR e suba o acelerador devagar.
//       - Girou liso => potência e ordem de fases OK.
//       - Vibrou/rangeu sem girar => troque dois PIN_PWM_* no config.h.
//  3. Volte OPEN_LOOP_TEST para 0 e grave. No primeiro boot o initFOC()
//     gira o motor para calibrar os Halls e imprime no serial o
//     MOTOR_ZERO_ELEC_ANGLE e o MOTOR_SENSOR_DIR — cole-os no config.h
//     para os próximos boots não precisarem calibrar.
//  4. Sentido de giro invertido? O initFOC já compensa por software; se
//     quiser "para frente = positivo", troque dois PIN_PWM_* E os dois
//     PIN_HALL_* correspondentes (ou inverta MOTOR_SENSOR_DIR).
//  5. Calibre os ADCs (config.h): divisor do VBAT, mV/A do shunt, beta do
//     NTC e VOLTAGE_POWER_SUPPLY. Só então suba os limites pela dashboard.
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

  // Buffer TX grande: prints/respostas do Commander são absorvidos pelo buffer
  // em vez de BLOQUEAREM a task do motor esperando a UART drenar (115200 é
  // lento — um bloqueio de ms no loop FOC vira solavanco de torque).
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=====================================================");
  Serial.println("  Patinete ESP — BLDC M365 + SimpleFOC (FD2103S 3PWM)");
  Serial.println("=====================================================");

  // Parâmetros, curvas, log de faltas e odômetro (NVS) — antes das tasks
  paramsInit();
  curvesInit();
  faultLogInit();
  motorEnergyInit();

  // Núcleo 0: BLE + dashboard (criada primeiro — a task do motor tem
  // prioridade maior que esta loopTask e tomaria o núcleo 1 antes do
  // setup() terminar). No modo bancada puro (BENCH_TEST + BENCH_BLE 0)
  // ela NEM EXISTE — núcleo 0 fica só com o idle.
#if !(BENCH_TEST && !BENCH_BLE)
  xTaskCreatePinnedToCore(bleDashboardTask, "ble", WEB_TASK_STACK,
                          nullptr, WEB_TASK_PRIO, nullptr, 0);
#else
  Serial.println("[bench] MODO BANCADA PURO: BLE/dashboard DESLIGADOS (so serial)");
#endif

#if CUR_SENSE_INA226 || VBAT_SENSE_ENABLED || NTC_SENSE_ENABLED
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

// Tudo roda nas duas tasks acima; a loopTask fica ociosa.
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
