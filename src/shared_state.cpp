// ============================================================================
// shared_state.cpp — instâncias globais + acesso protegido por spinlock.
//
// portMUX_TYPE (taskENTER_CRITICAL) é a primitiva certa aqui: as seções
// críticas são só uma cópia de struct (< 1 µs), então bloquear interrupções
// brevemente é mais barato e mais determinístico que um mutex — e é seguro
// entre núcleos (spinlock de hardware).
// ============================================================================
#include "shared_state.h"
#include "config.h"

static portMUX_TYPE s_stateMux = portMUX_INITIALIZER_UNLOCKED;

// Estado inicial SEGURO: motor desabilitado, alvos zerados, limites de bancada.
static Command s_cmd = {
  .throttle    = 0.0f,
  .brake       = 0.0f,
  .enable      = false,
  .estop       = false,
  .mode        = MODE_TORQUE,
  .i_limit_a   = I_LIMIT_DEFAULT_A,
  .max_kmh     = 0.0f,             // 0 = segue o parâmetro P_MAX_KMH
  .v_limit     = VOLTAGE_LIMIT_DEFAULT,
  .zea         = NAN,
  .ride        = 1,        // liga em NORMAL
  .cruise      = false,
  .walk        = false,
  .locked      = false,
  .headlight   = false,          // RAM: reset = farol apagado
  .clear_req   = 0,
  .last_cmd_ms = 0,
};

static Telemetry s_tlm = {};

Command readCommand() {
  taskENTER_CRITICAL(&s_stateMux);
  Command c = s_cmd;
  taskEXIT_CRITICAL(&s_stateMux);
  return c;
}

void writeCommand(const Command &c) {
  taskENTER_CRITICAL(&s_stateMux);
  s_cmd = c;
  taskEXIT_CRITICAL(&s_stateMux);
}

// Escrita ATÔMICA só de acelerador/freio (usada pelo painel UART). Evita o
// read-modify-write da struct inteira: o painel escrevendo a 10 Hz nunca
// atropela um campo que o BLE mudou no meio (modo, limites, enable...).
// Writebacks do núcleo do motor: comandos de EVENTO voltam a 0 depois de
// consumidos/cancelados — senão o próximo clique do app não gera borda
// ("cliquei e não fez nada" do cruzeiro).
void clearCruiseCmd() {
  taskENTER_CRITICAL(&s_stateMux);
  s_cmd.cruise = false;
  taskEXIT_CRITICAL(&s_stateMux);
}
void clearVmaxCmd() {
  taskENTER_CRITICAL(&s_stateMux);
  s_cmd.max_kmh = 0.0f;
  taskEXIT_CRITICAL(&s_stateMux);
}
void cycleRideCmd() {
  taskENTER_CRITICAL(&s_stateMux);
  s_cmd.ride = (uint8_t)((s_cmd.ride + 1) % 3);
  taskEXIT_CRITICAL(&s_stateMux);
}

void writeControls(float throttle, float brake, bool stampDeadman) {
  taskENTER_CRITICAL(&s_stateMux);
  s_cmd.throttle = throttle;
  s_cmd.brake    = brake;
  if (stampDeadman) s_cmd.last_cmd_ms = millis();
  taskEXIT_CRITICAL(&s_stateMux);
}

Telemetry readTelemetry() {
  taskENTER_CRITICAL(&s_stateMux);
  Telemetry t = s_tlm;
  taskEXIT_CRITICAL(&s_stateMux);
  return t;
}

void writeTelemetry(const Telemetry &t) {
  taskENTER_CRITICAL(&s_stateMux);
  s_tlm = t;
  taskEXIT_CRITICAL(&s_stateMux);
}
