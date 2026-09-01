// ============================================================================
// params.cpp — tabela de parâmetros com defaults do config.h + NVS
// ============================================================================
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include "config.h"
#include "params.h"

struct ParamDef {
  uint8_t     id;
  const char *key;    // chave NVS (curta)
  float       def;    // default de fábrica (config.h)
  float       mn, mx; // clamp
};

// Ordem livre; ids estáveis (o app se guia pelo id, nunca pela posição).
static const ParamDef T[PARAM_N] = {
  { P_KE,         "ke",  MOTOR_KE_V_PER_RADS,    0.05f,  1.00f },  // diagnóstico
  { P_IQ_MAX,     "iqx", IQ_MAX_DEFAULT_A,       1.0f,  IQ_MAX_ABS_A },
  { P_MODCEIL,    "mc",  MODULATION_CEIL_FACTOR, 0.30f,  0.56f },  // bootstrap!
  { P_SLEW_UP,    "sua", TORQUE_SLEW_A_S,        5.0f, 600.0f  },   // A/s (chave nova)
  { P_SLEW_REL,   "sra", TORQUE_RELEASE_A_S,    20.0f, 2000.0f },   // A/s (chave nova)
  { P_ACCEL,      "ac",  ACCEL_RADS2,            5.0f, 100.0f  },
  { P_DECEL,      "de",  DECEL_RADS2,            1.0f,  50.0f  },
  { P_BRAKE_DEC,  "bd",  BRAKE_DECEL_RADS2,      5.0f, 120.0f  },
  { P_SPD_BAND,   "slp", SPD_LIM_P,              0.2f,  20.0f  },
  { P_PID_P,      "ppa", VEL_PID_P,              0.01f, 20.0f  },   // saída em A
  { P_PID_I,      "pia", VEL_PID_I,              0.0f, 200.0f  },   // saída em A
  { P_LPF_TF,     "tf",  VEL_LPF_TF,             0.01f,  0.5f  },
  { P_OC_TRIP,    "oc",  OC_TRIP_A,              5.0f,  40.0f  },
  { P_REGEN_I,    "rg",  REGEN_I_LIMIT_A,        1.0f,  25.0f  },
  { P_UV_CUT,     "uv",  UV_CUTOFF_V,           20.0f,  40.0f  },
  { P_RESERVADO_16, "td", TEMP_DERATE_C,        40.0f, 110.0f  },  // vago (ver params.h)
  { P_T_CUT,      "tc",  TEMP_CUTOFF_C,         50.0f, 120.0f  },
  { P_WHEEL_MM,   "wh",  WHEEL_DIAMETER_M * 1000.0f, 150.0f, 350.0f },
  { P_FW_START,   "fs",  92.0f,                 80.0f,  98.0f  },  // % de utilizacao do teto
  { P_FW_MAX_A,   "fwa", 0.0f,                   0.0f,  15.0f  },  // [A] id de FW; 0 = off
  { P_SHUNT_MOHM, "sh",  INA226_SHUNT_MOHM,      0.1f,  20.0f  },
  { P_ZEA,        "za",  MOTOR_ZERO_ELEC_ANGLE,  0.0f,  6.2832f },
  { P_BATT_WH,    "bw",  280.0f,                50.0f, 2000.0f },
  { P_DIR_INVERT, "di",  0.0f,                   0.0f,    1.0f },
  { P_MTPA,       "mt",  FEAT_MTPA_DEFAULT,      0.0f,    1.0f },
  { P_ECO_AUTO,   "ea",  FEAT_ECO_AUTO_DEFAULT,  0.0f,    1.0f },
  { P_PWM_FREQ,   "pf",  PWM_FREQUENCY_HZ,    8000.0f, 30000.0f },
  { P_MAX_KMH,    "vx",  MAX_KMH_DEFAULT,        1.0f, MAX_KMH_ABS },
  { P_ILIM_A,     "ix",  I_LIMIT_DEFAULT_A,      1.0f, I_LIMIT_MAX_A },
  { P_VLIM,       "lv",  VOLTAGE_LIMIT_DEFAULT,  0.5f, VOLTAGE_LIMIT_MAX },
  { P_UV_WARN,    "uw",  UV_WARN_V,             25.0f,  40.0f },
  { P_UV_RECOVER, "ur",  UV_RECOVER_V,          22.0f,  40.0f },
  { P_ECO_VMAX,   "ev",  RIDE_ECO_VMAX_KMH,      5.0f, MAX_KMH_ABS },
  { P_ECO_ILIM,   "ei",  RIDE_ECO_ILIM_A,        1.0f, I_LIMIT_MAX_A },
  { P_NOR_VMAX,   "nx",  RIDE_NORMAL_VMAX_KMH,   5.0f, MAX_KMH_ABS },
  { P_NOR_ILIM,   "nl",  RIDE_NORMAL_ILIM_A,     1.0f, I_LIMIT_MAX_A },
  { P_BATT_VMAX,  "bx",  BATT_VMAX_V,           36.0f,  42.5f },
  { P_IPH_TRIP,   "ipt", IPH_TRIP_A,            10.0f,  80.0f },
  { P_COAST,      "co",  FEAT_COAST_DEFAULT,     0.0f,    1.0f },
  { P_BATT_LEARN, "bl",  FEAT_BATT_LEARN_DEFAULT, 0.0f,   1.0f },
  { P_TRAC_DERATE,"tv",  FEAT_TRAC_DERATE_DEFAULT, 0.0f,  1.0f },
  { P_THERM_DERATE,"th", FEAT_THERM_DERATE_DEFAULT, 0.0f, 1.0f },
  { P_MT_DERATE,  "mtd", MOTOR_TEMP_DERATE_C,   40.0f, 160.0f },
  { P_MT_CUT,     "mtc", MOTOR_TEMP_CUTOFF_C,   50.0f, 180.0f },
  { P_CUR_P,      "cp",  CUR_PID_P,             0.005f,  5.0f },
  { P_CUR_I,      "ci",  CUR_PID_I,              0.0f, 3000.0f },
  { P_CUR_TF,     "ctf", CUR_LPF_TF,           0.0002f,  0.05f },
  { P_PH_SHUNT,   "psh", PHASE_CUR_SHUNT_MOHM,  0.05f,  20.0f },
  { P_PH_GAIN,    "pgn", 1.0f,                   0.5f,   2.0f },
  { P_SENS_DIR,   "sd",  (MOTOR_SENSOR_DIR == -1) ? -1.0f : 1.0f,
                                                 -1.0f,   1.0f },
  { P_COMM_LEAD,  "cle", COMM_LEAD_US_DEFAULT,    0.0f, 500.0f },
  { P_UD_MAX,     "udx", UD_MAX_FRAC,             0.0f,   0.5f },
  { P_CUR_DB,     "cdb", CUR_DEADBAND_A,          0.0f,   3.0f },
  { P_MEAS_AGE,   "age", IPH_MEAS_AGE_US,         0.0f, 1000.0f },
};

// Valores vivos, indexados por id (float alinhado => leitura atômica no Xtensa)
static float        s_val[P__COUNT];
static Preferences  s_prefs;
static volatile bool s_dirty    = false;
static uint32_t     s_dirtyMs   = 0;

static const ParamDef *find(uint8_t id) {
  for (int i = 0; i < PARAM_N; i++)
    if (T[i].id == id) return &T[i];
  return nullptr;
}

void paramsInit() {
  s_prefs.begin("params", false);
  for (int i = 0; i < PARAM_N; i++) {
    float v = s_prefs.getFloat(T[i].key, T[i].def);
    if (isnan(v) || v < T[i].mn || v > T[i].mx) v = T[i].def;
    s_val[T[i].id] = v;
  }
  Serial.printf("[params] %d parametros carregados (NVS + defaults do config.h)\n",
                PARAM_N);
}

float prm(uint8_t id) {
  return (id > 0 && id < P__COUNT) ? s_val[id] : 0.0f;
}

bool paramSet(uint8_t id, float v) {
  const ParamDef *d = find(id);
  if (!d || isnan(v)) return false;
  s_val[id] = constrain(v, d->mn, d->mx);
  return true;
}

void paramsRequestSave() {
  s_dirty   = true;
  s_dirtyMs = millis();
}

// Commit adiado: NVS = escrita de flash = congela os dois núcleos por ms.
// Só grava com a roda parada e 1 s depois da última mudança.
void paramsTick(float kmhAbs) {
  if (!s_dirty || millis() - s_dirtyMs < 1000) return;
  if (kmhAbs > 2.0f) return;
  for (int i = 0; i < PARAM_N; i++)
    s_prefs.putFloat(T[i].key, s_val[T[i].id]);   // NVS pula valores iguais
  s_dirty = false;
  Serial.println("[params] gravados na flash");
}

void paramsFactory() {
  for (int i = 0; i < PARAM_N; i++) s_val[T[i].id] = T[i].def;
  paramsRequestSave();
  Serial.println("[params] restaurados os defaults de fabrica");
}

int paramsPack(uint8_t *buf, int max) {
  int off = 0;
  if (max < 1 + PARAM_N * 9) return 0;
  buf[off++] = PARAM_N;
  for (int i = 0; i < PARAM_N; i++) {
    buf[off++] = T[i].id;
    float v = s_val[T[i].id], d = T[i].def;
    memcpy(buf + off, &v, 4); off += 4;
    memcpy(buf + off, &d, 4); off += 4;
  }
  return off;
}

// ---------------------------------------------------------------------------
// Visibilidade TOTAL do estado vivo. Nasceu de uma caçada de fantasma: o app
// reimpunha um P_VLIM da era antiga a cada conexão e nenhum paste mostrava —
// a NVS era invisível e três regressões "impossíveis" apontaram para o nada.
// Agora o boot despeja tudo (e o comando 'params' também): um paste do boot
// se autodocumenta. '*' marca valor diferente do default de fábrica.
// ---------------------------------------------------------------------------
void paramsDump() {
  Serial.println("[param] === parametros vivos (* = diferente do default do config) ===");
  char line[128]; int n = 0;
  for (int i = 0; i < PARAM_N; i++) {
    const float v    = prm(T[i].id);
    const bool  diff = fabsf(v - T[i].def) >
                       1e-4f * fmaxf(1.0f, fabsf(T[i].def));
    n += snprintf(line + n, sizeof(line) - n, "%s%s=%.4g  ",
                  diff ? "*" : "", T[i].key, (double)v);
    if (n > 88) { Serial.printf("[param] %s\n", line); n = 0; line[0] = 0; }
  }
  if (n) Serial.printf("[param] %s\n", line);
}

const char *paramKey(uint8_t id) {
  for (int i = 0; i < PARAM_N; i++)
    if (T[i].id == id) return T[i].key;
  return "?";
}

int paramIdByKey(const char *k) {
  for (int i = 0; i < PARAM_N; i++)
    if (!strcmp(T[i].key, k)) return T[i].id;
  return -1;
}
