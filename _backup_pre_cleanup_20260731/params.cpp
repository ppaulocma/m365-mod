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
  { P_KE,         "ke",  MOTOR_KE_V_PER_RADS,    0.05f,  1.00f },
  { P_HEADROOM,   "hd",  STALL_V_HEADROOM,       1.0f,  20.0f  },
  { P_MODCEIL,    "mc",  MODULATION_CEIL_FACTOR, 0.30f,  0.56f },  // bootstrap!
  { P_SLEW_UP,    "su",  TORQUE_SLEW_V_S,        2.0f, 100.0f  },
  { P_SLEW_REL,   "sr",  TORQUE_RELEASE_V_S,    20.0f, 400.0f  },
  { P_ACCEL,      "ac",  ACCEL_RADS2,            5.0f, 100.0f  },
  { P_DECEL,      "de",  DECEL_RADS2,            1.0f,  50.0f  },
  { P_BRAKE_DEC,  "bd",  BRAKE_DECEL_RADS2,      5.0f, 120.0f  },
  { P_CAP_MARGIN, "cm",  SPEED_CAP_MARGIN_V,     0.0f,   3.0f  },
  { P_PID_P,      "pp",  VEL_PID_P,              0.01f,  2.0f  },
  { P_PID_I,      "pi",  VEL_PID_I,              0.0f,  20.0f  },
  { P_LPF_TF,     "tf",  VEL_LPF_TF,             0.01f,  0.5f  },
  { P_OC_TRIP,    "oc",  OC_TRIP_A,              5.0f,  60.0f  },
  { P_REGEN_I,    "rg",  REGEN_I_LIMIT_A,        1.0f,  25.0f  },
  { P_UV_CUT,     "uv",  UV_CUTOFF_V,           20.0f,  40.0f  },
  { P_T_DERATE,   "td",  TEMP_DERATE_C,         40.0f, 110.0f  },
  { P_T_CUT,      "tc",  TEMP_CUTOFF_C,         50.0f, 120.0f  },
  { P_WHEEL_MM,   "wh",  WHEEL_DIAMETER_M * 1000.0f, 150.0f, 350.0f },
  { P_FW_START,   "fs",  92.0f,                 80.0f,  98.0f  },  // joelho de duty %
  { P_FW_MAX_DEG, "fm",  0.0f,                 -45.0f,  45.0f  },  // 0 = FW off
  { P_SHUNT_MOHM, "sh",  INA226_SHUNT_MOHM,      0.5f,  20.0f  },
  { P_ZEA,        "za",  MOTOR_ZERO_ELEC_ANGLE,  0.0f,  6.2832f },
  { P_BATT_WH,    "bw",  275.0f,                50.0f, 2000.0f },
  { P_DIR_INVERT, "di",  0.0f,                   0.0f,    1.0f },
  { P_MTPA,       "mt",  1.0f,                   0.0f,    1.0f },
  { P_ECO_AUTO,   "ea",  1.0f,                   0.0f,    1.0f },
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
  { P_HALL_INTERP,"hi",  1.0f,                   0.0f,    1.0f },
  { P_ILIM_FF,    "ff",  1.0f,                   0.0f,    1.0f },
  { P_COAST,      "co",  1.0f,                   0.0f,    1.0f },
  { P_BATT_LEARN, "bl",  1.0f,                   0.0f,    1.0f },
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
