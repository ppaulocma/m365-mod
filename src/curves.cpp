// ============================================================================
// curves.cpp — LUTs com validação física e double-buffer p/ o núcleo 1
// ============================================================================
#include <Arduino.h>
#include <Preferences.h>
#include "config.h"
#include "curves.h"

// Limites FÍSICOS de cada eixo (hard-coded: o app nunca passa disso)
struct CurveSpec { int16_t xMin, xMax, yMin, yMax; };
static const CurveSpec SPEC[CRV__COUNT] = {
  {},                            // 0 (não usado)
  {   0, 100, 0, 100 },          // THROTTLE  (%->%)
  {   0, 100, 0, 100 },          // BRAKE     (%->%)
  { 250, 440, 0, 100 },          // REGEN_V   (V*10->%)
  { 250, 440, 0, 100 },          // TRAC_V    (V*10->%)
  {   0, 120, 0, 100 },          // THERMAL   (°C->%)
  {   0,  45, 0, 100 },          // REGEN_S   (km/h->%)
};

struct Bank { uint8_t n; float x[CURVE_MAX_PTS], y[CURVE_MAX_PTS]; };
static Bank            s_bank[CRV__COUNT][2];
static volatile uint8_t s_active[CRV__COUNT];   // banco lido pelo núcleo 1

static Preferences  s_prefs;
static volatile bool s_dirty = false;
static uint32_t     s_dirtyMs = 0;

// Defaults = comportamento que o firmware já tinha (linear/limiares atuais)
struct Def { uint8_t id, n; int16_t x[CURVE_MAX_PTS], y[CURVE_MAX_PTS]; };
static const Def DEFS[CURVE_N] = {
  { CRV_THROTTLE, 2, {0, 100},            {0, 100} },
  { CRV_BRAKE,    2, {0, 100},            {0, 100} },
  // Sobretensão de regen (10s): 100% até 40,5 V, afunila, ZERO a 42,0 V
  { CRV_REGEN_V,  4, {250, 405, 418, 425},{100, 100, 25, 0} },
  // Tração x tensão: derate progressivo abaixo de ~34 V
  { CRV_TRAC_V,   4, {310, 320, 335, 440},{20, 45, 100, 100} },
  // Térmica (equivale ao derate linear 70->90 °C antigo)
  { CRV_THERMAL,  3, {70, 90, 120},       {100, 0, 0} },
  // Regen fraco em baixa velocidade (não trava a roda)
  { CRV_REGEN_S,  4, {0, 5, 12, 45},      {15, 55, 100, 100} },
};

static void loadInto(Bank &b, const int16_t *x, const int16_t *y, uint8_t n) {
  b.n = n;
  for (uint8_t i = 0; i < n; i++) { b.x[i] = x[i]; b.y[i] = y[i]; }
}

static bool validate(uint8_t id, const int16_t *x, const int16_t *y, uint8_t n) {
  if (id < 1 || id >= CRV__COUNT) return false;
  if (n < 2 || n > CURVE_MAX_PTS) return false;
  const CurveSpec &sp = SPEC[id];
  for (uint8_t i = 0; i < n; i++) {
    if (x[i] < sp.xMin || x[i] > sp.xMax) return false;
    if (y[i] < sp.yMin || y[i] > sp.yMax) return false;
    if (i > 0 && x[i] <= x[i - 1]) return false;   // x estritamente crescente
  }
  return true;
}

void curvesInit() {
  s_prefs.begin("curves", false);
  for (int i = 0; i < CURVE_N; i++) {
    const Def &d = DEFS[i];
    int16_t buf[2 * CURVE_MAX_PTS + 1];
    char key[6]; snprintf(key, sizeof(key), "c%d", d.id);
    size_t len = s_prefs.getBytes(key, buf, sizeof(buf));
    uint8_t n = (uint8_t)buf[0];
    bool ok = false;
    if (len >= 2u + 4u * 2 && n >= 2 && n <= CURVE_MAX_PTS &&
        len == 2u * (1 + 2 * n)) {
      ok = validate(d.id, buf + 1, buf + 1 + n, n);
      if (ok) loadInto(s_bank[d.id][0], buf + 1, buf + 1 + n, n);
    }
    if (!ok) loadInto(s_bank[d.id][0], d.x, d.y, d.n);
    s_bank[d.id][1] = s_bank[d.id][0];
    s_active[d.id] = 0;
  }
  Serial.printf("[curves] %d curvas carregadas\n", CURVE_N);
}

float curveEval(uint8_t id, float xin) {
  if (id < 1 || id >= CRV__COUNT) return 0.0f;
  const Bank &b = s_bank[id][s_active[id]];
  if (b.n < 2) return 0.0f;
  if (xin <= b.x[0])       return b.y[0];
  if (xin >= b.x[b.n - 1]) return b.y[b.n - 1];
  for (uint8_t i = 1; i < b.n; i++) {
    if (xin <= b.x[i]) {
      float f = (xin - b.x[i - 1]) / (b.x[i] - b.x[i - 1]);
      return b.y[i - 1] + f * (b.y[i] - b.y[i - 1]);
    }
  }
  return b.y[b.n - 1];
}

bool curveSet(uint8_t id, const int16_t *x, const int16_t *y, uint8_t n) {
  if (!validate(id, x, y, n)) return false;      // REJEITA — nada aplicado
  uint8_t spare = 1 - s_active[id];
  loadInto(s_bank[id][spare], x, y, n);
  s_active[id] = spare;                          // troca atômica de banco
  s_dirty = true; s_dirtyMs = millis();
  return true;
}

int curveGet(uint8_t id, uint8_t *buf, int max) {
  if (id < 1 || id >= CRV__COUNT) return 0;
  const Bank &b = s_bank[id][s_active[id]];
  int need = 2 + 4 * b.n;
  if (max < need) return 0;
  buf[0] = id; buf[1] = b.n;
  int off = 2;
  for (uint8_t i = 0; i < b.n; i++) {
    int16_t xv = (int16_t)b.x[i], yv = (int16_t)b.y[i];
    memcpy(buf + off, &xv, 2); off += 2;
    memcpy(buf + off, &yv, 2); off += 2;
  }
  return off;
}

void curvesFactory() {
  for (int i = 0; i < CURVE_N; i++) {
    const Def &d = DEFS[i];
    uint8_t spare = 1 - s_active[d.id];
    loadInto(s_bank[d.id][spare], d.x, d.y, d.n);
    s_active[d.id] = spare;
  }
  s_dirty = true; s_dirtyMs = millis();
}

void curvesTick(float kmhAbs) {
  if (!s_dirty || millis() - s_dirtyMs < 1000) return;
  if (kmhAbs > 2.0f) return;                     // flash só com roda parada
  for (int i = 0; i < CURVE_N; i++) {
    uint8_t id = DEFS[i].id;
    const Bank &b = s_bank[id][s_active[id]];
    int16_t buf[2 * CURVE_MAX_PTS + 1];
    buf[0] = b.n;
    for (uint8_t j = 0; j < b.n; j++) {
      buf[1 + j]       = (int16_t)b.x[j];
      buf[1 + b.n + j] = (int16_t)b.y[j];
    }
    char key[6]; snprintf(key, sizeof(key), "c%d", id);
    s_prefs.putBytes(key, buf, 2u * (1 + 2 * b.n));
  }
  s_dirty = false;
  Serial.println("[curves] gravadas na flash");
}
