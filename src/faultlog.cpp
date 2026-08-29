// ============================================================================
// faultlog.cpp — log persistente de faltas (anel de 32 entradas em NVS)
// ----------------------------------------------------------------------------
// Cada disparo de falta grava: odômetro, uptime, bits, e os valores no
// momento (corrente, tensão, temperatura, velocidade). Buffer em RAM;
// commit em NVS quando a roda parar (escrita de flash congela os núcleos).
// Download pelo app via BULK.
// ============================================================================
#include <Arduino.h>
#include <Preferences.h>
#include "faultlog.h"

#define FLOG_N 32

static FaultEntry   s_log[FLOG_N];
static uint8_t      s_count = 0, s_head = 0;
static Preferences  s_prefs;
static volatile bool s_dirty = false;

void faultLogInit() {
  s_prefs.begin("flog", false);
  s_count = (uint8_t)s_prefs.getUChar("n", 0);
  s_head  = (uint8_t)s_prefs.getUChar("h", 0);
  if (s_count > FLOG_N) s_count = 0;
  if (s_count)
    s_prefs.getBytes("e", s_log, sizeof(FaultEntry) * FLOG_N);
  Serial.printf("[flog] %u faltas no historico\n", s_count);
}

void faultLogAdd(const FaultEntry &e) {
  s_log[s_head] = e;
  s_head = (uint8_t)((s_head + 1) % FLOG_N);
  if (s_count < FLOG_N) s_count++;
  s_dirty = true;
}

void faultLogTick(float kmhAbs) {
  if (!s_dirty || kmhAbs > 2.0f) return;
  s_prefs.putBytes("e", s_log, sizeof(FaultEntry) * FLOG_N);
  s_prefs.putUChar("n", s_count);
  s_prefs.putUChar("h", s_head);
  s_dirty = false;
  Serial.println("[flog] log de faltas gravado");
}

// i = 0 é a falta MAIS RECENTE
bool faultLogGet(uint8_t i, FaultEntry &out) {
  if (i >= s_count) return false;
  int idx = (int)s_head - 1 - i;
  while (idx < 0) idx += FLOG_N;
  out = s_log[idx];
  return true;
}

void faultLogClear() {
  s_count = 0; s_head = 0; s_dirty = true;
}
