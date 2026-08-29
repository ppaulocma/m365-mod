// ============================================================================
// uart_panel.cpp — mestre do link com o painel (Serial2, núcleo 0)
// ----------------------------------------------------------------------------
// Ciclo de 100 ms: monta o frame de telemetria a partir da Telemetry
// compartilhada, transmite, e processa o que chegar (resposta do Nano com
// acelerador/freio). O eco do próprio frame é ignorado pelo cabeçalho.
// Segurança: sem resposta fresca, panelControlsActive() vira false e o
// dead-man existente (CMD_TIMEOUT_MS) zera o alvo do motor.
// ============================================================================
#include <Arduino.h>
#include "config.h"
#include "shared_state.h"
#include "uart_panel.h"
#include "ble_dashboard.h"

#if PANEL_UART_ENABLED

static volatile uint32_t s_lastCtrlMs = 0;

// Estatísticas p/ diagnóstico do link (impressas a cada 5 s no serial):
//   eco: bytes ouvidos no RX — no fio único o ESP OUVE O PRÓPRIO ECO, então
//        com a fiação do lado ESP ok isso é ~160 B/s MESMO SEM o Nano ligado.
//        eco=0 => problema LOCAL (diodo do TX17 invertido, RX16 fora da
//        linha, pullup) — nem precisa olhar o cabo/painel.
//   ctrl: frames válidos de controle vindos do NANO (link completo).
static uint32_t s_txFrames = 0, s_rxBytes = 0, s_ctrlFrames = 0;

bool panelControlsActive() {
  uint32_t t = s_lastCtrlMs;
  return (t != 0) && (millis() - t < PANEL_TIMEOUT_MS);
}

// CRC8 poly 0x07 (mesma implementação no sketch do Nano)
static uint8_t crc8(const uint8_t *d, uint8_t n) {
  uint8_t c = 0;
  while (n--) {
    c ^= *d++;
    for (uint8_t i = 0; i < 8; i++)
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

// SOC aproximado por tensão de um pack 10s Li-Ion (curva em repouso —
// sob carga lê um pouco baixo; bom o suficiente para o display do guidão)
static uint8_t socFromVolts(float v) {
  static const float   vs[] = {30.0f, 32.0f, 34.0f, 35.0f, 36.0f, 36.5f, 37.0f, 38.0f, 39.0f, 40.0f, 41.0f, 42.0f};
  static const uint8_t ss[] = {0,     2,     8,     15,    25,    35,    45,    60,    75,    87,    95,    100};
  const int n = sizeof(ss);
  if (v <= vs[0]) return 0;
  if (v >= vs[n - 1]) return 100;
  for (int i = 1; i < n; i++)
    if (v < vs[i]) {
      float f = (v - vs[i - 1]) / (vs[i] - vs[i - 1]);
      return (uint8_t)(ss[i - 1] + f * (ss[i] - ss[i - 1]) + 0.5f);
    }
  return 100;
}

// ---- Parser da resposta do Nano (BB 66 03 th br flags crc) -----------------
static void applyControls(uint8_t th, uint8_t br, uint8_t flags) {
  // failsafe do sensor: só aplica o que o Nano declarou como válido.
  // writeControls é atômico e NÃO toca nos outros campos do Command —
  // zero chance de atropelar um comando concorrente do BLE (modo, limites).
  writeControls((flags & 0x01) ? th / 255.0f : 0.0f,
                (flags & 0x02) ? br / 255.0f : 0.0f,
                true);            // alimenta o dead-man existente

  // Clique rápido do botão do painel = próximo modo de pilotagem.
  // O Nano manda um CONTADOR (bits 4-5, mod 4): cada mudança = um clique —
  // robusto a frame perdido e sem estado duplicado dos dois lados.
  static uint8_t prevClk = 0xFF;
  uint8_t clk = (flags >> 4) & 0x03;
  if (prevClk != 0xFF && clk != prevClk) cycleRideCmd();
  prevClk = clk;

  s_lastCtrlMs = millis();
  s_ctrlFrames++;
}

static void feedParser(uint8_t b) {
  static uint8_t st = 0, len = 0, idx = 0, buf[8];
  switch (st) {
    case 0: st = (b == 0xBB) ? 1 : 0; break;
    case 1: st = (b == 0x66) ? 2 : (b == 0xBB ? 1 : 0); break;
    case 2:
      len = b; idx = 0;
      st = (len == 3) ? 3 : 0;          // só aceitamos o frame de controles
      break;
    case 3:
      buf[idx++] = b;
      if (idx >= len) st = 4;
      break;
    case 4: {                            // CRC sobre LEN+payload
      uint8_t chk[4] = {len, buf[0], buf[1], buf[2]};
      if (crc8(chk, 4) == b) applyControls(buf[0], buf[1], buf[2]);
      st = 0;
      break;
    }
  }
}

// ---- Task principal --------------------------------------------------------
void panelTask(void *arg) {
  Serial2.setTxBufferSize(256);          // TX nunca bloqueia a task
  Serial2.begin(PANEL_BAUD, SERIAL_8N1, PIN_PANEL_RX, PIN_PANEL_TX);
  Serial.printf("[painel] link em %d baud (RX=%d TX=%d)\n",
                PANEL_BAUD, PIN_PANEL_RX, PIN_PANEL_TX);

  for (;;) {
    // 1) telemetria -> Nano
    Telemetry t = readTelemetry();
    float   powerF = t.vbat_v * t.ibat_a;
    int16_t kmh10  = (int16_t)constrain(t.kmh * 10.0f, -32000.0f, 32000.0f);
    uint16_t vb10  = (uint16_t)constrain(t.vbat_v * 10.0f, 0.0f, 65000.0f);
    int16_t ib10   = (int16_t)constrain(t.ibat_a * 10.0f, -32000.0f, 32000.0f);
    int16_t pw     = (int16_t)constrain(powerF, -32000.0f, 32000.0f);
    int8_t  tc     = (int8_t)constrain(t.temp_c, -127.0f, 127.0f);
    uint8_t soc    = t.soc_pct ? t.soc_pct : socFromVolts(t.vbat_v);
    uint8_t flags  = (t.enabled ? 0x01 : 0)
                   | (t.mode == MODE_VELOCITY ? 0x02 : 0)
                   | (t.ibat_a < -0.3f ? 0x04 : 0)    // regen
                   | (bleConnected() ? 0x08 : 0)      // app BLE (ícone BT)
                   | (t.headlight ? 0x10 : 0);        // FAROL (D6 do Nano)
    uint8_t fl     = (uint8_t)(t.faults & 0xFF);

    // Autonomia p/ visualização rápida no painel (mesma conta do app):
    // capacidade aprendida x SoC / consumo recente. 0 = ainda sem estimativa.
    uint8_t rangeKm = 0;
    if (t.wh_per_km > 1.0f && t.cap_wh > 0)
      rangeKm = (uint8_t)constrain(
          (float)t.cap_wh * (soc / 100.0f) / t.wh_per_km, 0.0f, 254.0f);
    uint16_t wk10 = (uint16_t)constrain(t.wh_per_km * 10.0f, 0.0f, 65000.0f);

    // flags2: modo de pilotagem + estados sob demanda (painel mostra tudo)
    uint8_t flags2 = (uint8_t)(t.ride & 0x03)
                   | (t.cruise   ? 0x04 : 0)
                   | (t.walk     ? 0x08 : 0)
                   | (t.locked   ? 0x10 : 0)
                   | (t.eco_auto ? 0x20 : 0)
                   | (t.fw_x2 > 0 ? 0x40 : 0);

    uint8_t p[16];
    p[0] = kmh10 & 0xFF;  p[1] = kmh10 >> 8;
    p[2] = vb10 & 0xFF;   p[3] = vb10 >> 8;
    p[4] = ib10 & 0xFF;   p[5] = ib10 >> 8;
    p[6] = pw & 0xFF;     p[7] = pw >> 8;
    p[8] = (uint8_t)tc;   p[9] = soc;
    p[10] = flags;        p[11] = fl;
    p[12] = rangeKm;
    p[13] = wk10 & 0xFF;  p[14] = wk10 >> 8;
    p[15] = flags2;

    uint8_t frame[20];
    frame[0] = 0xAA; frame[1] = 0x55; frame[2] = 16;
    memcpy(frame + 3, p, 16);
    uint8_t chk[17]; chk[0] = 16; memcpy(chk + 1, p, 16);
    frame[19] = crc8(chk, 17);
    Serial2.write(frame, 20);
    s_txFrames++;

    // 2) processa RX (resposta do Nano; o eco do AA55 não casa com BB66)
    uint32_t t0 = millis();
    while (millis() - t0 < PANEL_PERIOD_MS) {
      while (Serial2.available()) {
        feedParser((uint8_t)Serial2.read());
        s_rxBytes++;
      }
      vTaskDelay(pdMS_TO_TICKS(2));
    }

    // 3) FAILSAFE DE RESPOSTA VELHA: os controles chegam a 10 Hz — ficar
    // 2,5 ciclos sem resposta válida significa frames se perdendo. Zera
    // acelerador/freio JÁ (uma vez), em vez de segurar o último valor até
    // o dead-man de CMD_TIMEOUT_MS ("delay no acelerador quando solto").
    // Não estampa o dead-man: se o link não voltar, ele corta como sempre.
    {
      static bool staleZeroed = false;
      bool stale = s_lastCtrlMs && (millis() - s_lastCtrlMs > PANEL_CTRL_STALE_MS);
      if (stale && !staleZeroed) {
        writeControls(0.0f, 0.0f, false);
        staleZeroed = true;
      } else if (!stale) {
        staleZeroed = false;
      }
    }

    // Diagnóstico do link: só avisa quando o link ESTÁ COM PROBLEMA (uma
    // vez por transição) — o spam periódico de estatísticas saiu.
    if (s_txFrames % 50 == 0) {
      static uint8_t lastState = 0xFF;
      uint8_t state = (s_rxBytes == 0) ? 1 : (s_ctrlFrames == 0 ? 2 : 0);
      if (state != lastState && state != 0) {
        Serial.printf("[painel] %s\n", state == 1
          ? "ECO ZERO: fiacao do LADO ESP (diodo 17/RX16/pullup)"
          : "eco ok, Nano mudo: cabo/BSS/lado do painel");
      }
      lastState = state;
    }
  }
}

#endif  // PANEL_UART_ENABLED
