// ============================================================================
// ble_dashboard.cpp — servidor BLE (NimBLE) no NÚCLEO 0
// ----------------------------------------------------------------------------
// Por que BLE em vez de WiFi/web:
//   1. NimBLE consome uma fração da RAM/flash do stack WiFi+AsyncWebServer;
//   2. As rajadas de TX do BLE (~130 mA) são muito menores que as do WiFi
//      (~400 mA) — menos estresse no 3,3 V que alimenta os Halls;
//   3. O stack todo (controlador BT + host NimBLE) roda pinado no núcleo 0 —
//      o núcleo 1 permanece exclusivo do motor, sem exceções.
// Segurança: mesmo dead-man de sempre — comandos de acelerador via app
// alimentam last_cmd_ms; sem comandos, o alvo zera em CMD_TIMEOUT_MS.
// ============================================================================
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <math.h>
#include "config.h"
#include "shared_state.h"
#include "uart_panel.h"
#include "ble_dashboard.h"
#include "params.h"
#include "motor_control.h"
#include "curves.h"
#include "faultlog.h"
#include "ota.h"

static NimBLECharacteristic *s_telemChar  = nullptr;
static NimBLECharacteristic *s_paramsChar = nullptr;
static volatile bool         s_connected  = false;

bool bleConnected() { return s_connected; }

// STREAM de alta taxa (0 = desligado; 25/50 Hz) e fonte do BULK
static NimBLECharacteristic *s_streamChar = nullptr;
static NimBLECharacteristic *s_dbgChar    = nullptr;
static NimBLECharacteristic *s_bulkChar   = nullptr;
static volatile uint8_t  s_streamHz   = 0;
static volatile uint8_t  s_dbgHz      = 0;   // datalogger de debug (0=off)
static volatile uint8_t  s_bulkSrc    = 0;   // 1 captura | 2 log de faltas
static volatile uint32_t s_bulkOff    = 0;
static volatile uint8_t  s_curveSel   = CRV_THROTTLE;   // p/ leitura da CURVE

// ---------------------------------------------------------------------------
// Comandos recebidos do app ([id u8][valor f32] repetidos)
// ---------------------------------------------------------------------------
static void handleCommand(uint8_t id, float v) {
  Command c = readCommand();
  switch (id) {
    case 1:  // acelerador (ignorado com painel físico ativo — ele manda)
      if (!panelControlsActive()) c.throttle = constrain(v, 0.0f, 1.0f);
      break;
    case 2:  // freio
      if (!panelControlsActive()) c.brake = constrain(v, 0.0f, 1.0f);
      break;
    case 3: c.enable = (v > 0.5f); break;
    case 4:  // parada de emergência
      c.estop = true; c.enable = false; c.throttle = 0; c.brake = 0;
      break;
    case 5:  // limpar faltas (também solta o e-stop)
      c.clear_req++; c.estop = false;
      break;
    case 6:  // modo (zera alvos na troca, como sempre)
      c.mode = (v > 0.5f) ? MODE_VELOCITY : MODE_TORQUE;
      c.throttle = 0; c.brake = 0;
      break;
    // 7/9: corrente e tensão são PARÂMETROS (atalho de compat, RAM s/ gravar)
    // SEMPRE logados: o app reimpõe o que tem salvo NA CONEXÃO — um P_VLIM
    // da era do teto antigo reimposto em silêncio já mascarou o teto novo
    // (topo "voltou" para 23 km/h e ninguém via de onde).
    case 7:  paramSet(P_ILIM_A,  v);
             Serial.printf("[ble] app: ilim = %.1f A\n", v);         break;
    // 8: slider do Piloto = teto de SESSÃO (RAM; 0 volta ao parâmetro).
    //    O firmware aplica min(parâmetro, sessão) — nunca passa do gravado.
    case 8:  c.max_kmh = (v < 0.5f) ? 0.0f
                                    : constrain(v, 1.0f, MAX_KMH_ABS);  break;
    case 9:  paramSet(P_VLIM,    v);
             Serial.printf("[ble] app: VLIM = %.1f V (teto de tensao vindo "
                           "do app!)\n", v);                         break;
    case 10: c.zea       = constrain(v, 0.0f, 6.2832f);              break;
    case 20: paramsRequestSave();                                    break;
    case 21: paramsFactory(); curvesFactory();                       break;
    case 22: motorRequestCalZea();                                   break;  // GIRA o motor!
    case 23: motorRequestCalKe();                                    break;  // roda livre!
    // 24 (auto-cal completa) e 25 (re-zerar INA240) removidos: a primeira
    // gravava o ângulo SEM medir a direção do sensor — a calibração de ângulo
    // (22) faz as duas coisas —, e o zero é rastreado ao vivo a 8 kHz.
    case 29: motorRequestAngScan(3.0f, (v > 1.0f) ? v : 6.0f);       break;  // afina o ângulo GIRANDO
    case 26: motorRequestIphCal();                                   break;  // CALIBRA corrente de fase
    case 27: motorRequestIphDiag();                                  break;  // despejo cru no serial
    case 28: motorRequestIStep(v);                                   break;  // degrau da malha de corrente
    case 30: s_streamHz = (uint8_t)constrain(v, 0.0f, 50.0f);        break;
    case 47: s_dbgHz    = (uint8_t)constrain(v, 0.0f, 50.0f);        break;  // debug log
    case 31:  // step response (amplitude em km/h) — roda suspensa!
      if (!motorStartStep(v)) motorReportCmdStatus(CMDERR_BAD_STATE);
      break;
    case 32: s_bulkSrc = (uint8_t)v; s_bulkOff = 0;                  break;
    case 33: motorCaptureFree();                                     break;
    case 34: s_curveSel = (uint8_t)v;                                break;
    case 36: c.ride    = (uint8_t)constrain(v, 0.0f, 2.0f);          break;
    case 37: c.cruise  = (v > 0.5f);                                 break;
    case 38: c.walk    = (v > 0.5f);                                 break;
    case 39: c.locked  = (v > 0.5f);                                 break;
    case 40: otaRequest();                                           break;
    case 41: motorRequestCrash();                                    break;  // caixa-preta no serial
    case 44: motorTripReset();                                       break;
    case 45: faultLogClear();                                        break;
    case 46: c.headlight = v > 0.5f;                                 break;
    default: return;
  }
  c.last_cmd_ms = millis();     // alimenta o dead-man
  writeCommand(c);
}

class CtrlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *ch) override {
    NimBLEAttValue val = ch->getValue();
    const uint8_t *d = val.data();
    size_t n = val.length();
    for (size_t i = 0; i + 5 <= n; i += 5) {
      float v;
      memcpy(&v, d + i + 1, 4);
      handleCommand(d[i], v);
    }
  }
};

class ParamsCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic *ch) override {
    uint8_t buf[1 + PARAM_N * 9];
    int n = paramsPack(buf, sizeof(buf));
    ch->setValue(buf, n);
  }
  void onWrite(NimBLECharacteristic *ch) override {
    NimBLEAttValue val = ch->getValue();
    const uint8_t *d = val.data();
    size_t n = val.length();
    for (size_t i = 0; i + 5 <= n; i += 5) {
      float v;
      memcpy(&v, d + i + 1, 4);
      const float old = prm(d[i]);
      paramSet(d[i], v);
      // Toda escrita do app fica VISÍVEL no serial (só quando muda algo):
      // parâmetro reimposto em silêncio pela conexão já mascarou teto novo.
      if (fabsf(prm(d[i]) - old) > 1e-6f)
        Serial.printf("[ble] app: param %s = %.4g (era %.4g)\n",
                      paramKey(d[i]), (double)prm(d[i]), (double)old);
    }
  }
};

class CurveCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic *ch) override {
    uint8_t buf[2 + 4 * CURVE_MAX_PTS];
    int n = curveGet(s_curveSel, buf, sizeof(buf));
    ch->setValue(buf, n);
  }
  void onWrite(NimBLECharacteristic *ch) override {
    NimBLEAttValue val = ch->getValue();
    const uint8_t *d = val.data();
    size_t len = val.length();
    // formato: [id][n][x i16, y i16]*n — VALIDADO no firmware
    if (len < 2) { motorReportCmdStatus(CMDERR_REJECTED); return; }
    uint8_t id = d[0], n = d[1];
    if (len != 2u + 4u * n || n > CURVE_MAX_PTS) {
      motorReportCmdStatus(CMDERR_REJECTED); return;
    }
    int16_t x[CURVE_MAX_PTS], y[CURVE_MAX_PTS];
    for (uint8_t i = 0; i < n; i++) {
      memcpy(&x[i], d + 2 + 4 * i, 2);
      memcpy(&y[i], d + 4 + 4 * i, 2);
    }
    if (!curveSet(id, x, y, n)) motorReportCmdStatus(CMDERR_REJECTED);
  }
};

class BulkCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic *ch) override {
    uint8_t buf[240];
    int n = 0;
    if (s_bulkSrc == 1) {                       // captura do step response
      n = motorCaptureRead((uint16_t)(s_bulkOff / 6), buf, 240);
      s_bulkOff += n;
    } else if (s_bulkSrc == 2) {                // log de faltas
      uint8_t i = (uint8_t)(s_bulkOff / sizeof(FaultEntry));
      FaultEntry e;
      while (n + (int)sizeof(FaultEntry) <= 240 && faultLogGet(i, e)) {
        memcpy(buf + n, &e, sizeof(FaultEntry));
        n += sizeof(FaultEntry); i++;
      }
      s_bulkOff += n;
    }
    ch->setValue(buf, n);                       // 0 bytes = fim dos dados
  }
};

static NimBLEServer *s_srv = nullptr;
static volatile bool s_radioPaused = false;

// Silencia o rádio: para o advertising e derruba conexões. Existe para um
// experimento — a corrente de fase mostra blocos de ruído de ~3 ms a cada
// ~12-15 ms, e a suspeita é que sejam eventos de rádio (advertising também
// transmite, então "sem app conectado" NÃO é rádio quieto). Com o rádio
// realmente parado dá para medir a diferença em vez de deduzir.
#if IPH_DIAG_TOOLS
void bleRadioPause(bool pause) {
  // A ORDEM importa e a flag também: derrubar a conexão dispara onDisconnect,
  // que rearma o advertising na hora. Sem a flag, "parar o rádio" só trocava
  // conexão por advertising — que transmite igual, e o teste media nada.
  s_radioPaused = pause;
  if (pause) {
    if (s_srv) {
      auto peers = s_srv->getPeerDevices();
      for (auto h : peers) s_srv->disconnect(h);
      delay(120);                       // deixa a desconexão concluir
    }
    NimBLEDevice::stopAdvertising();    // só DEPOIS, senão o callback rearma
  } else {
    NimBLEDevice::startAdvertising();
  }
}
#else
void bleRadioPause(bool) {}          // ferramentas de diagnostico desligadas
#endif

class SrvCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s) override {
    s_connected = true;
    Serial.println("[ble] app conectado");
  }
  void onDisconnect(NimBLEServer *s) override {
    s_connected = false;
    // Não rearmar durante um teste de rádio parado: era isto que fazia
    // "parar o rádio" virar só "trocar conexão por advertising".
    if (s_radioPaused) { Serial.println("[ble] app desconectado (radio pausado)"); return; }
    Serial.println("[ble] app desconectado — anunciando de novo");
    NimBLEDevice::startAdvertising();
  }
};

// ---------------------------------------------------------------------------
// Empacota a telemetria (26 bytes, formato no ble_dashboard.h)
// ---------------------------------------------------------------------------
static bool s_brakeLightOn = false;   // estado vivo da luz de freio

static size_t packTelemetry(uint8_t *b) {
  Telemetry t = readTelemetry();
  float   powerF = t.vbat_v * t.ibat_a;
  int16_t kmh10  = (int16_t)constrain(t.kmh * 10.0f, -32000.0f, 32000.0f);
  uint16_t vb10  = (uint16_t)constrain(t.vbat_v * 10.0f, 0.0f, 65000.0f);
  int16_t ib10   = (int16_t)constrain(t.ibat_a * 10.0f, -32000.0f, 32000.0f);
  int16_t pw     = (int16_t)constrain(powerF, -32000.0f, 32000.0f);
  int16_t tg10   = (int16_t)constrain(t.target * 10.0f, -32000.0f, 32000.0f);
  uint8_t flags  = (t.enabled ? 0x01 : 0)
                 | (t.mode == MODE_VELOCITY ? 0x02 : 0)
                 | (t.ibat_a < -0.3f ? 0x04 : 0)
                 | (panelControlsActive() ? 0x08 : 0);
  // SoC: o inteligente (contagem de energia) quando disponível; senão tensão
  uint8_t soc = t.soc_pct;
  if (soc == 0) {
    float v = t.vbat_v;
    if      (v <= 32.0f) soc = 0;
    else if (v >= 42.0f) soc = 100;
    else                 soc = (uint8_t)((v - 32.0f) * 10.0f);
  }
  uint16_t gap   = t.loop_gap_us;
  uint16_t il10  = (uint16_t)(t.i_limit_a * 10.0f);
  uint16_t vm10  = (uint16_t)(t.max_kmh * 10.0f);
  uint16_t vl10  = (uint16_t)(t.v_limit * 10.0f);
  uint16_t za1k  = (uint16_t)constrain(t.zea * 1000.0f, 0.0f, 65000.0f);
  uint16_t trip_dam = (uint16_t)min(t.trip_m / 10UL, (unsigned long)65000UL);
  uint16_t whkm10   = (uint16_t)constrain(t.wh_per_km * 10.0f, 0.0f, 65000.0f);
  uint16_t capwh    = t.cap_wh;   // capacidade aprendida (bytes 34-35)
  uint16_t twh10    = (uint16_t)constrain(t.trip_wh * 10.0f, 0.0f, 65000.0f);
  uint16_t twr10    = (uint16_t)constrain(t.trip_wh_regen * 10.0f, 0.0f, 65000.0f);
  uint8_t flags2 = (t.cruise ? 0x01 : 0) | (t.walk ? 0x02 : 0)
                 | (t.locked ? 0x04 : 0) | (t.saturated ? 0x08 : 0)
                 | ((t.faults & FAULT_UV_WARN) ? 0x10 : 0)
                 | ((t.ride & 0x03) << 5)
                 | (t.eco_auto ? 0x80 : 0);

  b[0] = kmh10 & 0xFF;  b[1] = kmh10 >> 8;
  b[2] = vb10 & 0xFF;   b[3] = vb10 >> 8;
  b[4] = ib10 & 0xFF;   b[5] = ib10 >> 8;
  b[6] = pw & 0xFF;     b[7] = pw >> 8;
  b[8] = (uint8_t)(int8_t)constrain(t.temp_c, -127.0f, 127.0f);
  b[9] = soc;
  b[10] = flags;
  b[11] = (uint8_t)(t.faults & 0xFF);
  b[12] = gap & 0xFF;   b[13] = gap >> 8;
  b[14] = tg10 & 0xFF;  b[15] = tg10 >> 8;
  b[16] = (uint8_t)constrain(t.power_scale * 100.0f, 0.0f, 100.0f);
  b[17] = t.fw_x2;              // avanço de field weakening [°x2]
  b[18] = il10 & 0xFF;  b[19] = il10 >> 8;
  b[20] = vm10 & 0xFF;  b[21] = vm10 >> 8;
  b[22] = vl10 & 0xFF;  b[23] = vl10 >> 8;
  b[24] = za1k & 0xFF;  b[25] = za1k >> 8;
  // --- extensão F2 (26..43) ---
  b[26] = t.odo_m & 0xFF;         b[27] = (t.odo_m >> 8) & 0xFF;
  b[28] = (t.odo_m >> 16) & 0xFF; b[29] = (t.odo_m >> 24) & 0xFF;
  b[30] = trip_dam & 0xFF;  b[31] = trip_dam >> 8;
  b[32] = whkm10 & 0xFF;    b[33] = whkm10 >> 8;
  b[34] = capwh & 0xFF;     b[35] = capwh >> 8;
  b[36] = t.duty_pct;
  b[37] = t.headroom_pct;
  b[38] = flags2;
  b[39] = t.cmd_status;
  b[40] = twh10 & 0xFF;     b[41] = twh10 >> 8;
  b[42] = twr10 & 0xFF;     b[43] = twr10 >> 8;
  b[44] = (uint8_t)t.mtpa_x10;   // trim MTPA [°x10, com sinal]
  b[45] = (t.headlight ? 0x01 : 0)             // luzes: b0 farol
        | (s_brakeLightOn ? 0x02 : 0);         //        b1 luz de freio acesa

  // ===== BLOCO PROFUNDO (46+): estado do CORE em tempo real p/ debug =====
  auto i16 = [&](int o, float v, float sc) {
    int16_t x = (int16_t)constrain(v * sc, -32000.0f, 32000.0f);
    b[o] = x & 0xFF; b[o + 1] = (x >> 8) & 0xFF; };
  auto u16 = [&](int o, uint32_t v) {
    if (v > 65535) v = 65535; b[o] = v & 0xFF; b[o + 1] = (v >> 8) & 0xFF; };
  i16(46, t.vel_est, 100);        // velocidade interna [rad/s x100]
  i16(48, t.v_q, 100);            // tensão eixo q [V x100]
  i16(50, t.v_d, 100);            // tensão eixo d [V x100]
  i16(52, t.i_raw, 100);          // corrente CRUA [A x100]
  u16(54, (uint32_t)(constrain(t.eff_v_limit, 0.0f, 650.0f) * 100.0f));
  // (era a back-EMF estimada; agora a MAGNITUDE de corrente de fase MEDIDA,
  //  que é o escalar que de fato governa torque e proteção)
  u16(56, (uint32_t)(constrain(t.iph_mag, 0.0f, 650.0f) * 100.0f));
  b[58] = t.temp_scale;  b[59] = t.tracv_scale;
  b[60] = (uint8_t)t.elec_sector;  b[61] = t.ovr;  b[62] = t.coast_active ? 1 : 0;
  b[63] = t.hall_bits;   b[64] = t.hall_seen;
  u16(65, t.hall_eps);   u16(67, t.eps_a);  u16(69, t.eps_b);  u16(71, t.eps_c);
  u16(73, t.seq_skips);  u16(75, t.seq_revs);  u16(77, t.seq_inval);
  u16(79, t.rej_filt_a); u16(81, t.rej_filt_b); u16(83, t.rej_filt_c);
  u16(85, t.rej_conf_a); u16(87, t.rej_conf_b); u16(89, t.rej_conf_c);
  u16(91, t.hall_bad);   u16(93, t.link_drops); u16(95, t.loop_gap_us);
  // entradas do piloto + alvos/ângulo internos
  u16(97,  (uint32_t)(constrain(t.throttle, 0.0f, 1.0f) * 1000.0f));
  u16(99,  (uint32_t)(constrain(t.brake,    0.0f, 1.0f) * 1000.0f));
  u16(101, (uint32_t)(constrain(t.thr_eff,  0.0f, 1.0f) * 1000.0f));
  u16(103, (uint32_t)(constrain(t.brk_eff,  0.0f, 1.0f) * 1000.0f));
  u16(105, t.loop_hz);
  i16(107, t.vel_target, 100);    // alvo de velocidade [rad/s x100]
  // 109-110: slot RESERVADO. Era a tensão do regulador de costeira, que
  // deixou de existir (costeira virou Iq alvo = 0) e transmitia zero fixo
  // desde então. Fica zerado para não deslocar o pacote inteiro — o
  // desalinhamento silencioso entre app e firmware é caro demais.
  u16(109, 0);
  u16(111, (uint32_t)(constrain(t.elec_angle, 0.0f, 6.2832f) * 1000.0f));
  // ===== detalhe MÁXIMO: timing real, duties por fase, mecânica, saúde =====
  uint32_t us = micros();
  b[113] = us & 0xFF; b[114] = (us >> 8) & 0xFF;
  b[115] = (us >> 16) & 0xFF; b[116] = (us >> 24) & 0xFF;   // timestamp ESP [µs]
  u16(117, (uint32_t)(constrain(t.dc_a, 0.0f, 1.0f) * 1000.0f));
  u16(119, (uint32_t)(constrain(t.dc_b, 0.0f, 1.0f) * 1000.0f));
  u16(121, (uint32_t)(constrain(t.dc_c, 0.0f, 1.0f) * 1000.0f));
  u16(123, (uint32_t)(fmodf(fabsf(t.shaft_angle), 6.2832f) * 1000.0f));  // mec [rad]
  i16(125, t.shaft_vel, 100);
  b[127] = (uint8_t)(constrain(t.pscale_fold, 0.0f, 1.0f) * 100.0f);     // foldback
  u16(128, (uint32_t)(constrain(t.eco_v, 0.0f, 650.0f) * 100.0f));
  u16(130, (uint32_t)(constrain(t.regen_ilim, 0.0f, 650.0f) * 100.0f));
  u16(132, t.ms_since_cmd);
  u16(134, (uint32_t)(ESP.getFreeHeap() / 1024));          // RAM livre [KB]
  b[136] = (uint8_t)(int8_t)constrain(t.motor_temp_c, -127.0f, 127.0f);  // temp motor
  // ===== CORRENTE DE FASE (137..152) — 2x INA240A1 low-side =====
  i16(137, t.iph_a, 100);       // [A x100] fase A
  i16(139, t.iph_b, 100);       // [A x100] fase B
  i16(141, t.iph_c, 100);       // [A x100] fase C (reconstruída se só 2 sensores)
  i16(143, t.iph_d, 100);       // [A x100] Id medido
  i16(145, t.iph_q, 100);       // [A x100] Iq medido
  i16(147, t.iq_target, 100);   // [A x100] Iq PEDIDO (alvo da malha)
  b[149] = t.duty_max_pct;     // [%] duty máximo da janela
  b[150] = (t.iph_ok ? 0x01 : 0)          // b0 = calibração válida
         | ((t.iph_ref & 0x0F) << 1);     // b1-4 = classificação do REF
  // Zeros medidos: dizem se o módulo INA240 é bidirecional (o app mostra).
  u16(151, (uint32_t)(constrain(t.iph_zero1, 0.0f, 5.0f) * 1000.0f));
  u16(153, (uint32_t)(constrain(t.iph_zero2, 0.0f, 5.0f) * 1000.0f));
  return 155;
}

// Frame do STREAM de alta taxa (20 B) a partir da amostra rápida do núcleo 1
static size_t packStream(uint8_t *b) {
  float kmh, ibat, vbat, tgt; uint8_t duty; int8_t temp;
  motorFastSample(&kmh, &ibat, &vbat, &tgt, &duty, &temp);
  uint16_t tms   = (uint16_t)millis();
  int16_t  kmh10 = (int16_t)constrain(kmh * 10.0f, -32000.0f, 32000.0f);
  int16_t  ib10  = (int16_t)constrain(ibat * 10.0f, -32000.0f, 32000.0f);
  uint16_t vb10  = (uint16_t)constrain(vbat * 10.0f, 0.0f, 65000.0f);
  int16_t  tg10  = (int16_t)constrain(tgt * 10.0f, -32000.0f, 32000.0f);
  b[0] = tms & 0xFF;   b[1] = tms >> 8;
  b[2] = kmh10 & 0xFF; b[3] = kmh10 >> 8;
  b[4] = ib10 & 0xFF;  b[5] = ib10 >> 8;
  b[6] = vb10 & 0xFF;  b[7] = vb10 >> 8;
  b[8] = tg10 & 0xFF;  b[9] = tg10 >> 8;
  b[10] = duty;        b[11] = (uint8_t)temp;
  // 12-15: performance (FW, foldback, MTPA, headroom térmico) — app antigo
  // que só lê 12 B continua funcionando (campos extras no fim)
  uint8_t fw2, scale, head; int8_t mtpa10;
  motorPerfSample(&fw2, &scale, &mtpa10, &head);
  b[12] = fw2; b[13] = scale; b[14] = (uint8_t)mtpa10; b[15] = head;
  // 16-19: corrente de fase medida x pedida — é o par que se olha para ajustar
  // o PI de corrente (o erro entre eles É a malha).
  float iqm, iqt;
  motorCurSample(&iqm, &iqt);
  int16_t iqm10 = (int16_t)constrain(iqm * 10.0f, -32000.0f, 32000.0f);
  int16_t iqt10 = (int16_t)constrain(iqt * 10.0f, -32000.0f, 32000.0f);
  b[16] = iqm10 & 0xFF; b[17] = iqm10 >> 8;
  b[18] = iqt10 & 0xFF; b[19] = iqt10 >> 8;
  return 20;
}

// ---------------------------------------------------------------------------
// TASK PRINCIPAL — criada no núcleo 0
// ---------------------------------------------------------------------------
void bleDashboardTask(void *arg) {
#if BRAKE_LIGHT_ENABLED
  ledcSetup(BRAKE_LIGHT_LEDC_CH, 2000, 8);
  ledcAttachPin(PIN_BRAKE_LIGHT, BRAKE_LIGHT_LEDC_CH);
  ledcWrite(BRAKE_LIGHT_LEDC_CH, 0);
#endif
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setMTU(247);                      // pacote de parametros ~200 B
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);         // TX moderado (economia/rail)

  s_srv = NimBLEDevice::createServer();
  NimBLEServer *srv = s_srv;
  srv->setCallbacks(new SrvCallbacks());

  NimBLEService *svc = srv->createService(BLE_UUID_SERVICE);
  s_telemChar = svc->createCharacteristic(BLE_UUID_TELEMETRY,
                                          NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic *ctrl =
      svc->createCharacteristic(BLE_UUID_CONTROL,
                                NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  ctrl->setCallbacks(new CtrlCallbacks());
  s_paramsChar = svc->createCharacteristic(BLE_UUID_PARAMS,
                                NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  s_paramsChar->setCallbacks(new ParamsCallbacks());
  s_streamChar = svc->createCharacteristic(BLE_UUID_STREAM,
                                NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  s_dbgChar    = svc->createCharacteristic(BLE_UUID_DEBUG,
                                NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic *curveCh = svc->createCharacteristic(BLE_UUID_CURVE,
                                NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  curveCh->setCallbacks(new CurveCallbacks());
  s_bulkChar = svc->createCharacteristic(BLE_UUID_BULK, NIMBLE_PROPERTY::READ);
  s_bulkChar->setCallbacks(new BulkCallbacks());
  svc->start();

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_UUID_SERVICE);
  adv->setScanResponse(true);
  adv->start();
  Serial.printf("[ble] anunciando como \"%s\"\n", BLE_DEVICE_NAME);

  uint8_t buf[160];   // telemetria profunda (~136 B) + stream (16 B)
  uint32_t n20 = 0;
  for (;;) {
    n20++;
    // OTA pedido pelo app: entra em modo manutenção (nunca retorna)
    if (otaRequested()) otaRun();

#if BRAKE_LIGHT_ENABLED
    // Luz de freio a 50 Hz: manete OU regen forte acende cheio; farol ligado
    // deixa a lanterna fraca (PWM); nada = apagada.
    {
      Command   lc = readCommand();
      Telemetry lt = readTelemetry();
      bool braking = lc.brake > 0.03f || lt.ibat_a < -1.5f;
      ledcWrite(BRAKE_LIGHT_LEDC_CH,
                braking ? 255 : (lc.headlight ? BRAKE_LIGHT_DIM_DUTY : 0));
      s_brakeLightOn = braking;
    }
#endif

    // Telemetria completa no período do config (BLE_TELEM_MS). O divisor sai
    // do próprio define em vez de um "5" mágico: antes o número estava fixo
    // aqui e o BLE_TELEM_MS não fazia nada — mudá-lo no config não tinha
    // efeito nenhum, o tipo de mentira silenciosa que já custou caro aqui.
    if (n20 % (BLE_TELEM_MS / BLE_TASK_TICK_MS) == 0 && s_connected && s_telemChar) {
      size_t n = packTelemetry(buf);
      s_telemChar->setValue(buf, n);
      s_telemChar->notify();
    }
    // STREAM de alta taxa (25 Hz = tick sim/tick não; 50 Hz = todo tick)
    if (s_connected && s_streamChar && s_streamHz > 0) {
      if (s_streamHz >= 50 || (n20 & 1) == 0) {
        size_t n = packStream(buf);
        s_streamChar->setValue(buf, n);
        s_streamChar->notify();
      }
    }
    // DATALOGGER DE DEBUG: pacote RICO completo (113 B) em alta taxa. Mesmo
    // conteúdo da telemetria, mas até 50 Hz (5x) — datalogger de baixo nível.
    if (s_connected && s_dbgChar && s_dbgHz > 0) {
      uint8_t div = (s_dbgHz >= 50) ? 1 : (uint8_t)(50 / s_dbgHz);
      if (div < 1) div = 1;
      if (n20 % div == 0) {
        size_t n = packTelemetry(buf);
        s_dbgChar->setValue(buf, n);
        s_dbgChar->notify();
      }
    }
    // persistências adiadas (todas gravam SÓ com a roda parada)
    if (n20 % 50 == 0) {
      float kmhAbs = fabsf(readTelemetry().kmh);
      paramsTick(kmhAbs);
      curvesTick(kmhAbs);
      faultLogTick(kmhAbs);
      motorEnergyTick(kmhAbs);
    }
    vTaskDelay(pdMS_TO_TICKS(BLE_TASK_TICK_MS));
  }
}
