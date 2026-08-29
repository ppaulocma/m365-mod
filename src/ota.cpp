// ============================================================================
// ota.cpp — MODO MANUTENÇÃO: atualização de firmware por WiFi sob demanda
// ----------------------------------------------------------------------------
// WiFi NUNCA coexiste com pilotagem (lição da guerra dos congelamentos).
// O app manda o comando OTA -> motor é desabilitado e TRAVADO (estop),
// sobe um Access Point próprio com uma página de upload, e o ESP reinicia
// sozinho ao fim do upload. Sem upload em 5 min, reinicia de volta ao normal.
//   Rede:  Patinete-OTA  /  senha patinete123  ->  http://192.168.4.1
// ============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include "config.h"
#include "shared_state.h"
#include "ota.h"

static volatile bool s_otaReq = false;

void otaRequest() { s_otaReq = true; }
bool otaRequested() { return s_otaReq; }

static const char PAGE[] =
  "<meta name=viewport content='width=device-width,initial-scale=1'>"
  "<body style='font-family:sans-serif;background:#0B0F10;color:#F2F5F4;"
  "padding:24px;max-width:420px;margin:auto'>"
  "<h2>Patinete &mdash; atualiza&ccedil;&atilde;o de firmware</h2>"
  "<p>Selecione o <code>firmware.bin</code> (pasta .pio/build) e envie. "
  "O patinete reinicia sozinho ao terminar.</p>"
  "<form method=POST action=/update enctype=multipart/form-data>"
  "<input type=file name=fw accept=.bin style='margin:12px 0'><br>"
  "<input type=submit value='Enviar firmware' "
  "style='background:#FF7A1A;color:#1A0E02;border:0;padding:12px 20px;"
  "font-weight:700'></form></body>";

// Roda DENTRO da task BLE (nunca retorna: termina em reboot)
void otaRun() {
  // Trava o veículo: estop + desabilita (o núcleo 1 continua rodando,
  // com o motor cortado — seguro)
  Command c = readCommand();
  c.estop = true; c.enable = false; c.throttle = 0; c.brake = 0;
  writeCommand(c);
  vTaskDelay(pdMS_TO_TICKS(200));

  Serial.println("[ota] MODO MANUTENCAO: subindo AP Patinete-OTA...");
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Patinete-OTA", "patinete123");

  static WebServer srv(80);
  srv.on("/", HTTP_GET, []() { srv.send(200, "text/html", PAGE); });
  srv.on("/update", HTTP_POST,
    []() {
      bool ok = !Update.hasError();
      srv.send(200, "text/plain", ok ? "OK — reiniciando" : "FALHOU");
      delay(500);
      ESP.restart();
    },
    []() {
      HTTPUpload &up = srv.upload();
      if (up.status == UPLOAD_FILE_START) {
        Serial.printf("[ota] recebendo %s\n", up.filename.c_str());
        Update.begin(UPDATE_SIZE_UNKNOWN);
      } else if (up.status == UPLOAD_FILE_WRITE) {
        Update.write(up.buf, up.currentSize);
      } else if (up.status == UPLOAD_FILE_END) {
        Update.end(true);
        Serial.printf("[ota] fim: %u bytes, %s\n", up.totalSize,
                      Update.hasError() ? "ERRO" : "ok");
      }
    });
  srv.begin();
  Serial.println("[ota] http://192.168.4.1 — aguardando firmware (5 min)");

  uint32_t t0 = millis();
  for (;;) {
    srv.handleClient();
    vTaskDelay(1);
    if (millis() - t0 > 5 * 60 * 1000UL) {
      Serial.println("[ota] timeout sem upload — reiniciando");
      ESP.restart();
    }
  }
}
