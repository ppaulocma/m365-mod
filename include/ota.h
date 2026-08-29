// ============================================================================
// ota.h — modo manutenção: atualização de firmware por WiFi sob demanda
// ============================================================================
#pragma once
void otaRequest();     // chamado pelo comando BLE 40
bool otaRequested();
void otaRun();         // roda na task BLE; nunca retorna (termina em reboot)
