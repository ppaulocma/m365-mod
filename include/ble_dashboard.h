// ============================================================================
// ble_dashboard.h — dashboard via Bluetooth Low Energy (substitui a web)
// ----------------------------------------------------------------------------
// Servidor GATT (NimBLE — o stack mais leve p/ ESP32) com um serviço e duas
// características:
//
//   TELEMETRIA (notify, período BLE_TELEM_MS, 155 bytes little-endian):
//     O pacote cresce por CAMADAS e o app decodifica pelo tamanho, então
//     firmware novo continua abrindo em app velho. O mapa completo, campo a
//     campo, é o packTelemetry() no .cpp — mantê-lo duplicado aqui só criava
//     documentação que envelhece sozinha (este cabeçalho jurou "26 bytes"
//     por muito tempo depois de o pacote passar de 150).
//     flags b[10]: b0 ligado | b1 modo velocidade | b2 regen | b3 painel
//
//   CONTROLE (write): sequência de comandos [id u8][valor f32 LE] (5 B cada):
//     1 th 0..1 | 2 br 0..1 | 3 enable 0/1 | 4 estop | 5 limpar faltas
//     6 modo 0/1 | 7 corrente máx A | 8 vel máx km/h (sessão) | 9 tensão máx V
//     10 ângulo zero rad
//     20 SALVAR na flash | 21 restaurar defaults de fábrica
//     -- calibração (a ORDEM importa; ver Config.js do app) --
//     26 CALIBRAR corrente de fase (roda suspensa; libera a malha)
//     22 calibrar ângulo + DIREÇÃO do sensor (GIRA o motor!)
//     29 afinar o ângulo GIRANDO (varredura; valor = km/h do teste)
//     23 medir Ke (roda livre)
//     -- diagnóstico --
//     27 despejo cru do sensor (5 s, motor parado) | 28 degrau da malha (A)
//     41 caixa-preta da última falta que travou
//     -- geral --
//     30 taxa do stream | 31 step response (amp km/h) | 32 fonte BULK
//     33 liberar captura | 34 selecionar curva p/ leitura | 36 modo pilotagem
//     37 cruise | 38 walk | 39 trava | 40 modo OTA | 44 zera trip | 45 limpa log
//     46 farol | 47 taxa do datalogger de debug
//     (24 auto-cal e 25 re-zero foram REMOVIDOS — ver ble_dashboard.cpp)
//   th/br são IGNORADOS enquanto o painel físico estiver ativo (mesma regra
//   da dashboard antiga). enable/estop/limites valem sempre.
//
// Toda a lógica pesada de BLE roda no núcleo 0 (NimBLE já se prende lá);
// o núcleo 1 continua exclusivo do motor.
// ============================================================================
#pragma once
#include "config.h"

// UUIDs do serviço (os mesmos estão no app React Native)
#define BLE_UUID_SERVICE   "7a0b1000-c1e5-4c0d-9a9e-8f5d0e5c0001"
#define BLE_UUID_TELEMETRY "7a0b1001-c1e5-4c0d-9a9e-8f5d0e5c0001"
#define BLE_UUID_CONTROL   "7a0b1002-c1e5-4c0d-9a9e-8f5d0e5c0001"
// PARÂMETROS DE TUNING (params.h):
//   READ  -> [n u8] + n x [id u8, valor f32 LE, default f32 LE]
//   WRITE -> k x [id u8, valor f32 LE]  (aplica em RAM; persistir = cmd 20)
#define BLE_UUID_PARAMS    "7a0b1003-c1e5-4c0d-9a9e-8f5d0e5c0001"
// STREAM alta taxa (notify, 20 B: t u16, kmh10 i16, ibat10 i16, vbat10 u16,
// tgt10 i16, duty u8, temp i8, fw u8, scale u8, mtpa i8, head u8,
// iq_med10 i16, iq_alvo10 i16) — taxa via cmd 30 (0/25/50 Hz)
#define BLE_UUID_STREAM    "7a0b1004-c1e5-4c0d-9a9e-8f5d0e5c0001"
// CURVAS (read: cmd 34 seleciona; write: [id][n][x i16,y i16]*n, validada)
#define BLE_UUID_CURVE     "7a0b1005-c1e5-4c0d-9a9e-8f5d0e5c0001"
// BULK (read sequencial; cmd 32 seleciona fonte: 1 captura, 2 log de faltas;
// leitura vazia = fim)
#define BLE_UUID_BULK      "7a0b1006-c1e5-4c0d-9a9e-8f5d0e5c0001"
// Datalogger de DEBUG: o MESMO pacote da telemetria, em alta taxa (até 50 Hz)
#define BLE_UUID_DEBUG     "7a0b1007-c1e5-4c0d-9a9e-8f5d0e5c0001"

// Task FreeRTOS (criar no núcleo 0): inicializa o NimBLE e faz o broadcast
// de telemetria a cada BLE_TELEM_MS enquanto houver central conectada.
void bleDashboardTask(void *arg);

// true = app conectado (o painel físico mostra o ícone de Bluetooth)
bool bleConnected();

// Para/religa o rádio (advertising + conexões). Só para diagnóstico: o
// advertising transmite tanto quanto uma conexão, então medir "sem app" não é
// medir sem rádio. Derruba a conexão do app — o resultado sai pelo serial.
void bleRadioPause(bool pause);
