// ============================================================================
// curves.h — motor de CURVAS configuráveis (lookup tables com interpolação)
// ----------------------------------------------------------------------------
// O app manda tabelas de pontos; o firmware VALIDA contra limites físicos
// hard-coded e interpola linearmente no núcleo de controle (double-buffer:
// leitura sem lock, escrita troca o banco ativo — nunca lê curva pela metade).
// Unidades dos eixos são fixas por curva (documentadas abaixo) — o app usa
// as mesmas.
// ============================================================================
#pragma once
#include <Arduino.h>

#define CURVE_MAX_PTS 8

enum : uint8_t {
  CRV_THROTTLE = 1,  // x: acelerador 0..100 %      y: saída 0..100 %
  CRV_BRAKE    = 2,  // x: manete 0..100 %          y: força de regen 0..100 %
  CRV_REGEN_V  = 3,  // x: tensão do pack V*10      y: regen permitido 0..100 %
                     //    (SOBRETENSÃO: 0 % com bateria cheia!)
  CRV_TRAC_V   = 4,  // x: tensão do pack V*10      y: tração permitida 0..100 %
                     //    (derate progressivo com bateria fraca)
  CRV_THERMAL  = 5,  // x: temperatura °C           y: potência 0..100 %
  CRV_REGEN_S  = 6,  // x: velocidade km/h          y: regen 0..100 %
                     //    (fraco em baixa p/ não travar a roda)
  CRV__COUNT
};
#define CURVE_N (CRV__COUNT - 1)

void  curvesInit();                      // NVS + defaults (equivalem ao
                                         // comportamento antigo)
float curveEval(uint8_t id, float x);    // núcleo 1: interp linear, sem lock
bool  curveSet(uint8_t id, const int16_t *x, const int16_t *y, uint8_t n);
                                         // valida (faixas físicas, monotônico);
                                         // false = REJEITADO, nada aplicado
int   curveGet(uint8_t id, uint8_t *buf, int max);  // [id][n][x i16,y i16]*n
void  curvesFactory();                   // todas de volta ao default
void  curvesTick(float kmhAbs);          // commit NVS adiado (roda parada)
