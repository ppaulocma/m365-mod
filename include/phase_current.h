// ============================================================================
// phase_current.h — corrente de FASE medida (2x INA240A1, shunts low-side)
// ----------------------------------------------------------------------------
// Reescrito do zero. O desenho segue o que controladoras de verdade (VESC,
// ODrive, ST MC SDK) fazem com shunt low-side, adaptado ao hardware do ESP32
// clássico (UM SAR ADC, sem trigger de hardware, sem dupla amostragem):
//
//   1) PWM center-aligned. O driver MCPWM da SimpleFOC configura os três
//      timers em contador UP-DOWN sincronizados no TEZ do timer 0, duty mode
//      ativo-alto. Disso decorre, por construção (não por calibração):
//         - janela HIGH  (MOSFET de cima)  centrada no TEZ, largura duty·T
//         - janela LOW   (MOSFET de baixo) centrada no TEP, largura (1−duty)·T
//      O instante de amostragem da corrente é o CENTRO da janela low-side
//      (TEP): além de ser o ponto mais longe das bordas de chaveamento, a
//      corrente ali vale exatamente a sua MÉDIA no período — o ripple de
//      chaveamento se cancela sozinho, sem filtro.
//
//   2) ZERO VIVO, medido por período — a correção de offset que as camadas
//      antigas (canal de referência no GPIO36, rastreio por variância,
//      mediana, âncoras) tentavam imitar. No TEZ o MOSFET de baixo está
//      DESLIGADO (janela high-side): não passa corrente pelo shunt e a saída
//      do INA240 é exatamente REF — o zero daquele canal, naquele instante,
//      já incluindo deriva do trilho de 3,3 V, deriva térmica e a oscilação
//      da referência do ADC induzida pelo rádio BLE, porque é o MESMO ADC
//      lendo. Amostrando corrente no TEP e zero no TEZ, a subtração
//      (corrente − zero) cancela tudo que é de modo comum por construção.
//      O erro residual do rádio cai de ~1,2 A para ~0,1 A: o desvio da
//      referência passa a multiplicar só a tensão DE SINAL (G·Rsh·I), não o
//      1,65 V inteiro do repouso.
//
//   3) AMOSTRA VALIDADA POR HARDWARE. Cada conversão guarda o valor do
//      CONTADOR do timer no instante do disparo (timer_status). Se o S&H do
//      ADC não couber dentro da janela de condução correspondente (calculada
//      contra o compare REAL da fase, lido do registrador), a amostra é
//      DESCARTADA e contada — nunca entra um número medido fora da janela.
//      O teto de modulação continua garantindo que isso não aconteça em
//      operação; a validação é a prova, não o remendo.
//
//   4) SEM MÉDIA NO REFERENCIAL DO ESTATOR. As correntes de fase são AC (até
//      ~200 Hz elétricos neste motor): a média de 3 ms da versão anterior as
//      atenuava e defasava dezenas de graus — era por isso que o Iq medido
//      "sumia" com o motor andando e a malha disparava. Aqui cada canal usa
//      a ÚLTIMA amostra válida (8 kHz por canal) e o filtro fica onde ele é
//      matematicamente correto: em Id/Iq (referencial rotante, sinal DC em
//      regime), com Tf = P_CUR_TF.
//
// CADEIA DE MEDIÇÃO
//   shunt low-side -> INA240A1 (REF = V+/2) -> ADC1 (GPIO 34/39)
//   ISR do MCPWM nos DOIS eventos do timer 0 (TEZ e TEP), nível 3 (preempta
//   as ISRs de Hall — o instante do S&H não pode esperar por elas), em
//   PIPELINE: cada evento colhe a conversão disparada no evento anterior e
//   dispara a próxima. Rodízio: TEP corrente ch0 | TEZ zero ch0 | TEP
//   corrente ch1 | TEZ zero ch1. Cada canal: corrente E zero a 8 kHz.
//
// A JANELA DE AMOSTRAGEM — a conta certa
//   O S&H acontece DEPOIS do evento (latência de ISR + disparo), então só a
//   METADE da janela após o evento está disponível:
//       (1 − duty)·T/2  >=  t_latencia + t_sh + margem
//       duty_max = 1 − 2·(t_latencia + IPH_SAMPLE_US + PHASE_ISR_MARGIN_US)/T
//   (a versão anterior omitia o fator 2 — em modulação alta o S&H caía FORA
//   da janela e a malha lia lixo justamente perto da velocidade máxima).
//   A latência é MEDIDA no boot pelo próprio contador do timer; o teto de
//   modulação usa o valor medido e aparece no boot e no app.
//
// MAPA DE FASES, SINAL E ESCALA: continuam MEDIDOS pela calibração de
// bancada (roda suspensa) e gravados na NVS — fiação é fato físico que não
// se adivinha. O que DEIXOU de ser calibrado é o instante de amostragem:
// ele é derivado da configuração do PWM (com um fallback de verificação que
// só age, avisando alto, se uma atualização da lib inverter a polaridade).
// Sem calibração válida a malha de corrente fica bloqueada e o motor não
// liga — sinal de sensor invertido é realimentação positiva.
//
// REFERÊNCIAS (o desenho segue prática de produção, não invenção):
//   - Centro do vetor zero como instante de amostragem: NXP AN14164; VESC
//     (mcpwm_foc.c, ADC no topo do contador = centro de V0); ODrive
//     (board.cpp, amostra nos centros de V0 e V7); ST UM1052 §5.1.
//   - Zero vivo amostrado no vetor oposto: é o dc_calib do ODrive
//     (motor.cpp dc_calib_cb — shunts lidos no centro de V7, onde carregam
//     0 A, com IIR por fase e trava de plausibilidade). Aqui o IIR é mais
//     rápido (τ~2 ms) de propósito: além do térmico, acompanha as rajadas
//     de ~3 ms que o rádio BLE induz na referência do ADC.
//   - Filtro no referencial dq, feedback sem filtro abc: VESC (id/iq_filter
//     só para exibição) e SimpleFOC (LPF_current_q/d, default 5 ms).
//   - LIMITAÇÃO CONHECIDA: os dois canais são amostrados com 1 período de
//     PWM de defasagem (um SAR só no ESP32 clássico; a própria SimpleFOC
//     faz o mesmo rodízio). Erro ≤ ~4,5° elétricos / ~8% de |I| no topo de
//     velocidade, como ondulação em 2·f_e que o filtro dq amortece. A
//     correção por ângulo-por-amostra (estilo ODrive) exigiria ângulo fino,
//     que o Hall de 60° não fornece — abaixo do ruído de ângulo, não vale o
//     código. Caminho de upgrade real: pré-disparo por comparador reserva
//     (como a SimpleFOC dev 2.4.0 faz) e/ou ADC via I2S/DMA.
// ============================================================================
#pragma once
#include <Arduino.h>
#include "config.h"

#if PHASE_CUR_ENABLED

class CurrentSense;   // fwd (SimpleFOC)
class BLDCDriver;
class BLDCMotor;

// Amostra completa das correntes de fase.
struct PhaseCur {
  float ia, ib, ic;      // [A] correntes de fase CRUAS (a 3ª de ia+ib+ic = 0)
  float ialpha, ibeta;   // [A] Clarke (referencial estator, crus)
  float imag;            // [A] MAGNITUDE do vetor — independe do ângulo, então
                         //     é exata mesmo com o Hall grosso. É por ela que
                         //     a proteção dispara. SEM filtro.
  float id, iq;          // [A] Park FILTRADOS com Tf = P_CUR_TF (é o que a
                         //     malha consome; o filtro é aqui, no referencial
                         //     rotante, onde o sinal é DC em regime)
};

// Classificação do zero medido — diz como o módulo ligou os pinos REF.
enum : uint8_t {
  IPH_REF_BIDIR = 0,   // zero em ~V+/2: mede os dois sentidos (o que serve)
  IPH_REF_LOW   = 1,   // zero em ~0 V: REF no GND, unidirecional
  IPH_REF_HIGH  = 2,   // zero em ~V+: REF no V+, unidirecional invertido
  IPH_REF_NONE  = 3,   // ainda não medido, ou zero sem sentido físico
};

// Resultado da calibração (persistido em NVS, versão 2).
struct PhaseCal {
  bool    valid;          // calibração completa e coerente
  uint8_t event;          // 0 = TEP (derivado do PWM; ver header). 1 = TEZ só
                          //   se o fallback de verificação detectar inversão.
  int8_t  ph[2];          // fase medida em cada pino: 0=A, 1=B, 2=C
  int8_t  sign[2];        // sinal de cada canal (+1/-1)
  float   offset[2];      // [V] zero VIVO de cada canal (p/ telemetria/app)
  bool    zeroed;         // o zero já foi medido alguma vez
  float   zeroRef[2];     // [V] zero da CALIBRAÇÃO — cerco de sanidade: zero
                          //   vivo longe demais dele = canal morto/curto
  uint8_t ref[2];         // IPH_REF_* — DERIVADO do zero, nunca persistido
  float   ampsPerVolt[2]; // [A/V] escala MEDIDA por canal (cruzada com o
                          //   INA226 na calibração; 0 = usa a nominal)
  float   resp[2][3];     // [V] matriz de resposta canal x fase (diagnóstico)
};

// Inicializa ADC + ISR do MCPWM e devolve o objeto p/ motor.linkCurrentSense().
// Chamar DEPOIS de driver.init() e ANTES de motor.initFOC().
CurrentSense *phaseCurrentInit(BLDCDriver *drv, BLDCMotor *mot);

// CALIBRAÇÃO DE BANCADA — roda suspensa, fonte limitada em corrente.
// Mede zero -> resposta de cada canal a vetores DC (±) em cada fase ->
// mapa/sinal/escala. Grava na NVS. Bloqueia ~4 s. Sem ela o motor não roda
// em malha de corrente — de propósito.
bool phaseCurrentCalibrate();

// Mede o ZERO dos canais e classifica o REF (motor desabilitado, roda parada).
bool phaseCurrentZero();

bool  phaseCurrentCalValid();
const PhaseCal *phaseCurrentCal();

// Instantâneo cru p/ caixa-preta: dev = leitura−zeroCal [LSB], zoff = zero
// vivo−zeroCal [LSB], acc = amostras aceitas desde a última chamada (por
// canal; cadência sã ≈ 8/ms). Separa buraco / passeio de zero / corrupção.
void  phaseCurrentRawDbg(int16_t dev[2], int8_t zoff[2], uint8_t acc[2]);

// Teto de duty de OPERAÇÃO — abaixo do físico, com folga p/ latência+jitter
// (IPH_OP_FOLGA_US): é DESTE que o vModCeil da malha deve partir. Operar no
// duty máximo físico deixa cada fase cega no arco do próprio pico de duty.
float phaseCurrentDutyOp();

// Reafirma FORCE_XPD_SAR=3 (trava de power do SAR — errata GPIO-3.11).
// Chamar após cada rajada de analogReadMilliVolts (ADC2 desfaz a trava).
void  phaseCurrentSarPowerAssert();

// Latência média de disparo medida [µs] — diagnóstico.

// Converte as últimas amostras válidas em correntes (chamada no núcleo 1).
// id/iq saem FILTRADOS (Tf = P_CUR_TF); o resto é cru.
void phaseCurrentRead(PhaseCur *out, float angleEl);

// Rastreio do zero em REPOUSO (motor desabilitado + roda parada — quem
// garante as condições é o chamador). Complementa o zero vivo do TEZ, que
// precisa de duty > ~15% para ter janela: parado com duty 0 é ESTE caminho
// que mantém o zero colado. snap = puxão final mais forte (ainda filtrado),
// usado ao habilitar o motor — o último instante de corrente comprovadamente
// nula antes de a malha assumir.
void phaseCurrentTrackZero(float dt, bool snap);

// true = o zero vivo saiu do cerco físico em torno do zero da calibração.
// Não é deriva (o cerco é largo): é canal morto, em curto ou desligado.
// O zero em uso fica preso no último valor são; isto é um AVISO.
bool phaseCurrentZeroPinned();

// true = a saída do INA240 encostou num trilho AGORA. Saturado, o sensor lê
// MENOS que a corrente real — a malha veria folga onde não há. Corta.
bool phaseCurrentClipping();

// false = NENHUMA amostra nova de corrente há >3 ms: a malha está CEGA
// (janela fechada, ISR morta, ADC preso). O motor_control corta por isto —
// malha que continua empurrando sem realimentação é o disparo clássico.
bool phaseCurrentFresh();

// Despejo de diagnóstico no serial: zeros, escalas, estatística de ruído,
// contadores de amostras rejeitadas, latência, tabela de amostras.
void phaseCurrentDiag(uint32_t ms);

// Captura 100 ms a 4 kHz e desenha no serial (estrutura temporal do ruído).

#endif  // PHASE_CUR_ENABLED
