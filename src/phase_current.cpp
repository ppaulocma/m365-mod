// ============================================================================
// phase_current.cpp — 2x INA240A1 low-side, amostragem sincronizada ao MCPWM
// ----------------------------------------------------------------------------
// Ver o desenho completo no phase_current.h. Resumo do que roda aqui:
//
//   ISR (nível 3, IRAM, ~1,5 µs) nos DOIS eventos do timer 0 do MCPWM:
//     TEP (centro da janela low-side)  -> converte CORRENTE  (canais 0/1
//                                         alternados: 8 kHz por canal)
//     TEZ (centro da janela high-side) -> converte o ZERO VIVO do canal
//                                         (low FET desligado => shunt a 0 A
//                                         => saída do INA240 = REF exato)
//   Cada conversão é disparada num evento e colhida no seguinte (pipeline:
//   a ISR nunca espera o SAR). No disparo, o CONTADOR do timer é lido e a
//   amostra só vale se o S&H couber na janela de condução da fase, calculada
//   contra o compare REAL do registrador — amostra fora da janela é
//   descartada e contada, nunca usada.
//
//   Lado de leitura (task do motor, 1 kHz): corrente = (amostra − zero
//   vivo) x escala x sinal, mapa de fases da calibração, Clarke/Park, e o
//   ÚNICO filtro fica em Id/Iq (referencial rotante, Tf = P_CUR_TF) — nunca
//   nas correntes de fase, que são AC.
//
// POR QUE O ZERO VIVO RESOLVE O QUE TRÊS MECANISMOS ANTIGOS NÃO RESOLVIAM:
//   o zero do INA240 é REF = V+/2 e anda com o trilho de 3,3 V; a leitura do
//   ADC anda com a referência interna, que o rádio BLE faz oscilar. Todos
//   esses movimentos aparecem POR IGUAL na amostra de corrente e na amostra
//   de zero do MESMO canal, colhidas a 31 µs de distância pelo MESMO ADC — a
//   subtração cancela o modo comum por construção, sob carga, andando, com
//   rádio ligado. O canal de referência no GPIO 36, o rastreio por variância
//   e a âncora refAtZero eram aproximações disso por caminhos indiretos; o
//   GPIO 36 fica LIVRE.
// ============================================================================
#include <Arduino.h>
#include <math.h>
#include <Preferences.h>
#include <SimpleFOC.h>
#include "drivers/hardware_specific/esp32/esp32_driver_mcpwm.h"
#include "config.h"
#include "params.h"
#include "ina226.h"
#include "ble_dashboard.h"
#include "phase_current.h"

#if PHASE_CUR_ENABLED

#include "driver/mcpwm.h"
#include "driver/adc.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include <soc/sens_reg.h>

// Leitor de ADC do SimpleFOC — usado no init para configurar o SAR
// (atenuação 11 dB, 12 bits, modo por software) e resolver os canais. A ISR
// nunca o chama: ele resolve canal por tabela em flash, e ISR IRAM que toca
// flash trava o chip durante escrita de NVS. Na ISR, só registrador.
extern uint16_t adcRead(uint8_t pin);

#define ADC_V_PER_COUNT   (PHASE_CUR_MV_PER_COUNT * 0.001f)

// ---------------------------------------------------------------------------
// Acesso direto ao SAR1 (dispara sem esperar; colhe sem esperar)
// ---------------------------------------------------------------------------
static inline void IRAM_ATTR adcStartCh(int ch) {
  CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
  SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD,
                    (1 << ch), SENS_SAR1_EN_PAD_S);
  SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
}
static inline bool IRAM_ATTR adcResult(uint16_t *out) {
  if (!GET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DONE_SAR)) return false;
  *out = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START1_REG,
                            SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
  return true;
}

// ---------------------------------------------------------------------------
// Estado partilhado com a ISR (tudo inteiro — ISR Xtensa NÃO pode usar float)
// ---------------------------------------------------------------------------
enum : uint8_t { KIND_NONE = 0, KIND_CUR = 1, KIND_ZERO = 2 };

static mcpwm_dev_t      *s_dev  = nullptr;
static mcpwm_unit_t      s_unit = MCPWM_UNIT_0;
static BLDCMotor        *s_motor  = nullptr;
static BLDCDriver       *s_driver = nullptr;
static bool              s_ready  = false;
static PhaseCal          s_cal    = {};
static Preferences       s_nvs;

static int               s_ch[2]        = { -1, -1 };  // canais ADC1
static volatile uint8_t  s_evCur        = 0;   // 0 = corrente no TEP (normal)
                                               // 1 = corrente no TEZ (fallback,
                                               //     só se a lib inverter o PWM)
static uint32_t          s_period       = 5000; // ticks do timer (lido no init)
static float             s_ticksPerUs   = 160.0f;
static volatile uint32_t s_gateTicks    = 400;  // S&H + margem, em ticks
// fase de cada canal p/ o gate do zero (255 = sem calibração: zero vivo off)
static volatile uint8_t  s_zeroPhase[2] = { 255, 255 };

// últimas amostras válidas + sequência (p/ o lado de leitura ver progresso).
// s_curRaw guarda a MEDIANA de 3 (anti-spike do SAR); s_curM3, as duas
// amostras anteriores de cada canal.
static volatile uint16_t s_curRaw[2]   = { 2047, 2047 };
static volatile uint16_t s_curM3[2][2] = { { 2047, 2047 }, { 2047, 2047 } };
static volatile uint32_t s_curSeq[2]  = { 0, 0 };
// zero vivo: EMA inteira em ponto fixo <<6, alpha = 1/16 por amostra
// (a 8 kHz por canal => tau ~2 ms: acompanha o trilho, o térmico E as
// rajadas do rádio, cortando o ruído de uma amostra por ~5,6x)
static volatile uint32_t s_zeroAcc[2] = { 2047u << 6, 2047u << 6 };
static volatile uint32_t s_zeroSeq[2] = { 0, 0 };
static volatile uint8_t  s_curGap[2]  = { 0, 0 };   // centros sem amostra

// conversão em voo (disparada no evento anterior)
static volatile uint8_t  s_ifKind = KIND_NONE;
static volatile uint8_t  s_ifCh   = 0;
static volatile uint8_t  s_ifOk   = 0;

// rodízio de canal por tipo
static volatile uint8_t  s_curNext  = 0;
static volatile uint8_t  s_zeroNext = 0;

// estatística: latência de disparo (ticks desde o evento) e rejeições
static volatile uint32_t s_latSum = 0, s_latN = 0, s_latMax = 0;
static volatile uint32_t s_rejCur = 0, s_rejZero = 0, s_rejZeroJump = 0;
static volatile uint32_t s_rejBoth = 0, s_rejLate = 0, s_rejNotReady = 0;
// zero da calibração em escala do acumulador (<<6) — p/ os clamps na ISR
static volatile uint32_t s_zeroRef64[2] = { 2047u << 6, 2047u << 6 };

static float             s_dutyMax    = 0.80f;  // conservador até medir
static float             s_dutyOp     = 0.65f;  // teto de OPERAÇÃO (folga real)
// Pré-disparo da corrente via compare B (ver config IPH_PRETRIG_ENABLE):
// armado no init depois de medir a latência; a ISR muda de geometria por ele.
static volatile bool     s_pretrig    = false;
static uint32_t          s_shStartTk  = 0;      // início do S&H após o start
static uint32_t          s_shEndTk    = 0;      // fim do S&H após o start
static uint32_t          s_marginTk   = 0;      // margem p/ as DUAS bordas
static float             s_latAvgUs   = 3.0f;
static volatile uint32_t s_xpdFix     = 0;      // reafirmações FORCE_XPD_SAR
static bool              s_zeroPinned = false;
static bool              s_clipping   = false;
static bool              s_isrLevel3  = false;  // ISR conseguiu o nível 3?

// MEDIANA DE 3 por canal: o SAR do ESP32 solta spikes isolados de dezenas de
// contagens (comportamento conhecido do silício). Um impulso isolado morre
// inteiro na mediana; sinal real (<=200 Hz amostrado a 8-16 kHz) e monotonico
// entre amostras e atravessa intacto.
static inline void IRAM_ATTR pushCur(uint8_t k, uint16_t v) {
  const uint16_t a = s_curM3[k][0], b = s_curM3[k][1];
  s_curM3[k][0] = b;
  s_curM3[k][1] = v;
  s_curRaw[k] = (a > b) ? ((b > v) ? b : ((a > v) ? v : a))
                        : ((a > v) ? a : ((b > v) ? v : b));
  s_curSeq[k] = s_curSeq[k] + 1;
}

// ---------------------------------------------------------------------------
// ISR — colhe a conversão anterior, valida e dispara a próxima. Só inteiros.
// ---------------------------------------------------------------------------
static void IRAM_ATTR mcpwmIsr(void *) {
#if IPH_INLINE && !IPH_PRETRIG_ENABLE
#error "IPH_INLINE exige IPH_PRETRIG_ENABLE: sem o pre-disparo a ISR cai na geometria low-side, que amostra ZERO no TEZ — e com o shunt em serie o TEZ tem CORRENTE. O zero viraria a propria corrente."
#endif

#if IPH_INLINE
  // ===== GEOMETRIA IN-LINE =================================================
  // Dois pré-disparos por período, um antes de cada centro:
  //   compare B do operador 0, em (periodo − pretrig)  -> vale SUBINDO  (TEP)
  //   compare B do operador 1, em (pretrig)            -> vale DESCENDO (TEZ)
  // Cada compare cruza duas vezes por período; a direção errada sai fora
  // logo na entrada, custando poucas centenas de nanossegundos.
  // Não há mais evento de ZERO: com o shunt em série não existe instante de
  // corrente nula garantida (ver IPH_INLINE no config).
  if (s_pretrig) {
    const uint32_t stTep = s_dev->int_st.op0_teb_int_st;
    const uint32_t stTez = s_dev->int_st.op1_teb_int_st;
    if (!(stTep | stTez)) {
      // NÃO É NOSSO EVENTO — mas a linha está asserted, senão a ISR não teria
      // sido chamada. Sair daqui sem limpar deixa a fonte pendente e a ISR
      // RE-ENTRA PARA SEMPRE: foi um "Interrupt wdt timeout on CPU1" na
      // bancada, com o TEZ legado ainda habilitado na janela de armação.
      // Limpa exatamente o que estiver pendente e sai.
      s_dev->int_clr.val = s_dev->int_st.val;
      return;
    }
    if (stTep && stTez) {          // atraso de meio período: recomeça limpo
      s_dev->int_clr.op0_teb_int_clr = 1;
      s_dev->int_clr.op1_teb_int_clr = 1;
      s_ifKind = KIND_NONE;
      s_rejBoth = s_rejBoth + 1;
      return;
    }
    if (stTep) {
      s_dev->int_clr.op0_teb_int_clr = 1;
      if (s_dev->timer[0].timer_status.timer_direction) return;   // descendo
    } else {
      s_dev->int_clr.op1_teb_int_clr = 1;
      if (!s_dev->timer[0].timer_status.timer_direction) return;  // subindo
    }

    // ---- colhe a conversão do evento anterior ----------------------------
    if (s_ifKind != KIND_NONE) {
      uint16_t v;
      if (adcResult(&v)) { if (s_ifOk) pushCur(s_ifCh, v); }
      else               s_rejNotReady = s_rejNotReady + 1;
      s_ifKind = KIND_NONE;
    }

    // ---- valida e dispara a próxima --------------------------------------
    // Uma leitura só de timer_status entrega contagem E direção sem corrida.
    const uint32_t stv = s_dev->timer[0].timer_status.val;
    const bool nowDown = (stv & (1u << 16)) != 0;
    const uint32_t cnt = stv & 0xFFFFu;
    // ttc = ticks ATÉ o centro, COM SINAL. Positivo = o centro ainda vem;
    // NEGATIVO = ele já passou (a colheita demorou).
    //
    // Passar do centro NÃO estraga a amostra: a janela é simétrica em torno
    // dele, então cair 1 µs depois vale o mesmo que cair 1 µs antes. Quem
    // decide é o portão abaixo, que já trabalha com distância com sinal. A
    // versão anterior descartava tudo que passasse do centro e jogava fora
    // 13,7% das amostras (4,1 M de "atrasadas" no diag da bancada) — e jogava
    // fora justamente sob carga, quando a ISR disputa CPU e a medida importa.
    // O contador continua, agora só como diagnóstico.
    const uint32_t dist = stTep ? (s_period - cnt) : cnt;
    const bool passou = stTep ? nowDown : !nowDown;
    if (passou) s_rejLate = s_rejLate + 1;
    const int32_t ttc = passou ? -(int32_t)dist : (int32_t)dist;
    // Mesmo par de condições do low-side, só que a "janela" agora é a metade
    // do período em que o S&H tem de caber: LOW (periodo − compare) para a
    // amostra no TEP, HIGH (compare) para a amostra no TEZ.
    int32_t needClose = (int32_t)(s_shEndTk + s_marginTk) - ttc;
    int32_t needOpen  = ttc + (int32_t)s_marginTk - (int32_t)s_shStartTk;
    int32_t need = (needClose > needOpen) ? needClose : needOpen;
    if (need < 0) need = 0;

    // CONTABILIDADE DE PERDA. Um canal não caber NESTE centro não é perda: no
    // in-line cada canal tem o SEU centro (duty baixo -> TEP, duty alto ->
    // TEZ) e não caber no outro é o funcionamento normal. Contar isso encheu
    // o diag de 16 MILHÕES de "rejeições" com o motor parado — onde todos os
    // duties são 0 e portanto NENHUM canal cabe no TEZ, sempre.
    // E quando os DOIS canais querem o mesmo centro (duty extremo nas duas
    // fases medidas, ex. motor parado) eles têm de dividi-lo, o que já custa
    // um período. Perda de verdade é passar de DOIS períodos sem amostra.
    for (int k = 0; k < 2; k++)
      if (++s_curGap[k] >= 5) { s_rejCur = s_rejCur + 1; s_curGap[k] = 0; }

    const uint8_t k0 = s_curNext;
    for (int t = 0; t < 2; t++) {
      const uint8_t k = (uint8_t)(k0 ^ t);
      const uint8_t p = s_zeroPhase[k];
      uint32_t c;
      if (p <= 2) {
        c = s_dev->operators[p].timestamp[0].gen;
      } else {
        // Sem calibração (mapa desconhecido): pior caso das três fases —
        // o maior compare encolhe a janela LOW, o menor encolhe a HIGH.
        c = s_dev->operators[0].timestamp[0].gen;
        const uint32_t c1 = s_dev->operators[1].timestamp[0].gen;
        const uint32_t c2 = s_dev->operators[2].timestamp[0].gen;
        if (stTep) { if (c1 > c) c = c1; if (c2 > c) c = c2; }
        else       { if (c1 < c) c = c1; if (c2 < c) c = c2; }
      }
      const uint32_t halfWin = stTep ? (s_period - c) : c;
      if ((uint32_t)need < halfWin) {
        adcStartCh(s_ch[k]);
        s_ifKind = KIND_CUR; s_ifCh = k; s_ifOk = 1;
        s_curGap[k] = 0;
        s_curNext = (uint8_t)(k ^ 1);
        return;
      }
    }
    return;   // este centro não serve a nenhum dos dois; o outro serve
  }
#endif  // IPH_INLINE

  const uint32_t stTez = s_dev->int_st.timer0_tez_int_st;
  // Evento de CORRENTE: com o pré-disparo armado é o compare B do operador 0
  // (cruzamento ANTES do TEP — ver config); na geometria legada é o TEP.
  const uint32_t stCur = s_pretrig ? s_dev->int_st.op0_teb_int_st
                                   : s_dev->int_st.timer0_tep_int_st;
  if (!(stTez | stCur)) {                       // não é nosso
    s_dev->int_clr.val = s_dev->int_st.val;     // ver o comentário acima
    return;
  }

  if (stTez && stCur) {
    // Atraso de um evento inteiro (>31 µs): perdemos a referência de tempo da
    // conversão em voo. Descarta tudo e recomeça limpo no próximo evento.
    s_dev->int_clr.timer0_tez_int_clr = 1;
    if (s_pretrig) s_dev->int_clr.op0_teb_int_clr = 1;
    else           s_dev->int_clr.timer0_tep_int_clr = 1;
    s_ifKind = KIND_NONE;
    s_rejBoth = s_rejBoth + 1;
    return;
  }
  const bool evCur = (stCur != 0);

  if (evCur && s_pretrig) {
    // O compare B cruza DUAS vezes por período: SUBINDO (= pré-TEP, o nosso
    // disparo) e DESCENDO (= logo depois do TEP). A descida é limpa e
    // ignorada ANTES do bloco de colheita — a conversão em voo (disparada
    // na subida, ~2×PRETRIG atrás) ainda está convertendo e seria jogada
    // fora como "não-pronta", matando o pipeline.
    s_dev->int_clr.op0_teb_int_clr = 1;
    if (s_dev->timer[0].timer_status.timer_direction) return;   // descendo
  }

  // ---- 1) colher a conversão disparada no evento ANTERIOR -----------------
  if (s_ifKind != KIND_NONE) {
    uint16_t v;
    if (adcResult(&v)) {
      if (s_ifOk) {
        const uint8_t k = s_ifCh;
        if (s_ifKind == KIND_CUR) {
          pushCur(k, v);
        } else {
          // EMA inteira do zero vivo (<<6, alpha 1/16), com DUAS guardas de
          // contaminação (ver config, "GUARDAS DO ZERO VIVO"): offset é
          // grandeza lenta — salto grande numa amostra é corrente vazando
          // pela borda da janela ou spike do SAR, nunca deriva; e o total
          // nunca sai da banda física em torno do zero da calibração.
          const int32_t acc = (int32_t)s_zeroAcc[k];
          const int32_t dev = (int32_t)((uint32_t)v << 6) - acc;
          if (dev > (IPH_ZERO_STEP_MAX_CNT << 6) ||
              dev < -(IPH_ZERO_STEP_MAX_CNT << 6)) {
            s_rejZeroJump = s_rejZeroJump + 1;
          } else {
            int32_t na = acc + (dev >> 4);
            const int32_t lo = (int32_t)s_zeroRef64[k] - (IPH_ZERO_LIVE_BAND_CNT << 6);
            const int32_t hi = (int32_t)s_zeroRef64[k] + (IPH_ZERO_LIVE_BAND_CNT << 6);
            if (na < lo) na = lo; else if (na > hi) na = hi;
            s_zeroAcc[k] = (uint32_t)na;
            s_zeroSeq[k] = s_zeroSeq[k] + 1;
          }
        }
      }
    } else {
      s_rejNotReady = s_rejNotReady + 1;
    }
    s_ifKind = KIND_NONE;
  }

  // ---- 2) validar a janela e disparar a próxima ---------------------------
  if (s_pretrig && evCur) {
    // PRÉ-DISPARO: estamos ~(latência+guarda) ANTES do TEP. Uma leitura
    // única de timer_status entrega contagem E direção sem corrida entre as
    // duas; reler AQUI (a colheita acima custou ~1-2 µs) dá a posição
    // fresca. Se o contador já virou (o TEP passou durante a colheita), o
    // disparo é pulado — buraco de 1 amostra, inofensivo, e o portão nem
    // teria mais como posicionar o S&H.
    const uint32_t stv = s_dev->timer[0].timer_status.val;
    if (stv & (1u << 16)) {
      s_rejLate = s_rejLate + 1;               // virou durante a colheita
    } else {
      const uint32_t ttp = s_period - (stv & 0xFFFFu);   // ticks até o TEP
      const uint8_t  k = s_curNext;
      s_curNext = k ^ 1;
      const uint8_t  p = s_zeroPhase[k];       // fase física do canal k
      uint32_t c;
      if (p <= 2) {
        c = s_dev->operators[p].timestamp[0].gen;
      } else {
        // Sem calibração (mapa desconhecido): conservador — maior das três.
        c = s_dev->operators[0].timestamp[0].gen;
        const uint32_t c1 = s_dev->operators[1].timestamp[0].gen;
        const uint32_t c2 = s_dev->operators[2].timestamp[0].gen;
        if (c1 > c) c = c1;
        if (c2 > c) c = c2;
      }
      const uint32_t halfWin = s_period - c;   // meia-janela low após o TEP
      // Duas bordas, dois requisitos (ticks, relativos a "agora"; o S&H
      // ocupa [agora+setup, agora+setup+S&H] e o TEP está a ttp ticks):
      //   FECHAMENTO: fim do S&H + margem antes de o low FET ABRIR
      //   ABERTURA:   início do S&H ≥ margem depois de o low FET ter FECHADO
      int32_t needClose = (int32_t)(s_shEndTk + s_marginTk) - (int32_t)ttp;
      int32_t needOpen  = (int32_t)(ttp + s_marginTk) - (int32_t)s_shStartTk;
      int32_t need = (needClose > needOpen) ? needClose : needOpen;
      if (need < 0) need = 0;
      if ((uint32_t)need < halfWin) {
        adcStartCh(s_ch[k]);
        s_ifKind = KIND_CUR; s_ifCh = k; s_ifOk = 1;
      } else {
        s_rejCur = s_rejCur + 1;   // teto de operação deveria impedir isto
      }
    }
  } else {
  // (bloco legado intacto — em modo pré-disparo só o TEZ/zero passa aqui)
  // Ticks decorridos desde o evento: após o TEP o contador desce a partir do
  // período; após o TEZ sobe a partir de zero. (Se a latência passasse de
  // meio período, cairíamos no caso "os dois pendentes" acima.)
  const uint32_t cnt = s_dev->timer[0].timer_status.timer_value;
  const uint32_t elapsed = evCur ? (s_period - cnt) : cnt;
  if (elapsed > (s_period >> 1)) {              // leitura sem sentido: pula
    s_rejLate = s_rejLate + 1;
  } else {
    s_latSum = s_latSum + elapsed;
    s_latN   = s_latN + 1;
    if (elapsed > s_latMax) s_latMax = elapsed;

    const uint32_t need = elapsed + s_gateTicks;
    const bool curEvent = (evCur == (s_evCur == 0));

    if (curEvent) {
      // CORRENTE: a janela de condução low-side DA FASE DO PRÓPRIO CANAL após
      // este evento dura (período − compare) ticks — compare lido do
      // registrador, nada de valor de software possivelmente velho. A versão
      // anterior usava o MAIOR compare das três fases ("se a mais alta cabe,
      // todas cabem") — inofensivo no mundo centrado, FATAL no fundo-plano:
      // no teto de modulação a fase de PICO (muitas vezes a fase A, que nem
      // é medida) vetava as amostras dos DOIS canais durante o arco inteiro
      // dela, 1-3 ms por ciclo elétrico → buracos simultâneos nos dois
      // canais → mistura de fase velha+nova no dq → o caos do topo que
      // "seguia o teto" onde quer que ele fosse posto (caixas-pretas
      // 0x01/0x08). Para medir o shunt de B só importa a janela de B: o teto
      // de operação garante que ELA engole latência+S&H+folga no pior duty.
      const uint8_t k = s_curNext;
      s_curNext = k ^ 1;
      const uint8_t p = s_zeroPhase[k];       // fase física do canal k
      uint32_t c;
      if (p <= 2) {
        c = s_dev->operators[p].timestamp[0].gen;
      } else {
        // Sem calibração (mapa desconhecido): conservador como antes.
        c = s_dev->operators[0].timestamp[0].gen;
        const uint32_t c1 = s_dev->operators[1].timestamp[0].gen;
        const uint32_t c2 = s_dev->operators[2].timestamp[0].gen;
        if (c1 > c) c = c1;
        if (c2 > c) c = c2;
      }
      if (need < s_period - c) {
        adcStartCh(s_ch[k]);
        s_ifKind = KIND_CUR; s_ifCh = k; s_ifOk = 1;
      } else {
        s_rejCur = s_rejCur + 1;    // teto de operação deveria impedir isto
      }
    } else {
      // ZERO VIVO: precisa da janela high-side DA FASE DAQUELE CANAL (o low
      // FET dela desligado). Só com calibração (mapa conhecido). Se a fase do
      // canal da vez está com duty baixo (sem janela), tenta o outro canal —
      // com modulação não-centrada cada fase passa 1/3 do ciclo elétrico em
      // duty 0, e é o rodízio flexível que mantém os DOIS zeros frescos.
      uint8_t k = s_zeroNext;
      s_zeroNext = k ^ 1;
      uint32_t anyWin = 0;                      // alguma fase TINHA janela?
      for (int t = 0; t < 2; t++, k ^= 1) {
        const uint8_t p = s_zeroPhase[k];
        if (p > 2) break;                       // sem calibração: sem zero vivo
        const uint32_t c = s_dev->operators[p].timestamp[0].gen;
        if (c > (s_gateTicks >> 1)) anyWin = 1;
        if (need < c) {
          adcStartCh(s_ch[k]);
          s_ifKind = KIND_ZERO; s_ifCh = k; s_ifOk = 1;
          break;
        }
        // Duty ~0 (parado/desabilitado) NÃO é rejeição: não existe janela
        // por projeto — quem cobre esse estado é o rastreio de repouso. Só
        // conta quando uma janela existia e a latência não coube nela.
        if (t == 1 && anyWin) s_rejZero = s_rejZero + 1;
      }
    }
  }
  }   // fim do ramo legado (pré-disparo tem o seu próprio acima)

  // Em modo pré-disparo o TEB já foi limpo lá em cima (antes da colheita).
  if (evCur) { if (!s_pretrig) s_dev->int_clr.timer0_tep_int_clr = 1; }
  else       s_dev->int_clr.timer0_tez_int_clr = 1;
}

// ---------------------------------------------------------------------------
// Zero vivo — lado de leitura
// ---------------------------------------------------------------------------
// fração preservada (dividir, não deslocar): truncar aqui deixava até 1
// contagem de viés no zero em uso (~0,06 A) — apareceu como resíduo DC no diag
static inline float zeroCnt(int k) { return (float)s_zeroAcc[k] * (1.0f / 64.0f); }
static inline float zeroV(int k)   { return zeroCnt(k) * ADC_V_PER_COUNT; }

// Zero EM USO: o vivo, contido no cerco físico em torno do zero da
// calibração. Fora do cerco não é deriva (o cerco é largo), é canal
// morto/curto — o valor prende no limite e phaseCurrentZeroPinned() avisa.
// O flag é recalculado a CADA leitura (pinned é estado, não trava).
static float zeroUseV(int k, bool *pinned) {
  const float z = zeroV(k);
  const float lo = s_cal.zeroRef[k] - IPH_ZERO_MAX_DRIFT_V;
  const float hi = s_cal.zeroRef[k] + IPH_ZERO_MAX_DRIFT_V;
  if (z < lo) { *pinned = true; return lo; }
  if (z > hi) { *pinned = true; return hi; }
  return z;
}

// Ampères por volt de saída do INA240, POR CANAL (a medida da calibração;
// o nominal do config só enquanto não houver calibração).
static inline float ampsPerVolt(int k) {
  if (s_cal.valid && s_cal.ampsPerVolt[k] > 0.01f) return s_cal.ampsPerVolt[k];
  return prm(P_PH_GAIN) / ((prm(P_PH_SHUNT) * 0.001f) * PHASE_CUR_AMP_GAIN);
}

// Correntes mapeadas nas fases A/B/C (a que falta sai de ia+ib+ic = 0).
// Atualiza também a detecção de saturação (saída do INA240 no trilho).
static void phaseAmps(float ph[3]) {
  bool got[3] = { false, false, false };
  bool clip = false, pinned = false;
  for (int k = 0; k < 2; k++) {
    const uint16_t cnt = s_curRaw[k];
    if (cnt < IPH_CLIP_LO_CNT || cnt > IPH_CLIP_HI_CNT) clip = true;
    const float v = cnt * ADC_V_PER_COUNT - zeroUseV(k, &pinned);
    const int p = s_cal.valid ? s_cal.ph[k] : k;
    ph[p]  = v * ampsPerVolt(k) * (s_cal.valid ? s_cal.sign[k] : 1);
    got[p] = true;
  }
  s_clipping   = clip;
  s_zeroPinned = pinned;
  for (int p = 0; p < 3; p++)
    if (!got[p]) ph[p] = -ph[(p + 1) % 3] - ph[(p + 2) % 3];
}

// ---------------------------------------------------------------------------
// O sensor visto pelo SimpleFOC (initFOC exige um CurrentSense inicializado)
// ---------------------------------------------------------------------------
class PhaseCS : public CurrentSense {
 public:
  int init() override { return s_ready ? 1 : 0; }
  int driverAlign(float) override { return 1; }   // alinhamento = calibração
  PhaseCurrent_s getPhaseCurrents() override {
    PhaseCurrent_s c = { 0, 0, 0 };
    if (!s_cal.valid) return c;
    float ph[3];
    phaseAmps(ph);
    c.a = ph[0]; c.b = ph[1]; c.c = ph[2];
    return c;
  }
};
static PhaseCS s_cs;

// ---------------------------------------------------------------------------
// Leitura convertida (núcleo 1) — Clarke/Park + filtro dq
// ---------------------------------------------------------------------------
static float    s_idFilt = 0.0f, s_iqFilt = 0.0f;
static uint32_t s_seqSnap = 0, s_seqSnapUs = 0;   // frescor da realimentação
static bool     s_fresh = true;

void phaseCurrentRead(PhaseCur *out, float angleEl) {
  if (!out) return;
  float ph[3];
  phaseAmps(ph);
  out->ia = ph[0]; out->ib = ph[1]; out->ic = ph[2];

  // Clarke (forma amplitude-invariante, válida porque ia+ib+ic = 0)
  out->ialpha = ph[0];
  out->ibeta  = _1_SQRT3 * (ph[1] - ph[2]);
  out->imag   = sqrtf(out->ialpha * out->ialpha + out->ibeta * out->ibeta);

  // Park — e o filtro AQUI, onde o sinal é DC em regime. Filtrar as fases
  // (AC) atenuaria e defasaria o Iq com o motor andando: foi o erro central
  // da implementação anterior.
  const float ct = _cos(angleEl), st = _sin(angleEl);
  const float idRaw =  out->ialpha * ct + out->ibeta * st;
  const float iqRaw =  out->ibeta  * ct - out->ialpha * st;

  static uint32_t lastUs = 0;
  const uint32_t now = micros();
  float dt = (lastUs == 0) ? 0.001f : (now - lastUs) * 1e-6f;
  lastUs = now;
  dt = constrain(dt, 0.0f, 0.02f);
  const float tf = fmaxf(prm(P_CUR_TF), 0.0002f);
  const float a  = fminf(dt / tf, 1.0f);
  s_idFilt += a * (idRaw - s_idFilt);
  s_iqFilt += a * (iqRaw - s_iqFilt);
  out->id = s_idFilt;
  out->iq = s_iqFilt;

  // zeros vivos p/ telemetria (o app mostra ZERO CH1/CH2)
  s_cal.offset[0] = zeroV(0);
  s_cal.offset[1] = zeroV(1);

  // FRESCOR da realimentação: se a sequência de amostras parou de andar, a
  // malha está CEGA (janela fechada, ISR morta, ADC preso) — quem corta é o
  // motor_control via phaseCurrentFresh(); aqui só se mede.
  const uint32_t ss = s_curSeq[0] + s_curSeq[1];
  if (ss != s_seqSnap) { s_seqSnap = ss; s_seqSnapUs = now; }
  s_fresh = (uint32_t)(now - s_seqSnapUs) < 3000;
}

bool phaseCurrentFresh() { return s_fresh; }

bool phaseCurrentZeroPinned() { return s_zeroPinned; }
bool phaseCurrentClipping()   { return s_clipping; }

// ---------------------------------------------------------------------------
// Rastreio do zero em REPOUSO (complementa o zero vivo, que precisa de duty)
// ---------------------------------------------------------------------------
// Parado com o motor desabilitado os duties são 0: os low-side conduzem o
// tempo todo, não existe janela high-side e o zero vivo não atualiza — mas as
// amostras de CORRENTE, com corrente comprovadamente nula (quem garante é o
// chamador: motor desabilitado + 300 ms sem borda de Hall), SÃO o zero.
// Escreve no MESMO acumulador da ISR; não há corrida porque a ISR só escreve
// nele quando há janela high-side (duty > 0), e este caminho só roda com
// duty 0 — os dois estados são mutuamente exclusivos por construção.
#if IPH_INLINE
// ---------------------------------------------------------------------------
// Rastreio do zero COM O MOTOR GIRANDO — o substituto do zero no TEZ
// ---------------------------------------------------------------------------
// O zero vivo do low-side vinha de uma garantia GEOMÉTRICA: no TEZ o MOSFET de
// baixo está aberto, então a corrente pelo shunt é zero por construção e o que
// o INA240 mostra ali é exatamente o offset. Com o shunt EM SÉRIE COM A FASE
// esse instante não existe — a corrente atravessa o shunt o tempo inteiro.
//
// O que substitui a garantia geométrica é uma propriedade do motor: a corrente
// de fase NÃO TEM COMPONENTE CONTÍNUA. Numa volta elétrica ela passa tanto
// tempo positiva quanto negativa, com qualquer torque, em qualquer velocidade.
// Então a média longa da leitura CRUA de cada canal converge para o offset
// daquele canal — e só para ele.
//
// Duas condições, as duas obrigatórias:
//   - a média tem de cobrir MUITAS voltas elétricas, senão sobra resíduo da
//     própria senoide dentro do "zero";
//   - o motor tem de estar GIRANDO. PARADO com torque a corrente de fase é
//     contínua de verdade, e aprender essa média como zero seria aprender a
//     CORRENTE — o erro mais perigoso possível aqui, porque volta depois como
//     corrente fantasma permanente. Quem cobre o repouso é a função abaixo.
// O cerco físico em torno do zero da calibração continua valendo por cima:
// mesmo enganada, a média não sai da banda.
void phaseCurrentTrackZeroSpin(float dt, float fElecHz) {
  if (!s_ready || !s_cal.zeroed) return;
  if (!(fElecHz >= IPH_ZERO_SPIN_MIN_HZ)) return;      // NaN cai aqui também
  // τ efetivo: nunca menos que 16 voltas elétricas, para o resíduo da
  // fundamental na média ficar bem abaixo de uma contagem de ADC.
  const float tau = fmaxf(IPH_ZERO_SPIN_TAU_S, 16.0f / fElecHz);
  const float a   = fminf(fminf(dt, 0.02f) / tau, 1.0f);
  for (int k = 0; k < 2; k++) {
    const float accf = (float)s_zeroAcc[k];             // escala <<6
    const float tgt  = (float)s_curRaw[k] * 64.0f;
    float na = accf + a * (tgt - accf);
    const float lo = (float)s_zeroRef64[k] - (float)(IPH_ZERO_LIVE_BAND_CNT << 6);
    const float hi = (float)s_zeroRef64[k] + (float)(IPH_ZERO_LIVE_BAND_CNT << 6);
    if (na < lo) na = lo; else if (na > hi) na = hi;
    s_zeroAcc[k] = (uint32_t)(na + 0.5f);
    s_zeroSeq[k] = s_zeroSeq[k] + 1;      // o 'diag' conta por aqui agora
  }
}
#endif

void phaseCurrentTrackZero(float dt, bool snap) {
  if (!s_ready || !s_cal.zeroed) return;
  dt = fminf(dt, 0.010f);
  for (int k = 0; k < 2; k++) {
    const float meas = (float)s_curRaw[k];
    // RAIO CURTO DE CONFIANÇA (não o cerco largo): "300 ms sem borda de
    // Hall" NÃO prova corrente nula — o rotor balança DENTRO de um setor
    // depois de uma calibração/teste, sem gerar borda, empurrando corrente
    // real pelos shunts em curto. A bancada mostrou o rastreio aprendendo
    // esse balanço (−42 contagens = 5 A fantasmas) e congelando. Deriva
    // legítima de repouso é de poucos mV: só ela passa por aqui. O que for
    // maior é corrente de verdade — e corrente não se aprende como zero.
    if (fabsf(meas * ADC_V_PER_COUNT - s_cal.zeroRef[k]) > IPH_ZERO_REST_BAND_V)
      continue;
    // snap NÃO copia a amostra crua (uma amostra tem ±3 contagens de ruído,
    // que virariam offset): o rastreio já rodou os 300 ms de quietude que o
    // chamador exige, então o acumulador está convergido — o snap só dá um
    // puxão final mais forte, ainda filtrado.
    // A conta preserva a FRAÇÃO (arredonda, não trunca): reaplicar floor a
    // cada passo descartava os passos pequenos de subida e deixava um
    // ratchet que só andava para baixo.
    const float accf = (float)s_zeroAcc[k];              // escala <<6
    const float tgt  = meas * 64.0f;
    const float a    = snap ? 0.25f : fminf(dt / IPH_ZERO_REST_TAU_S, 1.0f);
    s_zeroAcc[k] = (uint32_t)(accf + a * (tgt - accf) + 0.5f);
  }
}

// ---------------------------------------------------------------------------
// NVS (versão 2 — a semântica mudou com a reescrita; cal antiga é recusada)
// ---------------------------------------------------------------------------
static uint8_t classifyRef(float z, bool measured) {
  if (!measured) return IPH_REF_NONE;
  const float vcc = PHASE_CUR_VCC_V;
  if (z > 0.30f * vcc && z < 0.70f * vcc) return IPH_REF_BIDIR;
  if (z < 0.15f * vcc)                    return IPH_REF_LOW;
  if (z > 0.85f * vcc)                    return IPH_REF_HIGH;
  return IPH_REF_NONE;
}
static void refresh() {
  for (int k = 0; k < 2; k++) {
    s_cal.ref[k] = classifyRef(s_cal.zeroRef[k], s_cal.zeroed);
    // a ISR só liga o zero vivo com o mapa conhecido; a âncora dos clamps
    // (banda do zero vivo) acompanha o zero da calibração
    s_zeroPhase[k]  = s_cal.valid ? (uint8_t)s_cal.ph[k] : 255;
    s_zeroRef64[k]  = (uint32_t)(s_cal.zeroRef[k] / ADC_V_PER_COUNT * 64.0f);
  }
}

// VERSÃO da calibração na NVS. Ela é AMARRADA À TOPOLOGIA do shunt, e isso é
// segurança, não organização: uma calibração low-side aplicada a shunt em
// série traz a ESCALA errada (o shunt mudou de valor) e, pior, pode trazer o
// SINAL errado — no low-side mede-se a corrente de RETORNO pelo FET de baixo,
// em série mede-se a corrente da fase, e as duas podem ter sentidos opostos.
// Sinal invertido na realimentação de corrente é realimentação POSITIVA: a
// malha empurra para o lado do erro e dispara. Trocar a topologia portanto
// invalida a calibração e o firmware bloqueia o motor até 'cal' rodar de novo.
#define IPH_CAL_NVS_VER   (IPH_INLINE ? 3 : 2)

static void calLoad() {
  s_nvs.begin("iphase", true);
  const uint8_t ver = s_nvs.getUChar("ver", 0);
  if (ver != IPH_CAL_NVS_VER) {
    const bool hadV1 = s_nvs.isKey("ok");
    s_nvs.end();
    s_cal = PhaseCal{};
    for (int k = 0; k < 2; k++) {
      s_cal.ph[k] = k; s_cal.sign[k] = 1;
      s_cal.offset[k] = 1.65f; s_cal.zeroRef[k] = 1.65f;
    }
    if (hadV1)
      Serial.printf("[iphase] calibracao da NVS DESCARTADA (versao %u, "
                    "esperada %u): topologia de shunt ou sensoriamento "
                    "mudaram. O motor fica BLOQUEADO ate rodar 'cal' com a "
                    "roda suspensa — escala e SINAL precisam ser medidos de "
                    "novo.\n", (unsigned)ver, (unsigned)IPH_CAL_NVS_VER);
    refresh();
    return;
  }
  s_cal.valid  = s_nvs.getBool("ok", false);
  s_cal.event  = s_nvs.getUChar("ev", 0) ? 1 : 0;
  s_cal.zeroed = s_nvs.getBool("zr", false);
  for (int k = 0; k < 2; k++) {
    char key[8];
    snprintf(key, sizeof(key), "ph%d", k); s_cal.ph[k]      = s_nvs.getChar(key, k);
    snprintf(key, sizeof(key), "sg%d", k); s_cal.sign[k]    = s_nvs.getChar(key, 1);
    snprintf(key, sizeof(key), "zr%d", k); s_cal.zeroRef[k] = s_nvs.getFloat(key, 1.65f);
    snprintf(key, sizeof(key), "ap%d", k); s_cal.ampsPerVolt[k] = s_nvs.getFloat(key, 0.0f);
    s_cal.offset[k] = s_cal.zeroRef[k];
    // NVS corrompida não pode virar índice fora de faixa nem sinal zero
    if (s_cal.ph[k] < 0 || s_cal.ph[k] > 2 || (s_cal.sign[k] != 1 && s_cal.sign[k] != -1))
      s_cal.valid = false;
  }
  s_nvs.end();
#if IPH_INLINE
  // IN-LINE não tem evento "certo" e evento "errado": a corrente atravessa o
  // shunt nos DOIS centros, e a ISR usa os dois de propósito. Um s_cal.event
  // herdado de uma calibração low-side antiga (valendo 1 = TEZ) deixaria o
  // pré-disparo sem armar e a geometria cairia no caminho legado — que
  // amostraria "zero" onde agora passa corrente. Fixado em 0.
  s_evCur = 0;
#else
  s_evCur = s_cal.event;
#endif
  refresh();
}

static void calSave() {
  s_nvs.begin("iphase", false);
  s_nvs.putUChar("ver", IPH_CAL_NVS_VER);
  s_nvs.putBool("ok", s_cal.valid);
  s_nvs.putUChar("ev", s_cal.event);
  s_nvs.putBool("zr", s_cal.zeroed);
  for (int k = 0; k < 2; k++) {
    char key[8];
    snprintf(key, sizeof(key), "ph%d", k); s_nvs.putChar(key, s_cal.ph[k]);
    snprintf(key, sizeof(key), "sg%d", k); s_nvs.putChar(key, s_cal.sign[k]);
    snprintf(key, sizeof(key), "zr%d", k); s_nvs.putFloat(key, s_cal.zeroRef[k]);
    snprintf(key, sizeof(key), "ap%d", k); s_nvs.putFloat(key, s_cal.ampsPerVolt[k]);
  }
  s_nvs.end();
  refresh();
}

bool  phaseCurrentCalValid() { return s_cal.valid; }
const PhaseCal *phaseCurrentCal() { return &s_cal; }
float phaseCurrentDutyOp()  { return s_dutyOp; }

// Instantâneo CRU para a caixa-preta (1 kHz, fora da ISR): por canal,
//   dev  = mediana atual − zero da CAL      [LSB]  (o que o ADC está vendo,
//          incluindo qualquer passeio de zero — referência FIXA de a bordo)
//   zoff = zero vivo em uso − zero da CAL   [LSB]  (o passeio do zero)
//   acc  = amostras de corrente ACEITAS desde a última chamada (satura 255;
//          cadência normal = 8/ms por canal a 16 kHz alternado)
// Com os três dá para separar, no registro da falha: buraco de amostragem
// (acc→0, dev congela), passeio de zero (zoff cresce, dev acompanha),
// corrupção de conversão (dev maluco com acc normal) e lixo de transformada
// (dev são e dq maluco).
void phaseCurrentRawDbg(int16_t dev[2], int8_t zoff[2], uint8_t acc[2]) {
  static uint32_t prev[2] = { 0, 0 };
  for (int k = 0; k < 2; k++) {
    bool pin;
    const float zRefCnt = s_cal.zeroRef[k] / ADC_V_PER_COUNT;
    const float zUseCnt = zeroUseV(k, &pin) / ADC_V_PER_COUNT;
    int32_t d = (int32_t)s_curRaw[k] - (int32_t)lroundf(zRefCnt);
    dev[k] = (int16_t)((d > 32000) ? 32000 : (d < -32000) ? -32000 : d);
    int32_t z = lroundf(zUseCnt - zRefCnt);
    zoff[k] = (int8_t)((z > 127) ? 127 : (z < -127) ? -127 : z);
    const uint32_t s  = s_curSeq[k];
    const uint32_t da = s - prev[k];
    prev[k] = s;
    acc[k] = (uint8_t)((da > 255) ? 255 : da);
  }
}

// Reafirma a trava de power do SAR (FORCE_XPD_SAR=3, errata GPIO-3.11). O
// caminho de leitura do ADC2 (task de sensores: VBAT/NTCs via
// analogReadMilliVolts) devolve o campo ao modo FSM e desfaz o
// adc_power_acquire do boot — o diag pegou FORCE_XPD_SAR=0 em operação.
// Chamada logo DEPOIS de cada rajada de leitura do ADC2 (o ponto que
// desfaz) e como retaguarda na fatia do motor. O contador sai no diag e
// mede quem está desfazendo a trava e com que frequência.
void phaseCurrentSarPowerAssert() {
  if (GET_PERI_REG_BITS2(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR,
                         SENS_FORCE_XPD_SAR_S) != 3) {
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR, 3,
                      SENS_FORCE_XPD_SAR_S);
    s_xpdFix++;
  }
}

// ---------------------------------------------------------------------------
// Inicialização
// ---------------------------------------------------------------------------
CurrentSense *phaseCurrentInit(BLDCDriver *drv, BLDCMotor *mot) {
  if (!drv || !mot) return nullptr;
  s_driver = drv; s_motor = mot;

  ESP32MCPWMDriverParams *dp = (ESP32MCPWMDriverParams *)drv->params;
  if (!dp || !dp->mcpwm_dev) {
    Serial.println("[iphase] ERRO: driver nao e MCPWM — sem sincronismo possivel");
    return nullptr;
  }
  s_dev  = dp->mcpwm_dev;
  s_unit = dp->mcpwm_unit;

  pinMode(PIN_IPH_1, INPUT);
  pinMode(PIN_IPH_2, INPUT);
  adcRead(PIN_IPH_1);          // força a configuração do SAR fora da ISR
  adcRead(PIN_IPH_2);

  // ERRATA ESP32 GPIO-3.11 + conversões corrompidas em voo: a chave de
  // alimentação do SAR alternando (modo FSM) puxa GPIO36/39 para baixo por
  // ~80 ns a cada ligada E corrompe conversões do SAR1 que estejam no meio
  // quando a task de sensores (ADC2) alterna a chave. A mitigação é travar
  // o ADC permanentemente LIGADO (~1 mA) — e TEM de vir DEPOIS do adcRead
  // acima: a configuração da SimpleFOC escreve "use XPD_FSM" nesse mesmo
  // registrador e desfazia a trava (a bancada mediu o resultado: 6-14% das
  // amostras com caudas de ±50 cnt e picos de -213 no GPIO39).
  adc_power_acquire();
  // adc_power_acquire() SOZINHO nem sempre deixa FORCE_XPD_SAR em 3 (o boot
  // mediu 0 logo depois de chamá-lo). Quem garante é a escrita direta — a
  // mesma que phaseCurrentSarPowerAssert() faz depois de cada rajada do ADC2.
  // Sem ela a amostragem começa com a errata GPIO-3.11 ativa até a primeira
  // reafirmação, e é justamente aí que a calibração costuma rodar.
  phaseCurrentSarPowerAssert();
  Serial.printf("[iphase] SAR power: FORCE_XPD_SAR=%lu (3 = travado LIGADO; "
                "0 = FSM alternando = errata ativa)\n",
                (unsigned long)GET_PERI_REG_BITS2(SENS_SAR_MEAS_WAIT2_REG,
                                                  SENS_FORCE_XPD_SAR,
                                                  SENS_FORCE_XPD_SAR_S));
  s_ch[0] = digitalPinToAnalogChannel(PIN_IPH_1);
  s_ch[1] = digitalPinToAnalogChannel(PIN_IPH_2);
  if (s_ch[0] < 0 || s_ch[0] > 7 || s_ch[1] < 0 || s_ch[1] > 7) {
    Serial.println("[iphase] ERRO: PIN_IPH_* precisa estar no ADC1 (GPIO 32-39)");
    return nullptr;
  }

  // Geometria REAL do timer (o driver da SimpleFOC ajusta prescaler/período
  // conforme a frequência — ler do registrador em vez de recalcular)
  s_period     = s_dev->timer[0].timer_cfg0.timer_period;
  s_ticksPerUs = 160.0f / (float)(s_dev->timer[0].timer_cfg0.timer_prescale + 1);
  const float tHalfUs = s_period / s_ticksPerUs;       // meio período [µs]
  // Margem do portão = a MESMA do teto de duty (S&H + margem de deadtime/
  // propagação do FD2103S). A versão com +1 µs deixava o S&H cair na borda
  // da condução na faixa de duty 15-25% — corrente de carga entrava no zero.
  // (A guarda de borda de +2 µs testada aqui foi REMOVIDA: alargava os
  // buracos de amostragem no topo sem ganho — quem protege o topo agora é o
  // teto de OPERAÇÃO, ver IPH_OP_FOLGA_US.)
  s_gateTicks  = (uint32_t)((IPH_SAMPLE_US + PHASE_ISR_MARGIN_US) * s_ticksPerUs);

  calLoad();
  // seed do zero vivo com o zero gravado (evita partir de meio de escala)
  for (int k = 0; k < 2; k++)
    s_zeroAcc[k] = (uint32_t)(s_cal.zeroRef[k] / ADC_V_PER_COUNT * 64.0f);

  // ISR em NÍVEL 3: as ISRs de Hall (nível padrão) fazem esperas de
  // confirmação de até ~24 µs — se pudessem atrasar este handler, o S&H
  // perderia a janela. Preemptando-as, o instante de amostragem é imune.
  esp_err_t e = mcpwm_isr_register(s_unit, mcpwmIsr, nullptr,
                                   ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3,
                                   nullptr);
  s_isrLevel3 = (e == ESP_OK);
  if (e != ESP_OK) {
    Serial.printf("[iphase] nivel 3 indisponivel (%d) — registrando no nivel "
                  "padrao (latencia um pouco maior; a validacao por janela "
                  "descarta o que atrasar)\n", (int)e);
    e = mcpwm_isr_register(s_unit, mcpwmIsr, nullptr, ESP_INTR_FLAG_IRAM, nullptr);
  }
  if (e != ESP_OK) {
    Serial.printf("[iphase] ERRO: mcpwm_isr_register falhou (%d)\n", (int)e);
    return nullptr;
  }
  s_dev->int_ena.timer0_tez_int_ena = 1;
  s_dev->int_ena.timer0_tep_int_ena = 1;

  // ---- mede a LATÊNCIA real de disparo e deriva o teto de modulação -------
  // duty_max = 1 − 2·(latência + S&H + margem)/T_pwm. O fator 2 é a metade
  // da janela que existe DEPOIS do evento — o S&H nunca cai antes dele.
  s_latSum = 0; s_latN = 0; s_latMax = 0;
  delay(200);
  const uint32_t lN = s_latN, lS = s_latSum, lM = s_latMax;
  s_latAvgUs = lN ? (lS / (float)lN) / s_ticksPerUs : 3.0f;
  const float latMaxUs = lM / s_ticksPerUs;
  // +1 µs de folga do TETO sobre o PORTÃO: o teto é o planejador e o portão é
  // a lei — se os dois coincidirem exatamente, no duty máximo metade das
  // amostras cai na fronteira e é rejeitada. Com a folga, a operação normal
  // nunca vive na borda do portão; só o jitter raro é descartado.
  const float needUs   = s_latAvgUs + IPH_SAMPLE_US + PHASE_ISR_MARGIN_US + 1.0f;
  s_dutyMax = constrain(1.0f - needUs / tHalfUs, 0.50f, 0.95f);
  // Teto de OPERAÇÃO: o duty_max acima é o limite FÍSICO da medição — operar
  // NELE deixa a amostra típica na fronteira do portão e cada fase cega no
  // arco do próprio pico (mistura de fase velha+nova no dq = o caos do topo;
  // ver IPH_OP_FOLGA_US no config). O teto que a malha usa (vModCeil) parte
  // DESTE valor: meia-janela no pior duty = portão + latência média + folga.
  s_dutyOp  = constrain(1.0f - (s_latAvgUs + IPH_SAMPLE_US + PHASE_ISR_MARGIN_US
                                + IPH_OP_FOLGA_US) / tHalfUs, 0.40f, s_dutyMax);

#if IPH_PRETRIG_ENABLE
  // ---- PRÉ-DISPARO: arma o compare B e refaz os tetos (ver config) --------
  // Só com o mapa normal (corrente no TEP); mapa invertido fica na geometria
  // legada calculada acima — fallback automático, sem caminho morto.
  if (s_evCur == 0) {
    const uint32_t pretrig = (uint32_t)((s_latAvgUs + IPH_PRETRIG_GUARD_US)
                                        * s_ticksPerUs);
    s_shStartTk = (uint32_t)(IPH_SAR_SETUP_US * s_ticksPerUs);
    s_shEndTk   = s_shStartTk + (uint32_t)(IPH_SAMPLE_US * s_ticksPerUs);
    s_marginTk  = (uint32_t)(PHASE_ISR_MARGIN_US * s_ticksPerUs);
    // Compare B do operador 0: o gerador B não tem pino nem ação — o valor
    // só serve ao nosso interrupt. Escrita imediata (upmethod 0) e estática.
    s_dev->operators[0].gen_stmp_cfg.gen_b_upmethod = 0;
    s_dev->operators[0].timestamp[1].gen = s_period - pretrig;
    // ORDEM IMPORTA. As fontes antigas saem de cena ANTES de s_pretrig virar
    // true: entre uma coisa e outra a ISR já usa a geometria nova, e um TEZ
    // (16 kHz!) que chegue nessa fresta cai num caminho que não sabe limpá-lo.
    s_dev->int_ena.timer0_tep_int_ena = 0;      // TEP sai de cena
    s_dev->int_clr.timer0_tep_int_clr = 1;
#if IPH_INLINE
    s_dev->int_ena.timer0_tez_int_ena = 0;      // TEZ/zero sai de cena
    s_dev->int_clr.timer0_tez_int_clr = 1;
#endif
    s_dev->int_clr.op0_teb_int_clr    = 1;
    s_pretrig = true;                            // a ISR muda de geometria
    s_dev->int_ena.op0_teb_int_ena    = 1;
#if IPH_INLINE
    // SEGUNDO pré-disparo, antes do TEZ: compare B do operador 1 (gerador
    // sem pino nem ação, igual ao do operador 0 — o valor só serve ao nosso
    // interrupt). Com os dois centros disponíveis, a amostra sempre tem para
    // onde ir: o TEZ deixa de ser o evento do zero e vira o segundo instante
    // de CORRENTE.
    s_dev->operators[1].gen_stmp_cfg.gen_b_upmethod = 0;
    s_dev->operators[1].timestamp[1].gen = pretrig;
    s_dev->int_clr.op1_teb_int_clr    = 1;
    s_dev->int_ena.op1_teb_int_ena    = 1;
#endif
    // Tetos da geometria nova: com o S&H centrado no TEP, o requisito de
    // meia-janela nominal é max(fechamento, abertura) — o mesmo par de
    // condições que o portão checa por amostra na ISR.
    const float ttp0   = IPH_PRETRIG_GUARD_US;   // ticks-até-TEP nominal
    const float closeN = IPH_SAR_SETUP_US + IPH_SAMPLE_US
                       + PHASE_ISR_MARGIN_US - ttp0;
    const float openN  = ttp0 + PHASE_ISR_MARGIN_US - IPH_SAR_SETUP_US;
    const float needN  = fmaxf(closeN, openN);
#if IPH_INLINE
    // IN-LINE: o requisito de meia-janela vale para O CENTRO ESCOLHIDO, e a
    // ISR escolhe o que tiver espaço. Como as duas metades somam o período,
    // a maior nunca é menor que T/4 — e T/4 (15,6 µs a 16 kHz) é folgado
    // demais para o requisito de ~5 µs. Ou seja: a AMOSTRAGEM não impõe mais
    // teto nenhum de duty. Quem passa a mandar é o bootstrap (P_MODCEIL).
    // O 0,98 aqui é só para não prometer 100% de duty ao resto do código.
    const float dutyLim = (needN * 2.0f < tHalfUs) ? 0.98f
                        : constrain(1.0f - (needN + 1.0f) / tHalfUs, 0.50f, 0.98f);
    s_dutyMax = dutyLim;
    s_dutyOp  = dutyLim;
#else
    s_dutyMax = constrain(1.0f - (needN + 1.0f) / tHalfUs, 0.50f, 0.95f);
    s_dutyOp  = constrain(1.0f - (needN + IPH_OP_FOLGA_US) / tHalfUs,
                          0.40f, s_dutyMax);
#endif
#if IPH_INLINE
    Serial.printf("[iphase] IN-LINE: pre-disparo DUPLO %.2f us antes de CADA "
                  "centro (TEP e TEZ) | requisito de meia-janela %.1f us "
                  "contra T/4 = %.1f us -> amostragem NAO limita mais o duty "
                  "(quem manda e o bootstrap, P_MODCEIL)\n",
                  pretrig / s_ticksPerUs, needN, tHalfUs * 0.5f);
    Serial.println("[iphase] IN-LINE: zero vivo do TEZ DESLIGADO — o zero "
                   "passa a sair da media longa girando (>=8 Hz eletricos) e "
                   "do repouso com a roda parada.");
#endif
    Serial.printf("[iphase] PRE-DISPARO armado: compare B %.2f us antes do "
                  "TEP | S&H nominal %+.1f..%+.1f us do TEP | requisito de "
                  "meia-janela %.1f us (era %.1f)\n",
                  pretrig / s_ticksPerUs,
                  IPH_SAR_SETUP_US - ttp0, IPH_SAR_SETUP_US + IPH_SAMPLE_US - ttp0,
                  needN, s_latAvgUs + IPH_SAMPLE_US + PHASE_ISR_MARGIN_US);
  } else {
    Serial.println("[iphase] PRE-DISPARO nao armado: calibracao detectou "
                   "corrente no TEZ (mapa invertido) — geometria legada.");
  }
#endif

  s_ready = true;
  s_cs.initialized = true;   // initFOC checa e aborta sem isto

  Serial.printf("[iphase] PWM %.0f Hz | periodo %lu ticks (%.0f MHz) | latencia "
                "media %.2f us (max %.2f) | S&H+margem %.1f us | duty max %.3f "
                "| duty OPERACAO %.3f (teto de modulacao %.3f)\n",
                prm(P_PWM_FREQ), (unsigned long)s_period, s_ticksPerUs,
                s_latAvgUs, latMaxUs, IPH_SAMPLE_US + PHASE_ISR_MARGIN_US,
                s_dutyMax, s_dutyOp, 0.577f * s_dutyOp);
  if (lN < 1000)
    Serial.println("[iphase] AVISO: poucos eventos de PWM no aquecimento — "
                   "confira se o driver MCPWM esta rodando.");
  if (s_cal.valid)
    Serial.printf("[iphase] calibracao NVS v2: pino1=fase %c (%+d, %.0f A/V) "
                  "pino2=fase %c (%+d, %.0f A/V) | zeros %.3f/%.3f V | "
                  "corrente no %s\n",
                  'A' + s_cal.ph[0], s_cal.sign[0], (double)ampsPerVolt(0),
                  'A' + s_cal.ph[1], s_cal.sign[1], (double)ampsPerVolt(1),
                  s_cal.zeroRef[0], s_cal.zeroRef[1],
                  s_evCur ? "TEZ (INVERTIDO!)" : "TEP");
  else
    Serial.println("[iphase] SEM CALIBRACAO — malha de corrente BLOQUEADA. "
                   "Rode 'Calibrar corrente de fase' no app (roda suspensa).");
  return &s_cs;
}

// ---------------------------------------------------------------------------
// ZERO dos canais (motor desabilitado, roda parada => corrente real = 0)
// ---------------------------------------------------------------------------
static const char *refName(uint8_t r) {
  switch (r) {
    case IPH_REF_BIDIR: return "bidirecional (REF em V+/2)";
    case IPH_REF_LOW:   return "UNIDIRECIONAL (REF no GND)";
    case IPH_REF_HIGH:  return "UNIDIRECIONAL invertido (REF no V+)";
    default:            return "ainda nao medido / sem sinal plausivel";
  }
}

bool phaseCurrentZero() {
  if (!s_ready) return false;
  if (s_motor && s_motor->enabled) {
    Serial.println("[iphase] zero exige o motor DESABILITADO");
    return false;
  }
  // Com o motor desabilitado os duties são 0: low-side sempre conduzindo,
  // corrente nula, e as amostras de CORRENTE são o próprio zero. ~300
  // leituras em 300 ms derrubam o ruído do ADC para ~0,2 contagem.
  double acc[2] = {};
  int n = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < 300) {
    for (int k = 0; k < 2; k++) acc[k] += (float)s_curRaw[k];
    n++;
    delay(1);
  }
  bool ok = true;
  s_cal.zeroed = true;
  for (int k = 0; k < 2; k++) {
    const float zc = (float)(acc[k] / n);
    s_cal.zeroRef[k] = zc * ADC_V_PER_COUNT;
    s_cal.offset[k]  = s_cal.zeroRef[k];
    s_zeroAcc[k]     = (uint32_t)(zc * 64.0f);
  }
  s_zeroPinned = false;
  refresh();
  for (int k = 0; k < 2; k++) {
    Serial.printf("[iphase] canal %d: zero %.3f V (%.0f%% de VCC) -> %s\n",
                  k + 1, s_cal.zeroRef[k],
                  100.0f * s_cal.zeroRef[k] / PHASE_CUR_VCC_V,
                  refName(s_cal.ref[k]));
    if (s_cal.ref[k] != IPH_REF_BIDIR) ok = false;
  }
  if (!ok) {
    Serial.println("[iphase] Este modulo NAO mede corrente de fase: ela e");
    Serial.println("[iphase] ALTERNADA e o zero precisa ficar em V+/2. No");
    Serial.println("[iphase] INA240: REF1 no 3,3 V e REF2 no GND (divisor");
    Serial.println("[iphase] interno). Ver HARDWARE.md 4b.");
    return false;
  }
  calSave();   // o zero vale persistir sozinho; NÃO valida a calibração
  return true;
}

// ---------------------------------------------------------------------------
// CALIBRAÇÃO — mede mapa de fases, sinal e escala (fatos da fiação)
// ---------------------------------------------------------------------------
// Vetor DC de tensão baixa alinhado a cada fase, nas DUAS polaridades; a
// semi-diferença (r+ − r−)/2 cancela offset e deriva de modo comum e deixa só
// o que responde à corrente. Com o vetor na fase X: +I em X, −I/2 nas outras
// duas — o canal da fase X responde com a MAIOR amplitude, o sinal da
// resposta é o sinal do canal, e a razão própria/outras ~2 é a assinatura
// que prova que a leitura é mesmo corrente de fase.
//
// A escala A/V sai do INA226 do barramento: com rotor parado e vetor DC,
// I_bus = duty x I_fase => I_fase = I_bus/duty, sem precisar saber o shunt.
static float s_iPhaseCal[3] = { 0, 0, 0 };

static bool calVector(float angleEl, float volts, float out[2], float *iPhase) {
  const uint32_t RAMP_MS = 200, HOLD_MS = 250;

  // linha de base do barramento (consumo da eletrônica parada) — sem tirar
  // isto, dividir pelo duty pequeno inventa dezenas de ampères
  double base = 0; int nb = 0;
  uint32_t tb = millis();
  while (millis() - tb < 60) {
    s_motor->setPhaseVoltage(0, 0, angleEl);
    base += ina226CurrentA(); nb++;
    delay(2);
  }
  const float iBase = (float)(base / fmaxf(1, nb));

  const float MIN_DUTY = 0.03f;
  auto dutyNow = [&]() {
    return fmaxf(s_driver->dc_a, fmaxf(s_driver->dc_b, s_driver->dc_c));
  };
  // guarda: corrente de FASE deduzida do barramento (EMA + persistência)
  float iFilt = 0; int over = 0;
  auto guard = [&]() -> bool {
    const float d = dutyNow();
    if (d < MIN_DUTY) return false;
    iFilt += 0.3f * (fabsf(ina226CurrentA() - iBase) / d - iFilt);
    if (iFilt > PHASE_CAL_I_MAX_A) {
      if (++over >= 5) {
        s_motor->setPhaseVoltage(0, 0, 0);
        // A guarda deduz a corrente do INA226 — se a ESCALA dele estiver
        // errada (parametro 'shunt'), ela aborta em amperes de mentira.
        // Imprime tambem a leitura dos PROPRIOS INA240 para o confronto.
        float ph[3];
        phaseAmps(ph);
        const float ib2 = _1_SQRT3 * (ph[1] - ph[2]);
        const float own = sqrtf(ph[0] * ph[0] + ib2 * ib2);
        Serial.printf("[iphase] CAL ABORTADA: corrente de fase %.1f A acima "
                      "do limite de %.1f A\n", iFilt, (double)PHASE_CAL_I_MAX_A);
        Serial.printf("[iphase]   deduzida do INA226: %.1f A | medida pelos "
                      "INA240 (escala atual): %.1f A\n", iFilt, own);
        Serial.println("[iphase]   se os dois divergem MUITO, o suspeito e a "
                       "escala do INA226: calibre com 'ibat' contra o "
                       "amperimetro da fonte ('shunt <mOhm>' + 'save') e so "
                       "entao rode a cal.");
        return true;
      }
    } else over = 0;
    return false;
  };

  uint32_t t0 = millis();
  while (millis() - t0 < RAMP_MS) {           // rampa: sem degrau, sem inrush
    s_motor->setPhaseVoltage(0, volts * (millis() - t0) / (float)RAMP_MS, angleEl);
    if (guard()) return false;
    delay(2);
  }
  double acc[2] = {}, accI = 0, accD = 0; int n = 0;
  t0 = millis();
  while (millis() - t0 < HOLD_MS) {
    s_motor->setPhaseVoltage(0, volts, angleEl);
    for (int k = 0; k < 2; k++) acc[k] += (float)s_curRaw[k] * ADC_V_PER_COUNT;
    accI += (ina226CurrentA() - iBase);
    accD += dutyNow();
    n++;
    if (guard()) return false;
    delay(2);
  }
  s_motor->setPhaseVoltage(0, 0, 0);
  for (int k = 0; k < 2; k++) out[k] = (float)(acc[k] / n);
  const float duty = (float)(accD / n);
  *iPhase = (duty > MIN_DUTY) ? fabsf((float)(accI / n)) / duty : 0.0f;
  delay(150);                                  // deixa a corrente zerar
  return true;
}

// Varre as 3 fases nas duas polaridades; preenche resp[][] e s_iPhaseCal[].
static bool calSweep(float volts, float *bestV, float *maxI) {
  *bestV = 0; *maxI = 0;
  for (int p = 0; p < 3; p++) {
    const float ang = p * (2.0f * PI / 3.0f);
    float rp[2], rm[2], ip = 0, im = 0;
    if (!calVector(ang,      volts, rp, &ip)) return false;
    if (!calVector(ang + PI, volts, rm, &im)) return false;
    for (int k = 0; k < 2; k++)
      s_cal.resp[k][p] = (rp[k] - rm[k]) * 0.5f;
    s_iPhaseCal[p] = 0.5f * (ip + im);
    Serial.printf("[iphase]   %.1f V na fase %c ->", volts, 'A' + p);
    for (int k = 0; k < 2; k++)
      Serial.printf("  canal%d %+.4f V (%+.1f LSB)", k + 1,
                    s_cal.resp[k][p], s_cal.resp[k][p] / ADC_V_PER_COUNT);
    Serial.printf(" | I_fase %.2f A\n", s_iPhaseCal[p]);
    for (int k = 0; k < 2; k++)
      *bestV = fmaxf(*bestV, fabsf(s_cal.resp[k][p]));
    *maxI  = fmaxf(*maxI, s_iPhaseCal[p]);
  }
  return true;
}

bool phaseCurrentCalibrate() {
  if (!s_ready || !s_motor) return false;
  Serial.println("[iphase] === CALIBRACAO DA CORRENTE DE FASE (roda suspensa) ===");

  s_cal.valid = false;
  refresh();                                   // desliga o zero vivo durante a cal
  s_motor->disable();
  delay(200);
  if (!phaseCurrentZero()) {
    Serial.println("[iphase] CAL FALHOU no zero — confira alimentacao/REF/pinos");
    calSave();
    return false;
  }

  // a calibração comanda TENSÃO direto (vetor DC), nunca malha
  TorqueControlType prevTc = s_motor->torque_controller;
  MotionControlType prevMc = s_motor->controller;
  float prevVl = s_motor->voltage_limit;
  s_motor->controller        = MotionControlType::torque;
  s_motor->torque_controller = TorqueControlType::voltage;
  s_motor->voltage_limit     = PHASE_CAL_V_MAX + 0.5f;
  s_motor->enable();

  const float minV = PHASE_CAL_MIN_LSB * ADC_V_PER_COUNT;
  const float tgtV = PHASE_CAL_TGT_LSB * ADC_V_PER_COUNT;

  // ---- 1) resposta no evento derivado do PWM (TEP) ------------------------
  // O instante NÃO é chutado: decorre do contador up-down + duty mode 0 do
  // driver (janela low-side centrada no TEP). O passo abaixo é VERIFICAÇÃO:
  // se não houver resposta, testa o evento oposto uma vez — pega o caso de
  // uma atualização da lib inverter a polaridade, avisando alto.
  bool aborted = false;
  float volts = PHASE_CAL_V_START, best = 0, maxI = 0;
  s_evCur = 0;                                  // corrente no TEP (normal)
  delay(20);
  if (!calSweep(volts, &best, &maxI)) aborted = true;
#if IPH_INLINE
  // Sem sondagem do evento oposto: com o shunt em série a corrente responde
  // nos dois centros por construção, então "não respondeu no TEP" só pode ser
  // canal morto, fiação errada ou o motor não girando — nunca polaridade de
  // PWM. Sondar o TEZ aqui só produziria um diagnóstico falso.
  if (!aborted && best < minV)
    Serial.println("[iphase] sem resposta no TEP. Com shunt EM SERIE isso nao "
                   "e polaridade de PWM: confira fiacao do shunt, do INA240 e "
                   "se o motor realmente girou.");
#else
  if (!aborted && best < minV) {
    Serial.println("[iphase] sem resposta no TEP — verificando o evento oposto "
                   "(isto NAO deveria acontecer com o driver atual)");
    s_evCur = 1;
    delay(20);
    if (!calSweep(volts, &best, &maxI)) aborted = true;
    if (!aborted && best >= minV)
      Serial.println("[iphase] AVISO GRAVE: resposta no TEZ — a polaridade do "
                     "PWM esta INVERTIDA em relacao ao driver conhecido. "
                     "Confira a versao da SimpleFOC.");
  }
#endif
  bool found = !aborted && best >= minV;
  if (found) s_cal.event = s_evCur;
#if IPH_PRETRIG_ENABLE
  // O pré-disparo foi armado no boot assumindo corrente no TEP; se ESTA
  // calibração acabou de descobrir o contrário, a geometria só se corrige
  // no próximo boot (o init cai sozinho no fallback legado).
  if (s_pretrig && s_evCur != 0)
    Serial.println("[iphase] ATENCAO: corrente detectada no TEZ com o "
                   "pre-disparo armado p/ TEP — REINICIE para aplicar a "
                   "geometria correta.");
#endif

  // ---- 2) sobe a tensão até ter resolução suficiente ----------------------
  for (int tries = 0; found && !aborted && tries < 3; tries++) {
    if (best >= tgtV) break;
    if (volts >= PHASE_CAL_V_MAX - 0.01f || maxI >= 0.8f * PHASE_CAL_I_MAX_A) {
      Serial.printf("[iphase] resolucao limitada a %.0f LSB (limite de %s)\n",
                    best / ADC_V_PER_COUNT,
                    volts >= PHASE_CAL_V_MAX - 0.01f ? "tensao" : "corrente");
      break;
    }
    float k = tgtV / fmaxf(best, minV);
    if (maxI > 0.1f) k = fminf(k, 0.70f * PHASE_CAL_I_MAX_A / maxI);
    k = fminf(k, PHASE_CAL_V_MAX / volts);
    if (k < 1.05f) break;
    volts *= k;
    Serial.printf("[iphase] sinal de %.0f LSB; repetindo com %.1f V\n",
                  best / ADC_V_PER_COUNT, volts);
    if (!calSweep(volts, &best, &maxI)) aborted = true;
  }

  s_motor->setPhaseVoltage(0, 0, 0);
  s_motor->disable();
  s_motor->torque_controller = prevTc;
  s_motor->controller        = prevMc;
  s_motor->voltage_limit     = prevVl;

  if (aborted || !found) {
    Serial.println(aborted
      ? "[iphase] CAL FALHOU: abortada por corrente."
      : "[iphase] CAL FALHOU: nenhuma resposta nos canais. Confira shunts, "
        "INA240 e alimentacao do estagio.");
    calSave();
    return false;
  }

  // ---- 3) mapa de fases, sinal e coerência --------------------------------
  bool ok = true;
  for (int k = 0; k < 2; k++) {
    int bp = 0; float bv = 0;
    for (int p = 0; p < 3; p++)
      if (fabsf(s_cal.resp[k][p]) > fabsf(bv)) { bv = s_cal.resp[k][p]; bp = p; }
    s_cal.ph[k]   = bp;
    s_cal.sign[k] = (bv > 0) ? 1 : -1;

    if (fabsf(bv) < minV) {
      Serial.printf("[iphase] canal %d: resposta propria de so %.0f LSB — "
                    "abaixo do minimo de %.0f\n",
                    k + 1, fabsf(bv) / ADC_V_PER_COUNT, (double)PHASE_CAL_MIN_LSB);
      ok = false;
    }
    // Assinatura da estrela: na fase própria +I; nas outras, −I/2 => a razão
    // própria/média(outras) tem de ficar ~2 e as outras com sinal oposto.
    float sumOther = 0;
    for (int p = 0; p < 3; p++) {
      if (p == bp) continue;
      sumOther += fabsf(s_cal.resp[k][p]);
      if (s_cal.resp[k][p] * s_cal.sign[k] > 0.15f * fabsf(bv)) {
        Serial.printf("[iphase] canal %d incoerente: fase %c respondeu com o "
                      "mesmo sinal da fase %c\n", k + 1, 'A' + p, 'A' + bp);
        ok = false;
      }
    }
    const float ratio = (sumOther > 1e-6f) ? fabsf(bv) / (sumOther * 0.5f) : 0.0f;
    Serial.printf("[iphase] canal %d -> fase %c, sinal %+d, razao %.2f "
                  "(esperado ~2,0)\n", k + 1, 'A' + bp, s_cal.sign[k], ratio);
    if (ratio < 1.3f || ratio > 3.5f) {
      Serial.printf("[iphase] canal %d: razao %.2f fora de 1,3-3,5 — sem a "
                    "assinatura de estrela trifasica\n", k + 1, ratio);
      ok = false;
    }
  }
  if (ok && s_cal.ph[0] == s_cal.ph[1]) {
    Serial.printf("[iphase] CAL FALHOU: os dois canais apontam para a MESMA "
                  "fase (%c). Confira em qual fase cada shunt esta.\n",
                  'A' + s_cal.ph[0]);
    ok = false;
  }

  // ---- 4) escala A/V por canal, cruzada com o INA226 ----------------------
  if (ok) {
    // A corrente de fase é a mesma nos três vetores; a MEDIANA descarta a
    // medida corrompida por movimento do rotor (back-EMF no barramento).
    const float a = s_iPhaseCal[0], b = s_iPhaseCal[1], c = s_iPhaseCal[2];
    const float iRef = fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
    Serial.printf("[iphase] corrente de referencia: %.2f A (mediana de "
                  "%.2f / %.2f / %.2f)\n", iRef, a, b, c);
    const float spread = (iRef > 0.1f)
        ? (fmaxf(a, fmaxf(b, c)) - fminf(a, fminf(b, c))) / iRef : 9.9f;
    if (spread > 0.4f)
      Serial.printf("[iphase] AVISO: medidas de corrente divergem %.0f%% — "
                    "roda girando durante a calibracao? (prenda a roda)\n",
                    spread * 100.0f);
    for (int k = 0; k < 2; k++) {
      const float r = fabsf(s_cal.resp[k][s_cal.ph[k]]);
      if (r > minV && iRef > 0.3f) {
        s_cal.ampsPerVolt[k] = iRef / r;
        Serial.printf("[iphase] canal %d: %.0f A/V (%.3f A por contagem)\n",
                      k + 1, s_cal.ampsPerVolt[k],
                      s_cal.ampsPerVolt[k] * ADC_V_PER_COUNT);
      } else {
        // reprova: um canal medido e outro no nominal = escalas
        // inconsistentes = Clarke distorcida e proteção falseada
        s_cal.ampsPerVolt[k] = 0.0f;
        Serial.printf("[iphase] FALHOU: sem escala do canal %d (resposta "
                      "%.1f LSB, corrente %.2f A)\n",
                      k + 1, r / ADC_V_PER_COUNT, iRef);
        ok = false;
      }
    }
  }

  // ---- 5) fundo de escala e resolução -------------------------------------
  if (ok) {
    const float swing = fminf(s_cal.zeroRef[0] - 0.20f, 3.10f - s_cal.zeroRef[0]);
    const float apvMin = fminf(s_cal.ampsPerVolt[0], s_cal.ampsPerVolt[1]);
    const float apvMax = fmaxf(s_cal.ampsPerVolt[0], s_cal.ampsPerVolt[1]);
    const float fullScale = swing * apvMin;
    Serial.printf("[iphase] fundo de escala: +-%.1f A (trip de fase %.0f A)\n",
                  fullScale, prm(P_IPH_TRIP));
    if (fullScale < IPH_FS_MARGIN * prm(P_IPH_TRIP)) {
      Serial.printf("[iphase] SHUNT GRANDE DEMAIS: o sensor satura em %.1f A "
                    "mas a protecao age em %.0f A — acima da saturacao a "
                    "leitura MENTE para baixo e a malha dispara. Troque o "
                    "shunt/ganho.\n", fullScale, prm(P_IPH_TRIP));
      ok = false;
    }
    const float aPerLsb = apvMax * ADC_V_PER_COUNT;
    Serial.printf("[iphase] resolucao (pior canal): %.3f A por contagem\n", aPerLsb);
    if (aPerLsb > 0.20f)
      Serial.printf("[iphase] AVISO: resolucao GROSSA (~%.1f A de ruido no Iq). "
                    "Melhorar e HARDWARE: INA240A4 ou shunt maior.\n",
                    3.0f * aPerLsb);
  }

  s_cal.valid = ok;
  calSave();
  if (ok)
    Serial.printf("[iphase] === CAL OK: pino %d = fase %c (%+d) | pino %d = "
                  "fase %c (%+d) | corrente no %s ===\n",
                  PIN_IPH_1, 'A' + s_cal.ph[0], s_cal.sign[0],
                  PIN_IPH_2, 'A' + s_cal.ph[1], s_cal.sign[1],
                  s_cal.event ? "TEZ (INVERTIDO)" : "TEP");
  else
    Serial.println("[iphase] === CAL FALHOU — malha de corrente segue bloqueada ===");
  return ok;
}

// ---------------------------------------------------------------------------
// DIAGNÓSTICO
// ---------------------------------------------------------------------------
#if IPH_DIAG_TOOLS
// ---------------------------------------------------------------------------
// ANÁLISE COMPLETA DE RUÍDO — cada etapa da cadeia, do ADC cru ao sinal final
// ---------------------------------------------------------------------------
// Captura CADA amostra nova do ADC (rastreando o contador de sequência da
// ISR: 8 kHz por canal, nenhuma amostra pulada nem repetida — reamostrar
// devagar esconde picos e mascara o espectro) e propaga a MESMA captura pela
// cadeia inteira: contagens cruas -> zero em uso -> ampères por canal ->
// fase reconstruída -> Clarke/|I| -> Park cru. Depois mede o sinal FINAL na
// cadência real da malha (1 kHz, id/iq filtrados por P_CUR_TF). Cada número
// é o ruído DAQUELA etapa, medido — dá para ver exatamente onde ele entra e
// onde é atenuado. Rodar com o motor DESABILITADO (corrente real = 0: tudo
// que aparecer É ruído; a média que sobrar é erro de zero).
#define NR_N 2048          // pares de amostras (256 ms a 8 kHz por canal)

struct NoiseStat {
  double s = 0, q = 0; float mn = 1e9f, mx = -1e9f; int n = 0;
  inline void add(float v) {
    s += v; q += (double)v * v;
    if (v < mn) mn = v; if (v > mx) mx = v; n++;
  }
  float mean() const { return n ? (float)(s / n) : 0.0f; }
  float sd()   const {
    if (!n) return 0.0f;
    const float m = mean();
    return sqrtf(fmaxf(0.0f, (float)(q / n) - m * m));
  }
  float pp()   const { return n ? (mx - mn) : 0.0f; }
};

static void noiseHist(const char *tag, const uint16_t *b, int n, float mean) {
  // histograma do desvio em contagens (-6..+6): mostra a FORMA do ruído —
  // gaussiano estreito = ADC; bimodal/caudas = interferência periódica
  uint16_t h[13] = {}; uint16_t lo = 0, hi = 0;
  for (int i = 0; i < n; i++) {
    const int d = (int)lroundf((float)b[i] - mean);
    if      (d < -6) lo++;
    else if (d >  6) hi++;
    else             h[d + 6]++;
  }
  Serial.printf("[inoise] hist %s (desvio, cnt) -6..+6:", tag);
  for (int i = 0; i < 13; i++)
    Serial.printf(" %2d", (int)lroundf(100.0f * h[i] / n));
  Serial.printf("  %% | fora: %u/%u\n", lo, hi);
}

static void noiseReport(const char *title) {
  static uint16_t b0[NR_N], b1[NR_N];       // 8 KB — só com IPH_DIAG_TOOLS 1

  // ---- captura: cada amostra nova, pelas sequências da ISR ---------------
  uint32_t lost = 0;
  uint32_t s0 = s_curSeq[0], s1 = s_curSeq[1];
  const uint32_t tCap0 = micros();
  for (int i = 0; i < NR_N; i++) {
    uint32_t tw = micros();
    while (s_curSeq[0] == s0) {
      if (micros() - tw > 3000) {
        Serial.println("[inoise] ABORTADO: ISR sem amostras novas (>3 ms)");
        return;
      }
    }
    uint32_t ns = s_curSeq[0]; lost += ns - s0 - 1; s0 = ns;
    b0[i] = s_curRaw[0];
    tw = micros();
    while (s_curSeq[1] == s1) {
      if (micros() - tw > 3000) {
        Serial.println("[inoise] ABORTADO: ISR sem amostras novas (>3 ms)");
        return;
      }
    }
    ns = s_curSeq[1]; lost += ns - s1 - 1; s1 = ns;
    b1[i] = s_curRaw[1];
  }
  const float capMs = (micros() - tCap0) * 1e-3f;

  // ---- escalas e zeros em uso (congelados para a análise) ----------------
  const float apc[2] = { ampsPerVolt(0) * ADC_V_PER_COUNT,
                         ampsPerVolt(1) * ADC_V_PER_COUNT };   // [A/cnt]
  bool pin = false;
  const float zC[2] = { zeroUseV(0, &pin) / ADC_V_PER_COUNT,
                        zeroUseV(1, &pin) / ADC_V_PER_COUNT }; // [cnt]
  const int p0 = s_cal.valid ? s_cal.ph[0] : 0;
  const int p1 = s_cal.valid ? s_cal.ph[1] : 1;
  const int pr = 3 - p0 - p1;                 // fase reconstruída
  const int sg0 = s_cal.valid ? s_cal.sign[0] : 1;
  const int sg1 = s_cal.valid ? s_cal.sign[1] : 1;
  const float ang = s_motor ? s_motor->electrical_angle : 0.0f;

  // ---- passada 1: cru por canal ------------------------------------------
  NoiseStat r0, r1;
  for (int i = 0; i < NR_N; i++) { r0.add(b0[i]); r1.add(b1[i]); }

  // ---- passada 2: derivadas por par + correlação -------------------------
  NoiseStat phS[3], alS, beS, mgS, idS, iqS;
  double cross = 0;
  const float m0 = r0.mean(), m1 = r1.mean();
  const float ct = _cos(ang), st = _sin(ang);
  for (int i = 0; i < NR_N; i++) {
    cross += (double)(b0[i] - m0) * (double)(b1[i] - m1);
    float ph[3];
    ph[p0] = ((float)b0[i] - zC[0]) * apc[0] * sg0;
    ph[p1] = ((float)b1[i] - zC[1]) * apc[1] * sg1;
    ph[pr] = -ph[p0] - ph[p1];
    const float ia = ph[0], ib = _1_SQRT3 * (ph[1] - ph[2]);
    phS[0].add(ph[0]); phS[1].add(ph[1]); phS[2].add(ph[2]);
    alS.add(ia); beS.add(ib);
    mgS.add(sqrtf(ia * ia + ib * ib));
    idS.add( ia * ct + ib * st);
    iqS.add( ib * ct - ia * st);
  }
  const float corr = (r0.sd() > 0.01f && r1.sd() > 0.01f)
      ? (float)(cross / NR_N) / (r0.sd() * r1.sd()) : 0.0f;

  // ---- relatório ----------------------------------------------------------
  Serial.printf("[inoise] ======== %s ========\n", title);
  Serial.printf("[inoise] captura: %d amostras/canal a %.0f Hz (%.0f ms) | "
                "perdidas %lu\n", NR_N, NR_N / (capMs * 0.001f), capMs,
                (unsigned long)lost);
  Serial.printf("[inoise] 1 cnt = %.3f mV = %.3f A (ch1) / %.3f A (ch2)\n",
                (double)PHASE_CUR_MV_PER_COUNT, apc[0], apc[1]);
  Serial.printf("[inoise] CRU ch1: media %.2f cnt (%.4f V) | dp %.2f cnt "
                "(%.1f mV, %.3f A) | p-p %.0f (%.0f..%.0f)\n",
                m0, m0 * ADC_V_PER_COUNT, r0.sd(),
                r0.sd() * PHASE_CUR_MV_PER_COUNT, r0.sd() * apc[0],
                r0.pp(), r0.mn, r0.mx);
  Serial.printf("[inoise] CRU ch2: media %.2f cnt (%.4f V) | dp %.2f cnt "
                "(%.1f mV, %.3f A) | p-p %.0f (%.0f..%.0f)\n",
                m1, m1 * ADC_V_PER_COUNT, r1.sd(),
                r1.sd() * PHASE_CUR_MV_PER_COUNT, r1.sd() * apc[1],
                r1.pp(), r1.mn, r1.mx);
  Serial.printf("[inoise] correlacao ch1xch2: %+.2f (0 = independente; alto = "
                "modo comum entrando nos dois)\n", corr);
  noiseHist("ch1", b0, NR_N, m0);
  noiseHist("ch2", b1, NR_N, m1);
  Serial.printf("[inoise] ZERO em uso: %.2f / %.2f cnt -> residuo DC: "
                "ch1 %+.2f cnt (%+.3f A) | ch2 %+.2f cnt (%+.3f A)\n",
                zC[0], zC[1], m0 - zC[0], (m0 - zC[0]) * apc[0] * sg0,
                m1 - zC[1], (m1 - zC[1]) * apc[1] * sg1);
  Serial.printf("[inoise] FASE [A] %c: dp %.3f p-p %.2f | %c: dp %.3f p-p %.2f "
                "| %c (reconstruida): dp %.3f p-p %.2f\n",
                'A' + p0, phS[p0].sd(), phS[p0].pp(),
                'A' + p1, phS[p1].sd(), phS[p1].pp(),
                'A' + pr, phS[pr].sd(), phS[pr].pp());
  Serial.printf("[inoise] CLARKE: ialpha dp %.3f | ibeta dp %.3f | "
                "|I|: media %.3f dp %.3f max %.2f A\n",
                alS.sd(), beS.sd(), mgS.mean(), mgS.sd(), mgS.mx);
  Serial.printf("[inoise] PARK cru (ang %.2f rad): id dp %.3f | iq dp %.3f A\n",
                ang, idS.sd(), iqS.sd());

  // ---- sinal FINAL: id/iq FILTRADOS, na cadência real da malha (1 kHz) ---
  NoiseStat fq, fd, fm;
  const uint32_t tf0 = micros();
  int k = 0;
  while (k < 600) {
    if ((int32_t)(micros() - tf0) >= (k + 1) * 1000) {
      k++;
      PhaseCur pc;
      phaseCurrentRead(&pc, ang);
      fq.add(pc.iq); fd.add(pc.id); fm.add(pc.imag);
    }
  }
  Serial.printf("[inoise] FINAL 1 kHz (Tf=%.1f ms — o que a malha consome): "
                "iq dp %.3f p-p %.2f | id dp %.3f | |I| cru dp %.3f A\n",
                prm(P_CUR_TF) * 1000.0f, fq.sd(), fq.pp(), fd.sd(), fm.sd());
  // VEREDITO DA ZONA MORTA — agora contra o ruído de id/iq FINAIS (é o que
  // entra nos integradores), não só contra |I| cru. O eixo d é o crítico:
  // ruído de id que fura a zona morta vira Ud, e Ud vira corrente REAL.
  {
    const float db  = prm(P_CUR_DB);
    const float pior = fmaxf(fd.sd(), fq.sd());
    Serial.printf("[inoise] zona morta P_CUR_DB=%.2f A vs ruido FINAL "
                  "(id dp %.3f | iq dp %.3f) -> %s\n",
                  (double)db, fd.sd(), fq.sd(),
                  db >= 1.5f * pior
                    ? "cobre com folga"
                    : (db >= pior ? "cobre, mas SEM folga (mire em 1,5x)"
                                  : "MENOR QUE O RUIDO: o integrador persegue "
                                    "ruido e vira corrente real — 'db <A>'"));
    if (db < 1.5f * pior)
      Serial.printf("[inoise]   sugestao: db %.2f  (1,5x o pior ruido final)\n",
                    (double)(1.5f * pior));
  }
}

uint32_t phaseCurrentRejects() { return s_rejCur; }

void phaseCurrentDiag(uint32_t ms) {
  if (!s_ready) { Serial.println("[idiag] sensor nao inicializado"); return; }
  Serial.printf("[idiag] === diagnostico (%lu ms) ===\n", (unsigned long)ms);
  Serial.printf("[idiag] zero vivo %.4f / %.4f V | zero da cal %.4f / %.4f V | "
                "pinned=%d\n", zeroV(0), zeroV(1),
                s_cal.zeroRef[0], s_cal.zeroRef[1], (int)s_zeroPinned);
  Serial.printf("[idiag] canal1 = fase %c(%+d) %.0f A/V | canal2 = fase %c(%+d) "
                "%.0f A/V\n",
                'A' + s_cal.ph[0], s_cal.sign[0], ampsPerVolt(0),
                'A' + s_cal.ph[1], s_cal.sign[1], ampsPerVolt(1));
  Serial.printf("[idiag] latencia media %.2f us | duty max %.3f (operacao "
                "%.3f) | rejeicoes: "
                "corrente %lu, zero %lu, SALTO-ZERO %lu, atrasadas %lu, dupla "
                "%lu, nao-pronta %lu\n", s_latAvgUs, s_dutyMax, s_dutyOp,
                (unsigned long)s_rejCur, (unsigned long)s_rejZero,
                (unsigned long)s_rejZeroJump,
                (unsigned long)s_rejLate, (unsigned long)s_rejBoth,
                (unsigned long)s_rejNotReady);
  Serial.printf("[idiag] amostras acumuladas: corrente %lu/%lu | zero %lu/%lu\n",
                (unsigned long)s_curSeq[0], (unsigned long)s_curSeq[1],
                (unsigned long)s_zeroSeq[0], (unsigned long)s_zeroSeq[1]);
  // As duas verificações de boot que o monitor serial costuma perder (ele
  // conecta depois de elas imprimirem): prioridade da ISR e trava de power
  // do SAR (errata GPIO-3.11). Aqui elas não escapam.
  Serial.printf("[idiag] ISR de amostragem: nivel %s | SAR power "
                "FORCE_XPD_SAR=%lu (3 = travado LIGADO; 0 = FSM = errata "
                "GPIO39 ativa) | reafirmacoes da trava: %lu\n",
                s_isrLevel3 ? "3 (preempta os Halls)" : "PADRAO (!) — "
                "latencia sujeita as ISRs de Hall",
                (unsigned long)GET_PERI_REG_BITS2(SENS_SAR_MEAS_WAIT2_REG,
                                                  SENS_FORCE_XPD_SAR,
                                                  SENS_FORCE_XPD_SAR_S),
                (unsigned long)s_xpdFix);

  // Análise completa de ruído em TODAS as etapas, com e sem rádio — a
  // diferença entre os dois blocos é exatamente o que o BLE injeta.
  noiseReport("RUIDO — RADIO ATIVO (motor desligado, corrente real = 0)");
  Serial.println("[idiag] --- parando o radio (advertising + conexoes) ---");
  bleRadioPause(true);
  delay(400);
  noiseReport("RUIDO — RADIO PARADO");
  bleRadioPause(false);
  Serial.println("[idiag] radio religado (reconecte o app)");

  Serial.println("[idiag] --- amostras ---");
  Serial.println("[idiag]  cnt1  cnt2  z1    z2   |   ia     ib     ic    |I|"
                 "     iq     id | duty A/B/C");
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    PhaseCur pc;
    phaseCurrentRead(&pc, s_motor ? s_motor->electrical_angle : 0.0f);
    Serial.printf("[idiag] %5u %5u %5.0f %5.0f | %+6.2f %+6.2f %+6.2f %6.2f "
                  "%+6.2f %+6.2f | %.2f %.2f %.2f\n",
                  s_curRaw[0], s_curRaw[1], zeroCnt(0), zeroCnt(1),
                  pc.ia, pc.ib, pc.ic, pc.imag, pc.iq, pc.id,
                  s_driver->dc_a, s_driver->dc_b, s_driver->dc_c);
    delay(100);
  }
  Serial.println("[idiag] === fim ===");
}

#else
void phaseCurrentDiag(uint32_t) {
  Serial.println("[idiag] ferramentas de diagnostico desligadas (IPH_DIAG_TOOLS 0)");
}
#endif  // IPH_DIAG_TOOLS

#endif  // PHASE_CUR_ENABLED
