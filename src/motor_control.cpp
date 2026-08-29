// ============================================================================
// motor_control.cpp — controle FOC no NÚCLEO 1
// ----------------------------------------------------------------------------
// Estrutura da task:
//   - loop principal roda motor.loopFOC() + motor.move() SEM bloqueios,
//     na máxima frequência possível (tipicamente vários kHz);
//   - a cada 1 ms ("fatia" de 1 kHz) roda o trabalho lento: leitura dos ADCs,
//     proteções, foldback de corrente, leitura do comando da dashboard e
//     cálculo do alvo com rampa;
//   - a cada 50 ms publica a telemetria para o núcleo 0.
//
// NOTAS DE HARDWARE IMPORTANTES:
//   - FD2103S com HIN+LIN unidos: PWM alto = MOSFET de cima, PWM baixo =
//     MOSFET de baixo. NÃO EXISTE estado de alta impedância! Com o motor
//     "desabilitado" (duty 0) as três fases ficam curto-circuitadas ao GND
//     pelos MOSFETs de baixo => girar a roda na mão com o sistema ligado
//     tem arrasto (freio dinâmico). É inerente ao hardware, não é bug.
//   - Dead-time é gerado DENTRO do FD2103S (~100 ns). O ESP32 só manda
//     um PWM por fase — nada de par complementar por software.
//   - CORRENTE DE FASE MEDIDA (2x INA240A1 low-side no ADC1): o torque agora
//     é controlado por MALHA DE CORRENTE própria (currentRegulator). O acelerador
//     pede AMPÈRES, não volts. Todos os estimadores de back-EMF que faziam as
//     vezes de limite de corrente foram REMOVIDOS — ver phase_current.h.
//   - O INA226 do barramento continua, agora no papel certo: limite de
//     corrente de BATERIA (foldback) e cobertura da faixa de modulação alta,
//     onde a janela de amostragem low-side fecha. Os dois são complementares.
// ============================================================================
#include <Arduino.h>
#include <math.h>
#include <SimpleFOC.h>
#include "config.h"
#include "shared_state.h"
#include "motor_control.h"
#include "ina226.h"
#include "phase_current.h"
#include "params.h"
#include "curves.h"
#include "faultlog.h"
#include <Preferences.h>

// ---------------------------------------------------------------------------
// Objetos SimpleFOC
// ---------------------------------------------------------------------------
static BLDCMotor       motor(MOTOR_POLE_PAIRS);
static BLDCDriver3PWM  driver(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C);

static HallSensor      hallSensor(PIN_HALL_A, PIN_HALL_B, PIN_HALL_C, MOTOR_POLE_PAIRS);

#if HALL_INTERP
// ---------------------------------------------------------------------------
// Interpolação do ângulo de Hall (ver a nota HALL_INTERP no config.h).
// O nome NÃO é decorativo: a base Sensor declara `friend class
// SmoothingSensor` — este é o ponto de integração que a própria SimpleFOC
// reservou para um wrapper de suavização.
//   ângulo = base do setor (borda de entrada) + velocidade x tempo desde a
//            borda, TRAVADO na largura do setor (nunca invade o próximo).
// A velocidade do HallSensor é por período de pulso e AUTO-ZERA quando fica
// velha (>2 períodos) — parado, o avanço morre sozinho e o comportamento
// vira o do Hall puro. Na chegada de cada borda o avanço recomeça de zero
// sobre a base nova: se a previsão fechou o setor, a emenda é contínua; se
// foi travada (aceleração), sobra um passinho para FRENTE limitado — nunca o
// salto para trás da implementação antiga.
// ---------------------------------------------------------------------------
// Avanço de comutação vivo [µs] — cache de P_COMM_LEAD atualizado na fatia
// (o update() do sensor roda no loop rápido; leitura de float é atômica).
static float s_commLeadUs = COMM_LEAD_US_DEFAULT;

class SmoothingSensor : public Sensor {
 public:
  SmoothingSensor(HallSensor &s, float sectorMechRad)
      : _s(s), _sec(sectorMechRad) {}
  float getVelocity() override { return _s.getVelocity(); }
  int   needsSearch() override { return _s.needsSearch(); }
  void  update() override {
    _s.update();     // base do setor em [0,2π) + timestamp da ÚLTIMA BORDA
    const float v = _s.getVelocity();
    // AVANÇO DE COMUTAÇÃO (P_COMM_LEAD): extrapola para o instante em que a
    // tensão ATUA nos FETs, não para "agora" — sem isto o vetor aplicado
    // inclina ω·τ com a velocidade e a malha de d paga a conta em Ud
    // (~5 V no topo, medidos na caixa-preta crua de 28/ago).
    const float lead = v * (s_commLeadUs * 1e-6f);
    float adv = v * (float)((long)(_micros() - _s.angle_prev_ts)) * 1e-6f
              + lead;
    // Trava = largura do setor + o próprio avanço: o rotor REALMENTE está
    // ω·τ à frente no instante da atuação; só a extrapolação além disso
    // (velocidade desatualizada) é que precisa de rédea.
    const float cap = _sec + fabsf(lead);
    if (adv >  cap) adv =  cap;
    if (adv < -cap) adv = -cap;
    float   a  = _s.angle_prev + adv;
    int32_t fr = _s.full_rotations;
    if      (a >= _2PI) { a -= _2PI; fr += 1; }
    else if (a <  0.0f) { a += _2PI; fr -= 1; }
    angle_prev     = a;
    full_rotations = fr;
    angle_prev_ts  = _s.angle_prev_ts;
  }
 protected:
  float getSensorAngle() override { return _s.getMechanicalAngle(); }
  HallSensor &_s;
  float       _sec;              // largura MECÂNICA de um setor (60°el/pp)
};
static SmoothingSensor smoothHall(hallSensor,
                                  _2PI / (6.0f * MOTOR_POLE_PAIRS));
#endif

// ---------------------------------------------------------------------------
// printf que NUNCA bloqueia a task de controle: se o buffer TX da serial não
// tem espaço livre, a mensagem é DESCARTADA. Um print bloqueante no meio do
// loop FOC congela a comutação por ms e vira solavanco de torque — diagnóstico
// não pode custar dirigibilidade. (O buffer TX é de 1024 bytes, ver main.cpp.)
// ---------------------------------------------------------------------------
static void mcPrintf(const char *fmt, ...) {
  char b[160];
  va_list ap;
  va_start(ap, fmt);
  int len = vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  if (len <= 0) return;
  if ((int)Serial.availableForWrite() < len) return;   // sem espaço: descarta
  Serial.write((const uint8_t *)b, (size_t)min(len, (int)sizeof(b) - 1));
}

// ---------------------------------------------------------------------------
// ISRs dos Halls com FILTRO DE GLITCH (registradas no núcleo 1)
// ---------------------------------------------------------------------------
// O chaveamento de potência induz pulsos de ns~µs nos cabos de Hall. Cada
// pulso gera 2 bordas (ida e volta): descartando TODAS as bordas que chegam
// a menos de HALL_GLITCH_US da anterior, o pulso inteiro é ignorado e o
// estado final fica correto. Bordas reais têm folga de sobra (>= 2,5 ms por
// pino até 30 km/h). O que ainda escapar é corrigido pela reconciliação de
// 1 kHz no controlSlice(). hallEdgeCount conta as bordas CRUAS — na
// dashboard, bordas/s > 0 com a roda parada = ruído induzido comprovado.
static volatile uint32_t hallEdgeCount = 0;
static volatile uint32_t lastEdgeUsA = 0, lastEdgeUsB = 0, lastEdgeUsC = 0;

// --- PROVA DE LEITURA DOS HALLS (verificador de sequência, roda na ISR) -----
// A sequência dos 6 estados é FIXA (como um relógio). Numa leitura perfeita,
// toda transição avança exatamente 1 setor, sempre no mesmo sentido enquanto
// a roda gira num sentido só. Qualquer coisa diferente é leitura errada:
//   seqSkips    -> pulou 2+ setores (borda perdida OU borda falsa)
//   seqRevs     -> sequência inverteu (bounce; legítimo só se a roda inverter)
//   seqInvalid  -> estado 000/111 entrou na lib (fio solto/curto)
//   edgeCntA/B/C-> bordas aceitas por pino; os 3 DEVEM ter a mesma taxa
static volatile uint32_t edgeCntA = 0, edgeCntB = 0, edgeCntC = 0;
// Forense de bordas DESCARTADAS, por pino e por causa:
//   rejFilt* -> caiu na janela de 100 µs (borda aceita <100 µs antes no pino)
//   rejConf* -> nível demorou a ASSENTAR na chegada da borda (forense de
//               qualidade do sinal; a borda NÃO é mais descartada por isso)
static volatile uint16_t rejFiltA = 0, rejFiltB = 0, rejFiltC = 0;
static volatile uint16_t rejConfA = 0, rejConfB = 0, rejConfC = 0;
static volatile uint32_t seqSkips = 0, seqRevs = 0, seqInvalid = 0;
static volatile int8_t   seqPrevSector = -2;   // -2 = sem referência ainda
static volatile int8_t   seqPrevDir    = 0;
                                               // (controlSlice atualiza; ISR lê)

// Leitura de GPIO por registrador (segura e rápida dentro de ISR)
// Instante da última borda de Hall aceita e duração do setor anterior. É daqui
// que sai a velocidade rápida do limitador (velFastRads): o intervalo entre
// bordas é a medida de velocidade com menos atraso que este hardware tem.
static volatile uint32_t s_lastHallUs   = 0;
// TRÊS intervalos, não um. A janela de glitch aceita qualquer borda com 100 µs
// de folga; um par espúrio que passe por ela produz um intervalo curtíssimo, e
// um intervalo curtíssimo vira uma velocidade ABSURDA. Como é essa velocidade
// que o limitador usa, um único intervalo falso derruba o teto de corrente a
// zero por um instante — torque some e volta. Parado, onde o chaveamento induz
// bordas na fiação de Hall, isso acontece o tempo todo: é o "tremelicando".
// A MEDIANA de três descarta o intervalo isolado e custa dois setores de
// atraso (~14 ms a 4 km/h), ainda 6x mais rápido que o filtro de 80 ms.
static volatile uint32_t s_hallPer[3]   = { 0, 0, 0 };
static volatile uint8_t  s_hallPerI     = 0;

static inline uint32_t IRAM_ATTR fastRead(int pin) {
  return (pin < 32) ? ((GPIO.in >> pin) & 1)
                    : ((GPIO.in1.val >> (pin - 32)) & 1);
}

// Estado 000/111 NUNCA é válido (a sequência troca 1 bit por vez, nunca passa
// por eles). Se os 3 pinos leem isso no momento da borda, é glitch/afundamento
// da alimentação dos Halls em curso: BLOQUEIA a borda antes de entrar na lib
// (o pino gera outra interrupção na recuperação, então nada se perde).
// seqInvalid agora conta esses bloqueios — é o detector de "rail caiu".
static inline bool IRAM_ATTR hallStateInvalid() {
#if (PIN_HALL_A >= 32) && (PIN_HALL_B >= 32) && (PIN_HALL_C >= 32)
  // Leitura ATÔMICA dos 3 pinos num único registrador (todos em GPIO32-39):
  // um snapshot só, sem o skew de ler 1 a 1 (que podia pegar um pino em plena
  // borda lenta do RC de 4,7 nF e montar um 000/111 falso).
  uint32_t r = GPIO.in1.val;
  uint8_t s = (uint8_t)((((r >> (PIN_HALL_A - 32)) & 1) << 2) |
                        (((r >> (PIN_HALL_B - 32)) & 1) << 1) |
                         ((r >> (PIN_HALL_C - 32)) & 1));
#else
  uint8_t s = (uint8_t)((fastRead(PIN_HALL_A) << 2) |
                        (fastRead(PIN_HALL_B) << 1) |
                         fastRead(PIN_HALL_C));
#endif
  if (s == 0 || s == 7) { seqInvalid = seqInvalid + 1; return true; }
  return false;
}

// PERMANENCIA POR SETOR — a geometria fisica dos Halls, medida.
// Com velocidade constante, tempo e angulo sao a mesma coisa: os 6 setores de
// 60 graus tem de dar 6 tempos iguais. Todo desvio aqui e erro de POSICAO dos
// sensores, e o angulo interpolado herda esse erro em forma de dente-de-serra
// (periodo 6x a frequencia eletrica). Isso vira ondulacao de torque que
// nenhum ajuste de ganho alcanca — por isso a medicao mora no ISR, o unico
// lugar onde existe o instante exato da borda.
static volatile uint32_t s_secAcc[6] = { 0, 0, 0, 0, 0, 0 };
static volatile uint16_t s_secCnt[6] = { 0, 0, 0, 0, 0, 0 };
static volatile uint32_t s_secPrevUs = 0;

static inline void IRAM_ATTR hallSeqCheck() {
  int8_t sec = ELECTRIC_SECTORS[hallSensor.hall_state & 0x07];
  if (sec < 0) { seqInvalid = seqInvalid + 1; seqPrevSector = -2; return; }
  if (seqPrevSector >= 0 && sec != seqPrevSector) {
    int8_t d = (int8_t)((sec - seqPrevSector + 6) % 6);
    if (d == 1)      { if (seqPrevDir == -1) seqRevs = seqRevs + 1; seqPrevDir =  1; }
    else if (d == 5) { if (seqPrevDir ==  1) seqRevs = seqRevs + 1; seqPrevDir = -1; }
    else             { seqSkips = seqSkips + 1; }   // pulou setor = leitura errada
    // O setor que ACABOU de terminar e o anterior; a permanencia dele e o
    // tempo desde a troca passada. So conta transicao limpa de 1 setor: um
    // salto ja e leitura errada e o tempo dele nao mede geometria nenhuma.
    const uint32_t nowSec = micros();
    const uint32_t dwell  = nowSec - s_secPrevUs;
    if ((d == 1 || d == 5) && s_secPrevUs != 0 &&
        dwell > 150 && dwell < 100000 && seqPrevSector < 6) {
      s_secAcc[seqPrevSector] += dwell;
      s_secCnt[seqPrevSector] = (uint16_t)(s_secCnt[seqPrevSector] + 1);
    }
    s_secPrevUs = nowSec;   // sempre, mesmo no salto: senao o proximo infla
  }
  seqPrevSector = sec;
}


static void IRAM_ATTR isrHallA() {
  hallEdgeCount = hallEdgeCount + 1;         // contadores de hall SEMPRE ligados
  uint32_t now = micros();
  if (now - lastEdgeUsA < HALL_GLITCH_US) { rejFiltA = rejFiltA + 1; return; }
#if HALL_CONFIRM_US > 0
  // ESPERA ASSENTAR (não descarta!): se o nível ainda oscila, aguarda até
  // estabilizar (máx. 3 esperas). O handleA() relê o pino no fim — pulso
  // falso vira estado repetido (a lib ignora) e borda REAL nunca se perde.
  // A forense provou que o descarte antigo comia bordas reais do Hall.
  {
    uint32_t lvlA = fastRead(PIN_HALL_A);
    for (int i = 0; i < 3; i++) {
      delayMicroseconds(HALL_CONFIRM_US);
      uint32_t l2 = fastRead(PIN_HALL_A);
      if (l2 == lvlA) break;
      lvlA = l2;
      rejConfA = rejConfA + 1;   // conta instabilidade (não descarta)
    }
  }
#endif
  if (hallStateInvalid()) return;
  lastEdgeUsA = now;
  if (s_lastHallUs) {
    s_hallPer[s_hallPerI] = now - s_lastHallUs;
    s_hallPerI = (uint8_t)((s_hallPerI + 1) % 3);
  }
  s_lastHallUs = now;
  hallSensor.handleA();
  edgeCntA = edgeCntA + 1;   // taxa por pino p/ data-log (sempre)
  hallSeqCheck();                  // verificador de comutação (sempre)
}
static void IRAM_ATTR isrHallB() {
  hallEdgeCount = hallEdgeCount + 1;
  uint32_t now = micros();
  if (now - lastEdgeUsB < HALL_GLITCH_US) { rejFiltB = rejFiltB + 1; return; }
#if HALL_CONFIRM_US > 0
  // ESPERA ASSENTAR (não descarta!): se o nível ainda oscila, aguarda até
  // estabilizar (máx. 3 esperas). O handleB() relê o pino no fim — pulso
  // falso vira estado repetido (a lib ignora) e borda REAL nunca se perde.
  // A forense provou que o descarte antigo comia bordas reais do Hall.
  {
    uint32_t lvlB = fastRead(PIN_HALL_B);
    for (int i = 0; i < 3; i++) {
      delayMicroseconds(HALL_CONFIRM_US);
      uint32_t l2 = fastRead(PIN_HALL_B);
      if (l2 == lvlB) break;
      lvlB = l2;
      rejConfB = rejConfB + 1;   // conta instabilidade (não descarta)
    }
  }
#endif
  if (hallStateInvalid()) return;
  lastEdgeUsB = now;
  if (s_lastHallUs) {
    s_hallPer[s_hallPerI] = now - s_lastHallUs;
    s_hallPerI = (uint8_t)((s_hallPerI + 1) % 3);
  }
  s_lastHallUs = now;
  hallSensor.handleB();
  edgeCntB = edgeCntB + 1;   // taxa por pino p/ data-log (sempre)
  hallSeqCheck();                  // verificador de comutação (sempre)
}
static void IRAM_ATTR isrHallC() {
  hallEdgeCount = hallEdgeCount + 1;
  uint32_t now = micros();
  if (now - lastEdgeUsC < HALL_GLITCH_US) { rejFiltC = rejFiltC + 1; return; }
#if HALL_CONFIRM_US > 0
  // ESPERA ASSENTAR (não descarta!): se o nível ainda oscila, aguarda até
  // estabilizar (máx. 3 esperas). O handleC() relê o pino no fim — pulso
  // falso vira estado repetido (a lib ignora) e borda REAL nunca se perde.
  // A forense provou que o descarte antigo comia bordas reais do Hall.
  {
    uint32_t lvlC = fastRead(PIN_HALL_C);
    for (int i = 0; i < 3; i++) {
      delayMicroseconds(HALL_CONFIRM_US);
      uint32_t l2 = fastRead(PIN_HALL_C);
      if (l2 == lvlC) break;
      lvlC = l2;
      rejConfC = rejConfC + 1;   // conta instabilidade (não descarta)
    }
  }
#endif
  if (hallStateInvalid()) return;
  lastEdgeUsC = now;
  if (s_lastHallUs) {
    s_hallPer[s_hallPerI] = now - s_lastHallUs;
    s_hallPerI = (uint8_t)((s_hallPerI + 1) % 3);
  }
  s_lastHallUs = now;
  hallSensor.handleC();
  edgeCntC = edgeCntC + 1;   // taxa por pino p/ data-log (sempre)
  hallSeqCheck();                  // verificador de comutação (sempre)
}

// ---------------------------------------------------------------------------
// Estado local da task (não compartilhado — só o núcleo 1 toca aqui)
// ---------------------------------------------------------------------------
#if PHASE_CUR_ENABLED
static PhaseCur s_iph         = {};     // última amostra de corrente de fase
static bool     s_iphOk       = false;  // sensor presente e zerado na faixa
static float    s_iphMagFilt  = 0.0f;   // |I| rápido  [A] — PROTEÇÕES
static float    s_iphMagDisp  = 0.0f;   // |I| lento   [A] — exibição
static float    s_iqTarget    = 0.0f;   // alvo de corrente do controle [A]
#endif
static float    iBatFilt      = 0.0f;   // corrente filtrada [A] (proteções)
static float    iBatDisp      = 0.0f;   // corrente EXIBIDA (filtro lento)
static float    tempDisp      = 25.0f;  // temperatura EXIBIDA (filtro lento)
static float    vBatFilt      = 0.0f;   // tensão filtrada [V]
static float    tempFilt      = 25.0f;  // temperatura filtrada [°C] (MOSFETs)
static float    motorTempFilt = 25.0f;  // temp filtrada do MOTOR (proteções)
static float    motorTempDisp = 25.0f;  // temp do MOTOR exibida (filtro lento)
static float    powerScale    = 1.0f;   // 0..1 — foldback de corrente
static float    appliedTarget = 0.0f;   // alvo entregue ao motor.move()
static float    velTarget     = 0.0f;   // alvo interno do modo velocidade [rad/s]
static float    velEst        = 0.0f;   // estimador PRÓPRIO de velocidade [rad/s]:
                                        // o getVelocity() dos Halls ZERA entre
                                        // pulsos em baixa rotação; este deriva o
                                        // ângulo e não colapsa (alimenta o
                                        // limitador BEMF, freio e telemetria)
static float    prevAngle     = 0.0f;   // ângulo da fatia anterior (p/ velEst)
static bool     prevAngleInit = false;

// Diagnóstico dos Halls (validação da fiação girando a roda na mão)
static uint8_t  hallBitsNow   = 0;      // estado atual (A<<2|B<<1|C)
static uint8_t  hallBitsLast  = 0xFF;
static uint8_t  hallSeenMask  = 0;      // estados válidos 1..6 já observados
#if PHASE_CUR_ENABLED
static float    iphMs         = 0.0f;   // [ms] acima do trip de |I| de fase
static float    fbBadMs       = 0.0f;   // [ms] realimentação de corrente ruim
static float    fbIqEma       = 0.0f;   // [A] média lenta do iq p/ a guarda
static float    clipMs        = 0.0f;   // [ms] saída do INA240 no trilho
// ---- TESTE 'gira' — estado (roda DENTRO da fatia, nunca a bloqueia) -------
// A primeira versão esperava com vTaskDelay dentro da task de controle: o
// controle parava junto, o PWM ficava em zero e o motor obviamente não girava.
// Um teste do sistema em regime não pode suspender o sistema que mede.
static bool s_wheelStill = false;   // roda parada DE VERDADE (bordas de Hall)
static volatile float s_spinReqA = -1.0f;  // >=0: alvo forçado pelo teste
enum : uint8_t { SPIN_OFF = 0, SPIN_SPINUP, SPIN_CAP, SPIN_COOL, SPIN_DONE,
                 SPIN_ABORT };
static volatile uint8_t s_spinPhase = SPIN_OFF;
static uint32_t s_spinT0    = 0;
static uint32_t s_spinSteady = 0;          // ms com a velocidade já assentada
static bool     s_spinSettled = false;      // chegou ao regime, ou estourou o tempo
static int      s_spinN     = 0;
static uint8_t  s_spinFail  = 0;           // 0=ok 2=estop/falta 3=disparou
static float    s_spinKmh   = SPIN_TEST_KMH;
#define SPIN_N         200                 // 200 ms de captura a 1 kHz
#define SPIN_SPINUP_MS 6000                // teto de espera pelo regime
#define SPIN_STEADY_MS  250                // assentado por este tempo = regime
#define SPIN_COOL_MS   1200                // roda desce antes da 2a passada
static int16_t  s_spinV[SPIN_N], s_spinI[SPIN_N];
static int16_t  s_spinC[SPIN_N], s_spinT[SPIN_N];   // teto e alvo de corrente
struct SpinRes { float kmh, sdKmh, pkKmh, amps, sdA, cap, tgt, feHz;
                 bool ok, settled; };
static SpinRes  s_spinRes;
static float    s_spdLimInt   = 0.0f;
static float    s_iqDemand    = 0.0f;   // pedido de corrente antes do teto   // integrador do limite de velocidade
static float    s_spdLimCap   = 0.0f;   // [A] teto de corrente que ele impõe
#endif
static uint16_t hallResyncs   = 0;      // correções de estado embaralhado (glitch)
static uint16_t hallEps       = 0;      // bordas cruas por segundo (ruído visível)
static uint32_t hallEdgeSnap  = 0;      // snapshot p/ calcular bordas/s
static uint16_t epsA = 0, epsB = 0, epsC = 0;  // bordas aceitas/s por pino

// Taxa de bordas dos Halls (bordas/s), SEMPRE ligada p/ o data-log de
// comutação: hallEps = ruído total; epsA/B/C = por pino (um destoando =
// canal/fio daquele Hall). Só computa fora do HALL_DIAG (senão o hallDiagTick
// faz — e os dois brigariam pelo mesmo snapshot).
static void hallRateTick() {
#if !HALL_DIAG
  // ESTADO DOS HALLS a 1 kHz (3 digitalRead, custo desprezível). Isto vivia
  // dentro do bloco HALL_DIAG, que é compilado FORA na build normal — então
  // hall_bits/hall_seen chegavam SEMPRE ZERADOS no app: uma tela de
  // diagnóstico de fiação que não diagnosticava nada. Amostrar aqui, na
  // fatia que já roda, devolve a função (gire a roda na mão e os 6 estados
  // válidos acendem em "hall_seen").
  {
    const uint8_t hb = (uint8_t)((digitalRead(PIN_HALL_A) << 2) |
                                 (digitalRead(PIN_HALL_B) << 1) |
                                  digitalRead(PIN_HALL_C));
    hallBitsNow = hb;
    if (hb >= 1 && hb <= 6) hallSeenMask |= (uint8_t)(1 << (hb - 1));
  }
  static uint32_t lastMs = 0, snE = 0, snA = 0, snB = 0, snC = 0;
  uint32_t now = millis();
  if (now - lastMs < 1000) return;
  lastMs = now;
  uint32_t e = hallEdgeCount, a = edgeCntA, b = edgeCntB, c = edgeCntC;
  hallEps = (uint16_t)min((uint32_t)65535, e - snE); snE = e;
  epsA = (uint16_t)min((uint32_t)65535, a - snA); snA = a;
  epsB = (uint16_t)min((uint32_t)65535, b - snB); snB = b;
  epsC = (uint16_t)min((uint32_t)65535, c - snC); snC = c;
#endif
}

static uint16_t linkDrops     = 0;      // ativações do dead-man (quedas de WiFi)
static uint16_t loopGapUs     = 0;
static volatile bool calZeaReq = false;   // pedidos de calibração vindos do BLE
static volatile bool calKeReq  = false;

// --- status do último comando (validações; o app lê pela telemetria) --------
static volatile uint8_t  s_cmdStatus   = 0;
static volatile uint32_t s_cmdStatusMs = 0;
void motorReportCmdStatus(uint8_t st) { s_cmdStatus = st; s_cmdStatusMs = millis(); }

// --- energia / odômetro ------------------------------------------------------
static uint32_t odoM = 0;         // [m] persistido (motorEnergyInit/Save)
static uint32_t tripM = 0;
static float    odoAcc = 0, tripWh = 0, tripWhRegen = 0;
static uint32_t odoSavedM = 0;
static bool     uvWarn = false;

// --- BATERIA INTELIGENTE -----------------------------------------------------
// Aprende a capacidade REAL do pack: entre dois repousos (corrente ~0 por 5 s,
// tensão sem sag), capacidade = Wh gastos / queda de SoC. O SoC vivo é
// contagem de energia ancorada no último repouso (imune ao sag sob carga).
// Consumo "recente" = EWMA por trecho de 200 m (a autonomia reage ao seu
// jeito de andar agora, não à média da viagem inteira).
static float    whNetTotal   = 0;      // Wh líquidos desde o boot (não zera c/ trip)
static float    capLearned   = 0;      // [Wh] capacidade aprendida (NVS)
static bool     capDirty     = false;
static float    socAnchor    = -1;     // SoC no último repouso (0..1)
static float    whAnchor     = 0;      // whNetTotal no último repouso
static float    socLive      = -1;     // SoC vivo (0..1)
static float    whkmRecent   = 18.0f;  // [Wh/km] consumo recente
static float    ckWh = 0, ckM = 0;     // checkpoint do consumo recente
static uint16_t restSec      = 0;

// [V] Ud aplicado pela malha do eixo d (reinjetado em motor.voltage.d
// depois do move()). Declarado AQUI, acima da caixa-preta, porque ela
// grava o Vd — sem ele o registro nao separa "o d esta bombeando" de
// "a comutacao esta errada".
static float s_udCmd = 0.0f;

static inline float socFromRestV(float v) {
  return constrain((v - 32.0f) / 10.0f, 0.0f, 1.0f);   // 10s: 32..42 V
}

// --- amostra rápida p/ STREAM BLE (escritor único = núcleo 1; floats são
// words alinhadas => leitura sem lock no núcleo 0) ---------------------------
static float   fsKmh = 0, fsIbat = 0, fsVbat = 0, fsTarget = 0;
static float   s_vmaxEffKmh = 0;      // vmax efetivo (modo+sessão+global)
static uint8_t fsDuty = 0;
static int8_t  fsTemp = 0;
static uint8_t fsFw = 0, fsScale = 100, fsHead = 100;   // performance p/ stream
static float   fsIRaw = 0, fsEffVLim = 0;              // core p/ data-log profundo
static float   fsThrEff = 0, fsBrkEff = 0;            // acel/freio após a curva
static float   fsRegenILim = 0;                      // limite de regen efetivo
static uint8_t fsTempScale = 100, fsTracVScale = 100;
static int8_t  fsMtpa = 0;

// --- CAIXA-PRETA: os últimos 512 ms congelados na 1ª falta que trava --------
// Diagnóstico por sintoma esgotou: isto grava a 1 kHz (alvo de iq, iq medido,
// |I|, velocidade rápida por Hall, Vq, setor) e CONGELA sozinho quando uma
// falta trava. O comando 'crash' despeja — a ordem causal aparece em vez de
// ser deduzida. Fica atrás de IPH_DIAG_TOOLS (5 KB de RAM de bancada).
#if PHASE_CUR_ENABLED && IPH_DIAG_TOOLS
#define BB_N 512
static int16_t  bbIqT[BB_N], bbIqM[BB_N], bbMag[BB_N], bbVel[BB_N], bbVq[BB_N];
// Vd e id MEDIDO: sem eles o registro nao distingue "a malha de d esta
// bombeando" de "a comutacao esta errada" — os dois dao |I| alto e ruido.
static int16_t  bbVd[BB_N], bbId[BB_N];
static uint8_t  bbSec[BB_N];
// Colunas CRUAS (ver phaseCurrentRawDbg): leitura−zeroCal e zero-vivo−zeroCal
// em LSB + amostras aceitas/ms por canal. É o que separa, no registro da
// falha, buraco de amostragem / passeio de zero / corrupção de conversão /
// lixo de transformada — sem precisar deduzir por sintoma.
static int16_t  bbC1[BB_N], bbC2[BB_N];
static int8_t   bbZ1[BB_N], bbZ2[BB_N];
static uint8_t  bbA1[BB_N], bbA2[BB_N];
static volatile uint16_t bbIdx    = 0;
static volatile uint8_t  bbFrozen = 0;
static uint16_t          bbFaults = 0;
static volatile bool     crashDumpReq = false;
#endif

// --- captura de 1 kHz p/ step response (BULK descarrega depois) --------------
#define CAP_N 1500
static int16_t capTgt[CAP_N], capVel[CAP_N], capCur[CAP_N];   // 9 KB
static volatile uint16_t capIdx = 0;
static volatile uint8_t  capState = 0;    // 0 livre | 1 capturando | 2 pronto
static float capAmpRads = 0, capBase = 0;
static float fwIdA        = 0;   // [A] magnitude ATUAL do id de field weakening
static float s_fwIdTarget = 0;   // [A] alvo entregue à malha de d (= −fwIdA)

// --- MTPA por BUSCA (mínimo de corrente) -------------------------------------
// SPM (Ld~Lq): MTPA = id 0 = comutação exatamente 90° à frente do fluxo.
// Sem corrente por fase, o jeito REAL de garantir: em cruzeiro estável,
// perturbar o ângulo em ±0,3° e caminhar na direção que REDUZ a corrente DC
// na mesma velocidade (perturb & observe). O trim aprendido (±5,7° máx)
// corrige erro residual de calibração e persiste em NVS.
static float    mtpaTrim    = 0;      // [rad] trim aprendido (NVS)
static float    mtpaExplore = 0;      // [rad] perturbação da fase atual
static bool     mtpaDirty   = false;
static uint8_t  mtpaPhase   = 0;      // 0 base | 1 +δ | 2 −δ
static uint8_t  mtpaSecs    = 0;
static float    mtpaAcc[3]  = {0, 0, 0};
static float    sIsum = 0, sVmin = 0, sVmax = 0;
static uint8_t  ovrNow = 0;

// --- ECO AUTOMÁTICO (modo ECO + torque) --------------------------------------
static uint8_t  ecoState    = 0;      // 0 fora | 1 ativo
static uint16_t ecoSteadyMs = 0;
static float    ecoVRef = 0, ecoV = 0, ecoThrRef = 0;
static float    ecoVelEma = 0, ecoThrEma = 0;
// COSTEIRA (soltura no modo torque): regula a tensão até a corrente de
// bateria ir a ZERO (roda solta). coastActive=false quando acelera/freia.
static bool     coastActive = false;

// O estado interno do PIDController da SimpleFOC é protected; esta subclasse
// (nunca instanciada — só um "óculos" via cast) permite semear o integrador
// para transferência torque->velocidade SEM SALTO na tensão (bumpless).
struct PIDSeed : public PIDController {
  void seedOutput(float v) {
    error_prev = 0; integral_prev = v; output_prev = v;
    timestamp_prev = _micros();
  }
};
static uint32_t sN = 0;
#define MTPA_DELTA   0.0052f    // 0,3° elétricos de exploração
#define MTPA_STEP    0.0026f    // 0,15° por decisão
#define MTPA_MAX     0.10f      // ±5,7° de trim total
static volatile bool iphZeroReq  = false;   // pedido de re-zero dos INA240
static volatile bool iphCalReq   = false;   // pedido de CALIBRAÇÃO da corrente
static volatile bool iphDiagReq  = false;   // pedido de despejo cru p/ o serial
static volatile bool iStepReq    = false;   // pedido de degrau da malha de corrente
static volatile float iStepAmps  = 3.0f;
static volatile bool  spinReq    = false;   // teste girando (roda livre)
static volatile float spinAmps   = 3.0f;
static volatile float spinKmh    = SPIN_TEST_KMH;
static volatile bool  angScanReq = false;   // scan de ângulo sob rotação
static volatile float angScanA   = 3.0f;
static volatile float angScanKmh = 6.0f;
static volatile bool  s_sliceResync = false; // rotina bloqueante acabou
static volatile bool  sweepReq   = false;   // varredura em rampa (roda livre)
static volatile float sweepKmh   = 20.0f;
static volatile float sweepA     = 5.0f;
static volatile bool  iStepOpen  = false;  // degrau de tensão em vez de corrente
static volatile bool s_cruiseLive = false;   // cruise realmente ativo (fatia)      // maior "buraco" entre iterações do loop
                                        // no último segundo (task de controle
                                        // sendo bloqueada aparece aqui)
static uint16_t faults        = 0;      // faltas travadas (FAULT_*)
static uint32_t lastClearReq  = 0;      // último clear_req processado
static uint8_t  curMode       = MODE_TORQUE;
static float    ocMs = 0, uvMs = 0, otMs = 0;   // acumuladores dos trips [ms]
static uint32_t sliceCount    = 0;
static uint32_t loopHz        = 0;      // frequência real do loopFOC

// ---------------------------------------------------------------------------
// Conversões e utilitários
// ---------------------------------------------------------------------------
static inline float sgnf(float v)          { return (v > 0) - (v < 0); }
static inline float wheelRadM()            { return prm(P_WHEEL_MM) * 0.0005f; }
static inline float kmhToRads(float kmh)   { return kmh / 3.6f / wheelRadM(); }
static inline float radsToKmh(float rads)  { return rads * wheelRadM() * 3.6f; }

// Rampa: aumenta |alvo| com rateUp, reduz/inverte com rateDown (mais rápido —
// soltar o acelerador deve responder rápido).
static float slewLimit(float current, float target, float rateUp, float rateDown, float dt) {
  bool increasing = (fabsf(target) > fabsf(current)) && (current * target >= 0);
  float maxStep = (increasing ? rateUp : rateDown) * dt;
  float d = target - current;
  if (d >  maxStep) return current + maxStep;
  if (d < -maxStep) return current - maxStep;
  return target;
}

// Corrente (INA226), tensão e NTC vêm da task de sensores no núcleo 0
// (ina226CurrentA/sensVbatV/sensTempC): analogRead/I2C têm seção crítica
// interna e no núcleo 1 atrasavam as ISRs dos Halls (skips em baixa rotação).

#if HALL_DIAG
// Diagnóstico dos Halls a 1 kHz (bits, reconciliação, taxas, logs) —
// compartilhado entre o modo normal (controlSlice) e o modo bancada.
static void hallDiagTick() {
  // Lê os 3 pinos direto e acompanha: estado atual, quais dos 6 estados
  // válidos já apareceram e quantas vezes caiu em estado inválido (000/111).
  // TESTE DE BANCADA: motor DESLIGADO, gire a roda BEM devagar com a mão —
  // o card "Halls" da dashboard deve alternar os 3 bits e chegar a 6/6
  // estados; "ruído" deve ficar em 0. Um bit que nunca muda = fio/canal
  // morto (causa clássica de "trava e só sai com empurrão").
  {
    uint8_t hb = (uint8_t)((digitalRead(PIN_HALL_A) << 2) |
                           (digitalRead(PIN_HALL_B) << 1) |
                            digitalRead(PIN_HALL_C));
    hallBitsNow = hb;
    if (hb != 0 && hb != 7)
      hallSeenMask |= (uint8_t)(1 << (hb - 1));    // estados 1..6 -> bits 0..5

#if !OPEN_LOOP_TEST
    // RECONCILIAÇÃO com confirmação em 2 fatias: em alta rotação chega uma
    // borda a cada ~1 ms, então ler os pinos e comparar com hall_state SEM
    // atomicidade gera divergência falsa (a borda cai no meio da checagem).
    // Uma divergência só é tratada se persistir por 2 fatias seguidas com a
    // MESMA leitura — só um estado realmente preso faz isso (corrigido em
    // ~2 ms). Girando, divergência transitória se resolve sozinha e não conta.
    // Só age com leitura VÁLIDA (1..6) p/ nunca injetar 000/111 na lib.
    {
      static uint8_t mismatchPrevHb = 0xFF;
      if (hb != 0 && hb != 7 && hb != (uint8_t)hallSensor.hall_state) {
        if (hb == mismatchPrevHb) {           // persistiu: estado preso de verdade
          noInterrupts();
          hallSensor.handleA(); hallSensor.handleB(); hallSensor.handleC();
          interrupts();
          hallResyncs++;
          mismatchPrevHb = 0xFF;
        } else {
          mismatchPrevHb = hb;                // 1ª vez: só marca e espera confirmar
        }
      } else {
        mismatchPrevHb = 0xFF;
      }
    }
#endif

    // Bordas cruas por segundo (ANTES do filtro): com a roda PARADA e motor
    // ligado deve ser 0 — se contar, é ruído induzido do PWM nos cabos.
    // Também calcula a taxa de bordas ACEITAS por pino: A, B e C devem ser
    // iguais (±1) — um pino fora dos outros = aquele canal falhando.
    if (sliceCount % 1000 == 0) {
      uint32_t ec = hallEdgeCount;
      hallEps = (uint16_t)min((uint32_t)65535, ec - hallEdgeSnap);
      hallEdgeSnap = ec;
      static uint32_t snapA = 0, snapB = 0, snapC = 0;
      uint32_t a = edgeCntA, b = edgeCntB, c = edgeCntC;
      epsA = (uint16_t)min((uint32_t)65535, a - snapA); snapA = a;
      epsB = (uint16_t)min((uint32_t)65535, b - snapB); snapB = b;
      epsC = (uint16_t)min((uint32_t)65535, c - snapC); snapC = c;
    }

    // Log serial na hora em que a sequência quebra (p/ correlacionar com o
    // tranco físico). Limitado a 1 linha a cada 250 ms para não inundar.
    {
      static uint32_t lastSk = 0, lastRv = 0, lastIv = 0, lastPrintMs = 0;
      uint32_t sk = seqSkips, rv = seqRevs, iv = seqInvalid;
      if ((sk != lastSk || rv != lastRv || iv != lastIv) &&
          millis() - lastPrintMs > 250) {
        mcPrintf("[hall] SEQ skip=%lu(+%lu) rev=%lu inv=%lu @%.1fkm/h eps=%u rejF=%u/%u/%u rejC=%u/%u/%u\n",
                      sk, sk - lastSk, rv, iv, (double)radsToKmh(velEst),
                      hallEps, rejFiltA, rejFiltB, rejFiltC,
                      rejConfA, rejConfB, rejConfC);
        lastSk = sk; lastRv = rv; lastIv = iv; lastPrintMs = millis();
      }
    }

    // Com o motor desabilitado, loga cada transição no serial (teste manual)
    if (hb != hallBitsLast && !motor.enabled) {
      mcPrintf("[hall] A=%d B=%d C=%d  (vistos %d/6, resyncs %u)\n",
                    (hb >> 2) & 1, (hb >> 1) & 1, hb & 1,
                    __builtin_popcount(hallSeenMask), hallResyncs);
    }
    hallBitsLast = hb;
  }
}

#endif  // HALL_DIAG

// Estimador próprio de velocidade: deriva o ângulo do sensor a 1 kHz e filtra
// com τ ~80 ms (o getVelocity() dos Halls zera entre pulsos em baixa rotação).
static void updateVelEst(float dt) {
  float velPhysMax = kmhToRads(MAX_KMH_ABS) * 1.5f;
  float ang = motor.shaft_angle;      // atualizado pelo move() mesmo desabilitado
  if (!prevAngleInit) { prevAngle = ang; prevAngleInit = true; }
  float velRaw = constrain((ang - prevAngle) / dt, -velPhysMax, velPhysMax);
  prevAngle = ang;
  // τ 40 ms (era 80): o filtro foi dimensionado para o ângulo em DEGRAUS de
  // 60°; com o Hall interpolado o ângulo é suave e 80 ms só sobravam como
  // atraso — um polo a 12,5 rad/s em cima do cruzamento da malha de
  // velocidade, comendo a margem de fase (era um dos três lags dos "bumps"
  // de rampa na bancada). 40 ms mantém a suavidade de exibição.
  velEst += (dt / 0.04f) * (velRaw - velEst);
}

// ---------------------------------------------------------------------------
// VELOCIDADE RÁPIDA — direto do intervalo entre bordas de Hall
// ---------------------------------------------------------------------------
// velEst tem filtro de 80 ms. Para exibir e para as proteções lentas, ótimo.
// Para o LIMITADOR DE VELOCIDADE, cego: uma roda solta com 3 A acelera a ~100
// rad/s² e chega aos 4 km/h em menos de 100 ms — dentro da própria constante de
// tempo do filtro. O limitador enxergava uma velocidade que ainda não tinha
// subido, mantinha o teto aberto, e quando o número finalmente chegava a roda
// já estava no dobro. É por isso que "com roda solta dispara" e no chão não:
// com o piloto em cima a aceleração é 10-50x menor e 80 ms não custam nada.
//
// O intervalo entre bordas de Hall dá a velocidade com atraso de UM setor
// (~7 ms a 4 km/h) e sem filtro nenhum. É assim que as controladoras de
// prateleira medem velocidade, e é o sinal certo para quem precisa fechar um
// teto a tempo. Desacelerando, o tempo desde a última borda já passa do período
// medido: usar o MAIOR dos dois faz a leitura cair sozinha em vez de ficar
// presa no último valor.
static float velFastRads() {
  uint32_t p[3], last;
  noInterrupts();
  p[0] = s_hallPer[0]; p[1] = s_hallPer[1]; p[2] = s_hallPer[2];
  last = s_lastHallUs;
  interrupts();
  // mediana de 3: descarta o intervalo isolado (borda falsa ou perdida)
  const uint32_t lo = min(p[0], p[1]), hi = max(p[0], p[1]);
  const uint32_t per = max(lo, min(hi, p[2]));
  if (per == 0) return velEst;                    // ainda sem histórico
  const uint32_t el = micros() - last;
  const float t = (float)((el > per) ? el : per) * 1e-6f;
  float w = (PI / 3.0f) / t / (float)MOTOR_POLE_PAIRS;
  w = fminf(w, kmhToRads(MAX_KMH_ABS) * 1.5f);    // teto físico, nunca absurdo
  return (velEst >= 0.0f) ? w : -w;
}

#if PHASE_CUR_ENABLED && IPH_DIAG_TOOLS
// Grava um quadro da caixa-preta (1 kHz) e congela na 1ª falta que trava.
static void bbTick() {
  if (bbFrozen) return;
  if (faults & FAULT_LATCHING_MASK) { bbFrozen = 1; bbFaults = faults; return; }
  const uint16_t i = bbIdx;
  bbIqT[i] = (int16_t)constrain(s_iqTarget * 100.0f, -32000.0f, 32000.0f);
  bbIqM[i] = (int16_t)constrain(s_iph.iq   * 100.0f, -32000.0f, 32000.0f);
  bbMag[i] = (int16_t)constrain(s_iph.imag * 100.0f, -32000.0f, 32000.0f);
  bbVel[i] = (int16_t)constrain(radsToKmh(velFastRads()) * 100.0f,
                                -32000.0f, 32000.0f);
  bbVq[i]  = (int16_t)constrain(motor.voltage.q * 100.0f, -32000.0f, 32000.0f);
  bbVd[i]  = (int16_t)constrain(s_udCmd      * 100.0f, -32000.0f, 32000.0f);
  bbId[i]  = (int16_t)constrain(s_iph.id     * 100.0f, -32000.0f, 32000.0f);
  bbSec[i] = (uint8_t)hallSensor.electric_sector;
  {
    int16_t dev[2]; int8_t zo[2]; uint8_t ac[2];
    phaseCurrentRawDbg(dev, zo, ac);
    bbC1[i] = dev[0]; bbC2[i] = dev[1];
    bbZ1[i] = zo[0];  bbZ2[i] = zo[1];
    bbA1[i] = ac[0];  bbA2[i] = ac[1];
  }
  bbIdx = (uint16_t)((i + 1) % BB_N);
}

// Despejo da caixa-preta (bloqueante — só com o motor desabilitado).
static void bbDump() {
  if (!bbFrozen) {
    Serial.println("[bb] nada congelado (nenhuma falta travou desde o ultimo "
                   "despejo). Reproduza a falha e rode 'crash' de novo.");
    return;
  }
  const PhaseCal *pc = phaseCurrentCal();
  Serial.printf("[bb] === CAIXA-PRETA: falta 0x%02X | 512 ms antes do corte "
                "(mais antigo primeiro) ===\n", bbFaults);
  // Carimbo do firmware NO PRÓPRIO despejo: um registro de build velho já
  // custou rodadas de diagnóstico — o despejo tem que se datar sozinho.
  Serial.printf("[bb] build %s %s | duty op %.3f | escala %.0f/%.0f A/V\n",
                __DATE__, __TIME__, phaseCurrentDutyOp(),
                (double)(pc->ampsPerVolt[0]), (double)(pc->ampsPerVolt[1]));
  Serial.printf("[bb] zeros agora %.4f/%.4f V (cal %.4f/%.4f) | pinned=%d "
                "clip=%d fresh=%d\n",
                pc->offset[0], pc->offset[1], pc->zeroRef[0], pc->zeroRef[1],
                (int)phaseCurrentZeroPinned(), (int)phaseCurrentClipping(),
                (int)phaseCurrentFresh());
  Serial.println("[bb] c1/c2 = leitura-zeroCal [LSB] | z1/z2 = zeroVivo-zeroCal "
                 "[LSB] | a1/a2 = amostras/ms (sao ~8)");
  Serial.println("[bb]   ms  iqAlvo  iqMed  idMed    |I|    km/h     Vq     Vd "
                 "st |    c1    c2  z1  z2 a1 a2");
  for (int n = 0; n < BB_N; n++) {
    const uint16_t i = (uint16_t)((bbIdx + n) % BB_N);
    Serial.printf("[bb] %4d %+7.2f %+7.2f %+6.2f %6.2f %7.2f %+7.2f %+6.2f "
                  "%d | %+5d %+5d %+3d %+3d %2u %2u\n",
                  n - BB_N, bbIqT[i] / 100.0f, bbIqM[i] / 100.0f,
                  bbId[i] / 100.0f, bbMag[i] / 100.0f, bbVel[i] / 100.0f,
                  bbVq[i] / 100.0f, bbVd[i] / 100.0f,
                  (int)bbSec[i], (int)bbC1[i], (int)bbC2[i],
                  (int)bbZ1[i], (int)bbZ2[i], (unsigned)bbA1[i],
                  (unsigned)bbA2[i]);
  }
  Serial.println("[bb] === fim (gravador re-armado) ===");
  bbFrozen = 0;
}
#endif

// ---------------------------------------------------------------------------
// Publica a telemetria para o núcleo 0 (dashboard)
// ---------------------------------------------------------------------------
static void publishTelemetry(const Command &cmd, bool linkOk, float scale) {
  Telemetry t;
  float vel     = velEst;   // estimador próprio: estável mesmo em baixa rotação
  t.kmh         = radsToKmh(vel);
  t.ibat_a      = iBatDisp;      // exibição calma; proteções usam iBatFilt
  t.vbat_v      = vBatFilt;
  t.temp_c      = tempDisp;      // idem (tempFilt segue nas proteções)
  t.motor_temp_c = motorTempDisp;   // temp interna do motor (NTC no GPIO26)
  t.target      = appliedTarget;
  t.power_scale = scale;
  t.i_limit_a   = prm(P_ILIM_A);   // limites agora são parâmetros
  t.max_kmh     = (s_vmaxEffKmh > 0.5f) ? s_vmaxEffKmh : prm(P_MAX_KMH);
  t.v_limit     = prm(P_VLIM);
  t.loop_hz     = loopHz;
  t.mode        = curMode;
  t.enabled     = motor.enabled;
  t.faults      = faults
                | (cmd.estop ? FAULT_ESTOP : 0)
                | (!linkOk   ? FAULT_LINK  : 0)
                | (uvWarn    ? FAULT_UV_WARN : 0);
  // energia / instrumentos
  t.odo_m        = odoM;
  t.trip_m       = tripM;
  t.trip_wh      = tripWh;
  t.trip_wh_regen= tripWhRegen;
  // Autonomia é derivada NO APP a partir de: consumo recente (EWMA),
  // capacidade aprendida e SoC inteligente — enviados abaixo.
  t.wh_per_km = whkmRecent;
  t.cap_wh    = (uint16_t)constrain(capLearned, 0.0f, 3000.0f);
  t.soc_pct   = (socLive >= 0) ? (uint8_t)(socLive * 100.0f + 0.5f) : 0;
  t.fw_x2     = (uint8_t)constrain(fwIdA * 2.0f, 0.0f, 255.0f);  // agora id [A]×2
  t.mtpa_x10  = (int8_t)constrain(mtpaTrim * 57.2958f * 10.0f, -127.0f, 127.0f);
  t.eco_auto  = (ecoState == 1);
  // --- CORE em tempo real (data-log profundo) ---
  t.vel_est      = velEst;
  t.v_q          = motor.voltage.q;
  t.v_d          = motor.voltage.d;
  t.i_raw        = fsIRaw;
  t.eff_v_limit  = fsEffVLim;
#if PHASE_CUR_ENABLED
  t.iph_a         = s_iph.ia;
  t.iph_b         = s_iph.ib;
  t.iph_c         = s_iph.ic;
  t.iph_mag       = s_iphMagDisp;
  t.iph_d         = s_iph.id;
  t.iph_q         = s_iph.iq;
  t.iq_target     = s_iqTarget;
  // Duty de OPERAÇÃO (não o físico): é ele que define o teto de tensão e,
  // portanto, a velocidade de topo. Mostrar o físico dava um número que a
  // malha nunca alcança.
  t.duty_max_pct = (uint8_t)constrain(phaseCurrentDutyOp() * 100.0f, 0.0f, 100.0f);
  t.iph_ok        = s_iphOk;
  {
    const PhaseCal *pc = phaseCurrentCal();
    t.iph_ref   = (uint8_t)((pc->ref[0] & 3) | ((pc->ref[1] & 3) << 2));
    t.iph_zero1 = pc->offset[0];
    t.iph_zero2 = pc->offset[1];
  }
#else
  t.iph_a = t.iph_b = t.iph_c = 0; t.iph_mag = 0;
  t.iph_d = t.iph_q = t.iq_target = 0;
  t.duty_max_pct = 0; t.iph_ok = false;
  t.iph_ref = 3; t.iph_zero1 = 0; t.iph_zero2 = 0;
#endif
  t.temp_scale   = fsTempScale;
  t.tracv_scale  = fsTracVScale;
  t.elec_sector  = hallSensor.electric_sector;
  t.ovr          = ovrNow;
  t.coast_active = coastActive;
  t.throttle     = cmd.throttle;      // ENTRADA do piloto (crua)
  t.brake        = cmd.brake;
  t.thr_eff      = fsThrEff;          // após a curva
  t.brk_eff      = fsBrkEff;
  t.vel_target   = velTarget;
  t.elec_angle   = motor.electrical_angle;
  t.dc_a         = driver.dc_a;
  t.dc_b         = driver.dc_b;
  t.dc_c         = driver.dc_c;
  t.shaft_angle  = motor.shaft_angle;
  t.shaft_vel    = motor.shaft_velocity;
  t.pscale_fold  = powerScale;                 // foldback isolado
  t.eco_v        = ecoV;
  t.regen_ilim   = fsRegenILim;
  t.ms_since_cmd = (uint16_t)min((uint32_t)65535, (uint32_t)(millis() - cmd.last_cmd_ms));
  t.duty_pct     = fsDuty;
  t.headroom_pct = (uint8_t)constrain(scale * 100.0f, 0.0f, 100.0f);
  t.ride         = cmd.ride;
  t.headlight    = cmd.headlight;
  t.saturated    = fsDuty >= 95;
  t.cruise       = s_cruiseLive;
  t.walk         = cmd.walk;
  t.locked       = cmd.locked;
  {
    uint32_t stMs = s_cmdStatusMs;
    t.cmd_status = (s_cmdStatus && millis() - stMs < 3000) ? s_cmdStatus : 0;
  }
  t.hall_bits   = hallBitsNow;
  t.hall_seen   = hallSeenMask;
  t.hall_bad    = hallResyncs;   // ressincronizações (HALL_DIAG); trava removida
  t.hall_eps    = hallEps;
  t.eps_a       = epsA;
  t.eps_b       = epsB;
  t.eps_c       = epsC;
  t.seq_skips   = (uint16_t)min((uint32_t)65535, (uint32_t)seqSkips);
  t.seq_revs    = (uint16_t)min((uint32_t)65535, (uint32_t)seqRevs);
  t.seq_inval   = (uint16_t)min((uint32_t)65535, (uint32_t)seqInvalid);
  t.rej_conf_a  = rejConfA;
  t.rej_conf_b  = rejConfB;
  t.rej_conf_c  = rejConfC;
  t.rej_filt_a  = rejFiltA;
  t.rej_filt_b  = rejFiltB;
  t.rej_filt_c  = rejFiltC;
  t.link_drops  = linkDrops;
  t.loop_gap_us = loopGapUs;
  t.zea         = motor.zero_electric_angle;
  writeTelemetry(t);
}

// ---------------------------------------------------------------------------
// REGULADOR DE CORRENTE — uma malha só, saída em TENSÃO
// ---------------------------------------------------------------------------
// A comutação continua sendo a do modo tensão (setPhaseVoltage(Uq, 0, ângulo)),
// que é a que este motor comprovadamente executa bem. O que muda é a origem do
// Uq: em vez de vir do punho, vem de um PI que persegue a corrente MEDIDA.
//
// POR QUE NÃO O foc_current DO SimpleFOC: ele decompõe a corrente em Id/Iq pelo
// ângulo do Hall e fecha DUAS malhas. Com 60° de quantização, o Id "medido"
// parado é até 50% do módulo e é quase todo artefato do sensor — fechar malha
// sobre ele faz o Vd girar o vetor de tensão e o motor trancar. O Iq erra no
// máximo 13% (é cosseno do desvio), que é gain error tolerável numa malha só.
//
// Anti-windup: o integrador é limitado ao mesmo teto da saída, então saturar
// não acumula dívida para descarregar depois.
static float curRegInt = 0.0f;          // integrador [V]

// MALHA DO EIXO D (id -> 0 via Ud) — ver UD_MAX_FRAC no config.h. Espelho da
// malha de q: mesma planta R-L, mesmos ganhos, mesma zona morta no
// integrador. Só roda no caminho NORMAL da fatia (runCurrentLoops); as
// rotinas bloqueantes de calibração/teste continuam em q puro com Ud=0 —
// o move() zera voltage.d a cada iteração e lá ninguém reinjeta.
static float curRegIntD = 0.0f;

static float dAxisRegulator(float idTarget, float idMeas, float udMax, float dt) {
  // Alvo normalmente 0 (mata o id residual de desalinhamento); o FIELD
  // WEAKENING escreve alvo NEGATIVO aqui — id de enfraquecimento preciso,
  // independente do torque, medido e coberto pelos trips de |I|.
  const float err  = idTarget - idMeas;
  const float db   = prm(P_CUR_DB);
  const float errI = err - constrain(err, -db, db);
  curRegIntD += prm(P_CUR_I) * errI * dt;
  curRegIntD  = constrain(curRegIntD, -udMax, udMax);
  return constrain(prm(P_CUR_P) * err + curRegIntD, -udMax, udMax);
}

static float currentRegulator(float iTarget, float iMeas, float vLimit, float dt) {
  const float err = iTarget - iMeas;
  // Só a parte do erro que EXCEDE o ruído da medição alimenta o integrador.
  // Erro menor que a incerteza não é erro, é ruído — e integrá-lo faz o
  // integrador passear e virar corrente real com o punho em zero (ver
  // P_CUR_DB). A subtração é contínua, sem degrau na borda.
  const float db   = prm(P_CUR_DB);
  const float errI = err - constrain(err, -db, db);
  curRegInt += prm(P_CUR_I) * errI * dt;
  curRegInt = constrain(curRegInt, -vLimit, vLimit);
  // O proporcional continua vendo o erro inteiro: ele não acumula, então ruído
  // nele só vira um tremor de tensão proporcional (0,6 A x 0,10 = 60 mV), que
  // com as fases curtas ao GND não move nada.
  return constrain(prm(P_CUR_P) * err + curRegInt, -vLimit, vLimit);
}
static void currentRegulatorReset(float v) {
  curRegInt  = constrain(v, -60.0f, 60.0f);
  curRegIntD = 0.0f;
  s_udCmd    = 0.0f;
  fwIdA = 0.0f; s_fwIdTarget = 0.0f;   // FW nunca sobrevive a um reset
}

// As duas malhas juntas, com ORÇAMENTO VETORIAL e PRIORIDADE ABSOLUTA DO Q:
// |U| = sqrt(Uq²+Ud²) nunca passa do teto, e o d recebe só a SOBRA depois do
// q. A versão anterior reservava 35% ao d mesmo no teto de modulação — a
// caixa-preta pegou o resultado a 24 km/h: id espetado por amostra marginal
// -> Ud bombeia -> uqMax desaba -> Vq leva tranco de 2-3 V por fatia com
// V≈BEMF -> ±10 A REAIS por tranco -> mais caos -> escalada até 45-57 A e
// trip. Com o q prioritário, no teto a sobra vai a zero, a malha de d SOLTA
// (o constrain do integrador dela ao udMax dinâmico a esvazia sozinho) e o
// topo de velocidade volta a ser q puro e estável. Abaixo do teto nada muda
// (sobra >> necessidade do d).
static float runCurrentLoops(float iqTarget, float dt, float vLimit) {
  // PRIORIDADE DO EIXO D (a caixa-preta crua de 28/ago fechou o diagnóstico
  // do "trava nos ~20 km/h"): existe uma inclinação REAL do vetor aplicado
  // proporcional à velocidade (atraso de comutação/latência — é o mesmo id
  // que media 8 A a 10 km/h antes da malha de d). Abaixo do teto, a malha de
  // d a cancelava em silêncio; no teto, a versão anterior ("prioridade
  // absoluta do q") dava ao d só a SOBRA do orçamento — que no teto é ZERO —
  // e a inclinação ficava sem compensação: id real explodia a −25/−30 A
  // (c1/c2 = ±150-200 LSB REAIS na frequência elétrica, com barramento
  // parado = corrente reativa), o iq medido via o vazamento negativo do id
  // (−25·sin~10° ≈ −4 A), o PI cravava Vq no teto e o sistema travava —
  // em QUALQUER teto (por isso o defeito "seguia o teto" build após build).
  // Padrão da indústria (ST/TI): o d toma o que precisa — é pouco, e é o
  // que mantém o vetor são — e o Q fica com a sobra. Custo no teto: só o
  // ud realmente usado (p.ex. 4 V de ud em 16,4 V de teto = q perde 0,5 V).
  // udMax constante também elimina o bater do clamp do d que a sobra
  // flutuante causava.
  const float udMax = prm(P_UD_MAX) * vLimit;   // 0 = malha de d desligada
  s_udCmd = dAxisRegulator(s_fwIdTarget, s_iph.id, udMax, dt);
  const float uqMax = sqrtf(fmaxf(vLimit * vLimit - s_udCmd * s_udCmd, 0.0f));
  return currentRegulator(iqTarget, s_iph.iq, uqMax, dt);
}

// ---------------------------------------------------------------------------
// Fatia de 1 kHz: ADCs, proteções, comando e cálculo do alvo
// ---------------------------------------------------------------------------
static void controlSlice(float dt) {
  sliceCount++;

  hallRateTick();               // bordas/s dos halls (sempre, p/ data-log)
#if HALL_DIAG
  hallDiagTick();
#endif

  // ---- 0) CORRENTE DE FASE (INA240 low-side) -----------------------------
  // Primeira coisa da fatia: é a realimentação da malha de torque e a base de
  // todas as proteções de fase.
#if PHASE_CUR_ENABLED
  // Toda amostra é medição real: o teto de modulação garante a janela, e cada
  // conversão ainda é validada contra o compare do PWM no instante do disparo
  // (ver phase_current.h) — amostra fora da janela nunca entra aqui. O iq/id
  // desta struct já vêm FILTRADOS em dq (P_CUR_TF); ia/ib/ic e |I| são crus.
  // Projeção dq com o ângulo DO INSTANTE DA AMOSTRA: as amostras têm idade
  // média IPH_MEAS_AGE_US e o sensor reporta o ângulo AVANÇADO de
  // P_COMM_LEAD — os dois somam rotação espúria ω·τ na medição (era o
  // vazamento id→iq de ~−4 A dos registros do topo).
  phaseCurrentRead(&s_iph, motor.electrical_angle -
                   motor.shaft_velocity * (float)MOTOR_POLE_PAIRS *
                   ((s_commLeadUs + IPH_MEAS_AGE_US) * 1e-6f));
  s_iphMagFilt += 0.3f * (s_iph.imag - s_iphMagFilt);        // τ~3 ms
  s_iphMagDisp += (dt / IPH_DISP_TAU_S) * (s_iph.imag - s_iphMagDisp);

  // RASTREIO DO ZERO: só com o motor desabilitado E a roda REALMENTE parada,
  // que é o único estado em que a corrente é comprovadamente nula. Fora dele o
  // offset fica congelado — rastrear com corrente real presente é "aprender" a
  // corrente como se fosse zero.
  //
  // "Parada" é medida pelas BORDAS DE HALL, não pela velocidade estimada. O
  // portão anterior (< 0,5 km/h) parecia conservador e não era: desabilitar o
  // motor põe as três fases em curto, e a roda descendo por esse curto empurra
  // corrente real pelos shunts — a 0,5 km/h ainda são ~2 A. O rastreador
  // caminhava para esse valor a 32 mV/s e em pouco mais de um segundo encostava
  // no limite de deriva, deixando ampères fantasmas no lugar do zero. Foi o que
  // matou a segunda passada do A/B: 2,61 A medidos com a roda imóvel, o
  // regulador se dando por satisfeito e o motor sem torque nenhum.
  // Isso NÃO era só da bancada: acontecia a cada parada com a roda ainda
  // girando, ou seja, no uso normal.
  {
    static uint32_t stillMs = 0, lastEdge = 0;
    const uint32_t e = hallEdgeCount;
    if (e != lastEdge)          { lastEdge = e; stillMs = 0; }
    else if (stillMs < 60000)   { stillMs += (uint32_t)(dt * 1000.0f + 0.5f); }
    s_wheelStill = (stillMs >= IPH_ZERO_STILL_MS);
  }
  if (!motor.enabled && s_wheelStill) phaseCurrentTrackZero(dt, false);
#endif

  // ---- 1) Leituras (escalonadas p/ não pesar na fatia) -------------------
#if CUR_SENSE_ENABLED
  // Corrente do INA226: toda fatia (1 kHz) — é a proteção mais crítica.
  float iRaw = ina226CurrentA();   // lido a ~1 kHz pela task do núcleo 0
  iBatFilt += 0.2f * (iRaw - iBatFilt);            // rápido: proteções (τ~5 ms)
  fsIRaw = iRaw;                                   // corrente CRUA p/ data-log
  iBatDisp += (dt / IBAT_DISP_TAU_S) * (iRaw - iBatDisp);   // lento: display
#else
  // Shunt desativado (CUR_SENSE_ENABLED 0): sem leitura — fica em 0 A.
  float iRaw = 0.0f;
#endif

  // Tensão e temperatura: a cada 25 ms, intercaladas (mudam devagar).
#if VBAT_SENSE_ENABLED
  if (sliceCount % 25 == 0) {
    float v = sensVbatV();
    // 1ª leitura válida assenta o filtro DIRETO (sem rampa desde 0 — a rampa
    // lenta cruzava a faixa de subtensão por ~2,5 s e travava falta no boot)
    if (vBatFilt < 5.0f && v > 5.0f) vBatFilt = v;
    else                             vBatFilt += VBAT_LPF_ALPHA * (v - vBatFilt);
    // O NORMALIZADOR DE DUTY SEGUE A BATERIA REAL. A SimpleFOC converte
    // tensão em duty dividindo por voltage_power_supply; fixado em 36 V,
    // com o pack CHEIO (41,5 V) o teto de tensão (Vbat_real x 0,426) virava
    // duty 0,85 — acima do teto de AMOSTRAGEM (0,738): o portão rejeitava
    // ~100% das amostras e a malha ficava CEGA com tensão máxima — o caos de
    // ±60 A no topo com punho cheio que a caixa-preta registrou. Com o
    // normalizador vivo, tensão comandada = tensão real e o teto de duty é
    // respeitado em QUALQUER estado de carga. (Também conserta a escala de
    // tudo que comanda volts: cal, ang, Ke.)
    if (vBatFilt > 20.0f) driver.voltage_power_supply = vBatFilt;
#if PHASE_CUR_ENABLED
    // Retaguarda da trava do SAR (a âncora principal fica na task de
    // sensores, logo após cada rajada de ADC2 — ver phaseCurrentSarPowerAssert)
    phaseCurrentSarPowerAssert();
#endif
    s_commLeadUs = prm(P_COMM_LEAD);   // cache p/ o SmoothingSensor (loop rápido)
  }
#endif
#if NTC_SENSE_ENABLED
  if (sliceCount % 25 == 12) {
    float tc = sensTempC();
    tempFilt += 0.05f * (tc - tempFilt);                    // proteções (τ~0,5 s)
    tempDisp += (0.025f / TEMP_DISP_TAU_S) * (tc - tempDisp);   // display
  }
#endif
#if MOTOR_NTC_SENSE_ENABLED
  if (sliceCount % 25 == 18) {
    float tc = sensMotorTempC();
    motorTempFilt += 0.05f * (tc - motorTempFilt);
    motorTempDisp += (0.025f / TEMP_DISP_TAU_S) * (tc - motorTempDisp);
  }
#endif

  // ---- 2) Proteções (com filtro temporal p/ imunidade a ruído) -----------
  uint16_t before = faults;

#if CUR_SENSE_ENABLED
  // Sobrecorrente de BATERIA (INA226, leitura crua — reação rápida)
  if (fabsf(iRaw) > prm(P_OC_TRIP)) {
    ocMs += dt * 1000.0f;
    if (ocMs >= OC_TRIP_MS) faults |= FAULT_OVERCURRENT;
  } else ocMs = 0;
#else
  (void)iRaw;   // shunt desativado: sem proteção de sobrecorrente por firmware
#endif

  // SOBRECORRENTE DE FASE — a proteção que antes não existia e que os tetos de
  // tensão sobre a back-EMF tentavam imitar. Dispara pela MAGNITUDE |I|, que
  // não depende do ângulo elétrico: o Hall grosso (±30° parado) não a
  // contamina, então vale igual parado e girando.
#if PHASE_CUR_ENABLED
  if (s_iphOk) {
    const float trip = prm(P_IPH_TRIP);
    // Trip INSTANTÂNEO com confirmação de 2 fatias (2 ms): a caixa-preta
    // pegou um snapshot solitário de 37 A a 93% do teto de duty — a zona em
    // que o S&H chega a ~1 µs da borda de chaveamento e uma amostra pode
    // capturar a transição do FET. Curto REAL persiste dezenas de ms: exigir
    // 2 fatias não custa nada à proteção e imuniza contra o artefato.
    static uint8_t hardN = 0;
    if (s_iph.imag > trip * IPH_TRIP_HARD_MULT) {
      if (++hardN >= 2) {
        faults |= FAULT_OVERCURRENT;        // curto / comutação invertida
        iphMs = 0;
      }
    } else if (s_iphMagFilt > trip) {
      hardN = 0;
      iphMs += dt * 1000.0f;
      if (iphMs >= IPH_TRIP_MS) faults |= FAULT_OVERCURRENT;
    } else { iphMs = 0; hardN = 0; }
  }

  // ---- DETECTOR DE REALIMENTAÇÃO QUEBRADA (guarda anti-disparo) -----------
  // Última linha de defesa contra o modo de falha que fez o motor disparar:
  // se a malha pede torque e o que volta do sensor NÃO acompanha, a corrente
  // não está sendo medida de verdade (instante errado, canal morto, sinal
  // trocado) e o PI vai integrar até o teto. Dois sintomas, ambos fatais:
  //   a) o medido tem SINAL OPOSTO ao pedido -> realimentação positiva;
  //   b) a tensão está no teto e o medido continua muito abaixo do pedido.
  // Qualquer um dos dois trava FAULT_SENSOR (exige reboot + recalibração).
  // SATURAÇÃO: com o motor tracionando e a saída do INA240 no trilho, a
  // corrente lida é MENOR que a real e a malha enxerga folga que não existe —
  // é o caminho para o mesmo disparo do sinal invertido. Corta e trava.
  if (motor.enabled && s_iphOk && phaseCurrentClipping()) {
    clipMs += dt * 1000.0f;
    if (clipMs >= IPH_CLIP_TRIP_MS) {
      faults |= FAULT_SENSOR;
      mcPrintf("[motor] SENSOR DE FASE SATURADO (saida no trilho): a corrente "
               "real e maior que a medida. Motor CORTADO — o shunt de fase e "
               "grande demais para a faixa de corrente.\n");
    }
  } else clipMs = 0;

  // REALIMENTAÇÃO CONGELADA: sem amostra nova, a malha está CEGA — e malha
  // cega empurrando tensão é o disparo que não para. Corta em 30 ms.
  {
    static float frzMs = 0;
    if (motor.enabled && s_iphOk && !phaseCurrentFresh()) {
      frzMs += dt * 1000.0f;
      if (frzMs >= 30.0f && !(faults & FAULT_SENSOR)) {
        faults |= FAULT_SENSOR;
        mcPrintf("[motor] REALIMENTACAO DE CORRENTE CONGELADA (>3 ms sem "
                 "amostra nova): malha cega. Motor CORTADO.\n");
      }
    } else frzMs = 0;
  }


  if (motor.enabled && s_iphOk && fabsf(s_iqTarget) > 3.0f) {
    // INVERSÃO pela MÉDIA LENTA (τ~100 ms), não pelo instantâneo: o ripple
    // de setor (±5 A no topo) cruza zero o tempo todo e o teste instantâneo
    // acusava inversão em operação sã. Sensor invertido DE VERDADE dá média
    // sustentada do lado errado — cai aqui em ~100-200 ms do mesmo jeito.
    fbIqEma += 0.01f * (s_iph.iq - fbIqEma);
    bool inverted  = (s_iqTarget * fbIqEma) < 0 && fabsf(fbIqEma) > 1.5f;
    // "SEM RESPOSTA" só acusa quando HÁ folga de tensão para responder: no
    // topo de velocidade (BEMF ≈ teto) o iq fisicamente NÃO alcança o alvo —
    // Vq saturado com iq pequeno ali é operação legítima, não sensor morto
    // (caixa-preta 28/ago: alvo 20 A, iq real 1-3 A, 22,4 km/h estáveis por
    // 512 ms — e a guarda cortava com 0x08). Abaixo de ~70% da velocidade
    // de topo a folga existe e a acusação volta a valer.
    const float bemfFrac = prm(P_KE) * fabsf(velEst) /
                           fmaxf(motor.voltage_limit, 1.0f);
    bool saturated = bemfFrac < 0.7f
                     && fabsf(motor.voltage.q) > 0.95f * motor.voltage_limit
                     && fabsf(s_iph.iq) < 0.3f * fabsf(s_iqTarget);
    if (inverted || saturated) {
      fbBadMs += dt * 1000.0f;
      if (fbBadMs >= IPH_FEEDBACK_TRIP_MS) {
        faults |= FAULT_SENSOR;
        mcPrintf("[motor] REALIMENTACAO DE CORRENTE QUEBRADA (%s): "
                 "alvo %.1f A, medido %.1f A, Vq %.1f V. Motor CORTADO — "
                 "recalibre a corrente de fase.\n",
                 inverted ? "sinal invertido" : "sem resposta",
                 s_iqTarget, s_iph.iq, motor.voltage.q);
      }
    } else fbBadMs = 0;
  } else { fbBadMs = 0; fbIqEma = 0.0f; }
#endif

#if VBAT_SENSE_ENABLED
  // Subtensão (filtrada — ignora afundamento momentâneo sob carga).
  // Carência de boot: sensores/filtro ainda assentando não podem travar falta.
  if (millis() > UV_BOOT_GRACE_MS &&
      vBatFilt < prm(P_UV_CUT) && vBatFilt > 5.0f) {  // >5 V: ignora ADC desconectado no bring-up
    uvMs += dt * 1000.0f;
    if (uvMs >= UV_TRIP_MS) faults |= FAULT_UNDERVOLT;
  } else uvMs = 0;
#endif

#if NTC_SENSE_ENABLED || MOTOR_NTC_SENSE_ENABLED
  // Sobretemperatura: dispara pelo PIOR dos dois sensores (MOSFETs OU motor)
  bool overTemp = false;
#if NTC_SENSE_ENABLED
  overTemp = overTemp || (tempFilt > prm(P_T_CUT));
#endif
#if MOTOR_NTC_SENSE_ENABLED
  overTemp = overTemp || (motorTempFilt > prm(P_MT_CUT));
#endif
  if (overTemp) {
    otMs += dt * 1000.0f;
    if (otMs >= OT_TRIP_MS) faults |= FAULT_OVERTEMP;
  } else otMs = 0;
#endif

  if (faults != before) {
    mcPrintf("[motor] FALTA! bits=0x%02X (I=%.1fA V=%.1fV T=%.0fC)\n",
                  faults, iBatFilt, vBatFilt, tempFilt);
    FaultEntry fe;
    fe.odo_m = odoM; fe.uptime_s = millis() / 1000;
    fe.faults = (uint8_t)((faults & ~before) & 0xFF);   // só os bits NOVOS
    fe.ibat10 = (int16_t)(iBatFilt * 10.0f);
    fe.vbat10 = (uint16_t)constrain(vBatFilt * 10.0f, 0.0f, 65000.0f);
    fe.temp_c = (int8_t)constrain(tempFilt, -127.0f, 127.0f);
    fe.kmh10  = (int16_t)(radsToKmh(velEst) * 10.0f);
    faultLogAdd(fe);
  }

  // ---- 3) Comando vindo da dashboard --------------------------------------
  Command cmd = readCommand();

#if PHASE_CUR_ENABLED && IPH_DIAG_TOOLS
  // Teste 'gira': o banco não tem app conectado, então o teste habilita o motor
  // por conta própria — o dead-man zeraria o comando e nada giraria. E-stop e
  // faltas continuam mandando mais que o teste.
  if (s_spinPhase == SPIN_SPINUP || s_spinPhase == SPIN_CAP) {
    if (cmd.estop || (faults & FAULT_LATCHING_MASK)) {
      s_spinFail = 2; s_spinPhase = SPIN_ABORT; s_spinReqA = -1.0f;
    } else {
      cmd.enable = true; cmd.throttle = 0; cmd.brake = 0;
      cmd.last_cmd_ms = millis();          // mantém o link "vivo"
    }
  }
#endif

  // Pedido de "limpar faltas": só limpa se as condições físicas normalizaram.
  // FAULT_SENSOR (initFOC falhou) exige reboot — não limpa por aqui.
  if (cmd.clear_req != lastClearReq) {
    lastClearReq = cmd.clear_req;
#if VBAT_SENSE_ENABLED
    bool okV = (vBatFilt > prm(P_UV_RECOVER)) || (vBatFilt < 5.0f);
#else
    bool okV = true;
#endif
#if NTC_SENSE_ENABLED
    bool okT = tempFilt < (prm(P_T_CUT) - 5.0f);
#else
    bool okT = true;
#endif
    bool okI = fabsf(iBatFilt) < prm(P_OC_TRIP) * 0.5f;
    if (okV && okT && okI) faults &= FAULT_SENSOR;
  }

  // Dead-man: sem comando recente => alvos zerados (WiFi caiu, aba fechada...)
  // linkDrops conta cada ativação — se incrementar junto com um "tranquinho",
  // o solavanco é queda momentânea do WiFi, não é o motor.
  bool linkOk = (cmd.last_cmd_ms != 0) && (millis() - cmd.last_cmd_ms < CMD_TIMEOUT_MS);
  {
    static bool lastLinkOk = false;
    static bool everLinked = false;
    if (linkOk) everLinked = true;
    if (everLinked && lastLinkOk && !linkOk) linkDrops++;
    lastLinkOk = linkOk;
  }
  if (!linkOk) { cmd.throttle = 0; cmd.brake = 0; }

  // ---- 3b) Modo de pilotagem + trava/walk/cruise --------------------------
  // Cada modo aplica tetos próprios; TURBO usa os limites do app e libera FW.
  float rideIlim = I_LIMIT_MAX_A, rideVmax = MAX_KMH_ABS, rideSlew = 1.0f;
  bool  fwAllowed = true;
  // tetos de ECO/NORMAL são parâmetros (app); as rampas seguem do config
  if (cmd.ride == 0) { rideIlim = prm(P_ECO_ILIM); rideVmax = prm(P_ECO_VMAX);
                       rideSlew = RIDE_ECO_SLEW_MUL;    fwAllowed = false; }
  if (cmd.ride == 1) { rideIlim = prm(P_NOR_ILIM); rideVmax = prm(P_NOR_VMAX);
                       rideSlew = RIDE_NORMAL_SLEW_MUL; fwAllowed = false; }
  // Slider do Piloto: teto de SESSÃO POR MODO (RAM; zera no reset). O valor
  // chega como evento (one-shot), vale para o modo ATUAL e só REDUZ abaixo
  // do vmax daquele modo. Cada modo lembra o seu até o próximo boot.
  static float sessVmax[3] = { 0, 0, 0 };
  if (cmd.max_kmh > 0.5f && cmd.ride < 3) {
    sessVmax[cmd.ride] = fminf(cmd.max_kmh, rideVmax);
    clearVmaxCmd();                    // consumido (senão vazaria p/ outro modo)
  }
  if (sessVmax[cmd.ride] > 0.5f) rideVmax = fminf(rideVmax, sessVmax[cmd.ride]);

  // Trava eletrônica: SÓ engata parada (validado AQUI, não confia no app)
  static bool lockActive = false;
  if (cmd.locked) {
    if (!lockActive) {
      if (fabsf(radsToKmh(velEst)) < 1.0f) lockActive = true;
      else motorReportCmdStatus(CMDERR_NEED_STOP);
    }
  } else lockActive = false;

  // Cruise: engata na borda do comando acima de CRUISE_MIN_KMH. Freio
  // cancela SEMPRE. Acelerador só cancela depois de ter sido SOLTO uma vez
  // após o engate (re-arme) — sem isso o cruzeiro era impossível de ativar:
  // andar >5 km/h exige punho puxado, e o punho puxado cancelava na hora.
  static bool prevCruiseCmd = false, cruiseOn = false;
  static bool cruiseThrArmed = false;    // punho já voltou ao repouso?
  static float cruiseRads = 0;
  if (cmd.cruise && !prevCruiseCmd) {
    if (radsToKmh(velEst) >= CRUISE_MIN_KMH) {
      cruiseOn = true; cruiseRads = velEst;
      cruiseThrArmed = false;            // engatou com punho puxado: espera soltar
    } else {
      motorReportCmdStatus(CMDERR_BAD_STATE);
      clearCruiseCmd(); cmd.cruise = false;   // recusado: próximo clique = borda
    }
  }
  prevCruiseCmd = cmd.cruise;
  if (!cmd.cruise) cruiseOn = false;
  if (cmd.throttle < 0.03f) cruiseThrArmed = true;
  if (cruiseOn && (cmd.brake > 0.02f ||
                   (cruiseThrArmed && cmd.throttle > 0.05f))) {
    cruiseOn = false;
    clearCruiseCmd(); cmd.cruise = false;     // cancelado: idem
  }
  s_cruiseLive = cruiseOn;

  // Prioridade dos overrides (todos rodam em malha de velocidade)
  uint8_t ovr = 0;                       // 1 trava | 2 walk | 3 cruise | 4 step
  if (lockActive) {
    ovr = 1;
    // roda girando com a trava ativa = forçaram o veículo: loga (1x/s)
    static uint32_t lastBreachMs = 0;
    if (fabsf(velEst) > kmhToRads(2.0f) && millis() - lastBreachMs > 1000) {
      lastBreachMs = millis();
      mcPrintf("[trava] roda FORCADA com trava ativa!\n");
    }
  }
  else if (cmd.walk)                    ovr = 2;
  else if (cruiseOn)                    ovr = 3;
  if (capState == 1)                    ovr = 4;   // step response em curso

  // Limite de corrente efetivo (modo + walk/trava por cima do ajuste do app).
  // O ÚNICO teto de tração é a "Corrente máxima" do app (P_ILIM_A) — sem teto
  // de potência oculto.
  float iLimEff = fminf(constrain(prm(P_ILIM_A), 1.0f, I_LIMIT_MAX_A), rideIlim);
  if (ovr == 2) iLimEff = fminf(iLimEff, WALK_ILIM_A);
  if (ovr == 1) iLimEff = fminf(iLimEff, LOCK_ILIM_A);
  float vmaxKmh = fminf(constrain(prm(P_MAX_KMH), 1.0f, MAX_KMH_ABS), rideVmax);
#if PHASE_CUR_ENABLED && IPH_DIAG_TOOLS
  // Bancada: o teste anda a uma velocidade baixa e própria, não no limite do
  // modo. Reutiliza o limitador de verdade — o que se mede continua sendo o
  // sistema real, só que devagar.
  if (s_spinPhase == SPIN_SPINUP || s_spinPhase == SPIN_CAP)
    vmaxKmh = fminf(vmaxKmh, s_spinKmh);
#endif
  s_vmaxEffKmh = vmaxKmh;              // telemetria mostra o efetivo do modo

  // Curvas do acelerador/freio (LUTs validadas; ver curves.h)
  float thrEff = curveEval(CRV_THROTTLE, cmd.throttle * 100.0f) * 0.01f;
  float regenVAllow = 1.0f;
#if VBAT_SENSE_ENABLED
  // SOBRETENSÃO DE REGEN: bateria cheia não aceita carga — corta gradual
  regenVAllow = curveEval(CRV_REGEN_V, vBatFilt * 10.0f) * 0.01f;
  // Tensão máxima do pack (parâmetro): acima dela a regen corta SECO —
  // camada dura por cima da curva gradual (bateria cheia não aceita carga)
  if (vBatFilt >= prm(P_BATT_VMAX)) regenVAllow = 0.0f;
#endif
  float brkEff = curveEval(CRV_BRAKE, cmd.brake * 100.0f) * 0.01f
               * curveEval(CRV_REGEN_S, fabsf(radsToKmh(velEst))) * 0.01f
               * regenVAllow;
  fsThrEff = thrEff; fsBrkEff = brkEff;              // p/ data-log profundo

#if !OPEN_LOOP_TEST
  // Ângulo zero: base = parâmetro persistido; ajuste ao vivo pela dashboard
  // sobrepõe (e a calibração remota grava direto no parâmetro).
  {
    static float lastZea = NAN;
    if (!isnan(cmd.zea) && cmd.zea != lastZea) {
      paramSet(P_ZEA, cmd.zea);
      lastZea = cmd.zea;
      mcPrintf("[motor] zero_electric_angle = %.4f rad\n", cmd.zea);
    }
  }

  // FIELD WEAKENING por INJEÇÃO DE id — reprojetado p/ a era dos sensores de
  // fase. A versão back-EMF avançava o ÂNGULO (girava o frame inteiro via
  // zero_electric_angle): com malha de corrente isso produz id ∝ iq
  // (id_real = −iq·sen(avanço)) — em roda livre (iq 1-2 A) o enfraquecimento
  // era ~ZERO — e ainda girava o frame de MEDIÇÃO junto, com a malha de d
  // brigando contra. Agora o FW escreve um ALVO NEGATIVO de id direto na
  // malha de d (que existe e tem prioridade de orçamento): id preciso,
  // independente do torque, MEDIDO (entra em |I| e nos trips) e desfeito
  // pela própria malha ao aliviar. O joelho é % de UTILIZAÇÃO DO TETO REAL
  // (|U|/teto vivo) — imune a mudança de teto de amostragem e a estado da
  // bateria (o antigo comparava com % de duty absoluto: quando o teto de
  // operação virou 78%, o joelho de 92% ficou inalcançável e o FW morreu).
  // Só no TURBO; nunca em regen; colapsa ao soltar o punho.
  {
    const float fwMaxA = fwAllowed ? prm(P_FW_MAX_A) : 0.0f;
    if (fwMaxA > 0.05f && velEst > 1.0f && cmd.brake < 0.02f
        && cmd.throttle > 0.05f && fsEffVLim > 1.0f) {
      const float util = 100.0f * hypotf(motor.voltage.q, motor.voltage.d)
                       / fsEffVLim;               // % do teto de tensão em uso
      const float err  = util - prm(P_FW_START);  // >0 = saturando
      // sobe ~10 A em ~1,5 s de saturação plena (err≈8%); alivia 2x mais
      // rápido quando a utilização volta para baixo do joelho
      fwIdA += err * (err > 0 ? 0.8f : 1.6f) * dt;
      fwIdA  = constrain(fwIdA, 0.0f, fwMaxA);
    } else {
      fwIdA = fmaxf(0.0f, fwIdA - 20.0f * dt);    // colapso rápido
    }
    s_fwIdTarget = -fwIdA;                         // alvo da malha de d
    motor.zero_electric_angle = prm(P_ZEA) - mtpaTrim - mtpaExplore;
  }

  // Ganhos do PID de velocidade ao vivo (parâmetros do app)
  motor.PID_velocity.P  = prm(P_PID_P);
  motor.PID_velocity.I  = prm(P_PID_I);
  motor.LPF_velocity.Tf = prm(P_LPF_TF);
#endif

  uint8_t effMode = (ovr != 0) ? MODE_VELOCITY : cmd.mode;
  ovrNow = ovr;                       // visível p/ blocos de 1 Hz (MTPA etc.)

  // ---- 4) Troca de modo (com alvo zerado, por segurança) ------------------
#if !OPEN_LOOP_TEST
  if (effMode != curMode) {
    curMode = effMode;
    if (curMode == MODE_VELOCITY) {
      // Alvo parte da velocidade REAL e o integrador do PID nasce carregado
      // com a CORRENTE que o torque pedia AGORA (a saída do PID virou ampère).
      // Com reset puro a saída partia de 0 A = torque cortado seco, que era o
      // que fazia a soltura "agarrar" mesmo em malha de velocidade.
      velTarget = fmaxf(0.0f, velEst);
      static_cast<PIDSeed&>(motor.PID_velocity).seedOutput(
          constrain(s_iqTarget, 0.0f, motor.PID_velocity.limit));
    } else {
      // volta ao torque partindo da corrente que o PID pedia agora; a rampa
      // de subida (P_SLEW_UP, em A/s) assume a partir daí
      motor.PID_velocity.reset();
      velTarget = 0;
    }
    // O SimpleFOC fica sempre em TORQUE/TENSÃO: a malha de velocidade é feita
    // aqui em cascata sobre a de corrente, senão o PID de velocidade escreveria
    // tensão direto e passaria por cima do limite de corrente.
    motor.controller = MotionControlType::torque;
    mcPrintf("[motor] Modo -> %s\n", curMode == MODE_VELOCITY ? "VELOCIDADE" : "TORQUE");
  }
#endif

  // ---- 4b) CORTE POR STALL (rotor travado / comutação errada) -------------
  // Corrente de fase alta com a roda ~parada por STALL_TRIP_MS = rotor travado
  // ou comutação errada => uma fase sustentando corrente sem girar = MOSFET
  // cozinhando. TRAVA (FAULT_STALL). Antes o critério era a TENSÃO Vq acima de
  // um limiar, um proxy da corrente que errava dos dois lados (disparava em
  // subida íngreme legítima, e não disparava com comutação errada em tensão
  // baixa). Agora é a corrente MEDIDA. Exclui a TRAVA eletrônica (ovr==1), que
  // segura a roda em 0 de propósito com corrente já limitada.
  {
    static float stallMs = 0;
#if PHASE_CUR_ENABLED
    bool driving = motor.enabled && s_iphOk
                   && s_iphMagFilt > STALL_IQ_MIN_A
                   && brkEff < 0.02f && ovr != 1;
    // "Parado" exige velEst baixo E silêncio de bordas de Hall (s_wheelStill,
    // 300 ms sem borda). Só o velEst não serve: quando o ruído denso come
    // bordas reais, o ângulo congela e o velEst desaba COM A RODA GIRANDO —
    // a corrente dispara contra a BEMF e o stall cortava "sem nexo" no meio
    // do teste. Rotor travado de verdade não emite borda nenhuma: passa pelo
    // portão e o trip continua pegando (em ~800 ms no total). O caso raro de
    // rotor travado COM bordas de ruído fica coberto pelos trips de |I|.
    bool stopped = fabsf(radsToKmh(velEst)) < STALL_SPEED_KMH && s_wheelStill;
#else
    bool driving = false;
    bool stopped = fabsf(radsToKmh(velEst)) < STALL_SPEED_KMH;
#endif
    if (driving && stopped) {
      stallMs += dt * 1000.0f;
      if (stallMs >= STALL_TRIP_MS && !(faults & FAULT_STALL)) {
        faults |= FAULT_STALL;
        mcPrintf("[motor] STALL! |I| de fase %.1fA com roda parada (TRAVA)\n",
                 s_iphMagFilt);
        FaultEntry fe; fe.odo_m = odoM; fe.uptime_s = millis() / 1000;
        fe.faults = FAULT_STALL; fe.ibat10 = (int16_t)(iBatFilt * 10.0f);
        fe.vbat10 = (uint16_t)constrain(vBatFilt * 10.0f, 0.0f, 65000.0f);
        fe.temp_c = (int8_t)constrain(tempFilt, -127.0f, 127.0f);
        fe.kmh10  = (int16_t)(radsToKmh(velEst) * 10.0f);
        faultLogAdd(fe);
      }
    } else stallMs = 0;
  }

  // ---- 5) Habilita/desabilita ---------------------------------------------
  // A TRAVA liga o motor sozinha: segurar a roda exige energia (PID em
  // velocidade 0, corrente limitada a LOCK_ILIM_A). Sem isso, travar com o
  // motor desligado só bloqueava o acelerador e a roda ficava livre.
  // E-STOP e faltas continuam mandando mais que a trava.
  bool wantEnable = (cmd.enable || lockActive)
                    && !cmd.estop && (faults & FAULT_LATCHING_MASK) == 0;
#if PHASE_CUR_ENABLED
  // Leitura de zero rejeitada NÃO bloqueia mais: o zero em uso continua sendo o
  // último BOM (ver phaseCurrentTrackZero), então a malha está medindo certo.
  // Bloquear aqui era consequência do rastreador aceitar a leitura ruim; com
  // ela descartada, o que resta é um aviso — e quem guarda contra disparo é o
  // detector de realimentação quebrada, que compara torque pedido com corrente
  // que volta.
  if (s_iphOk && phaseCurrentZeroPinned()) {
    static uint32_t lastBlk = 0;
    if (millis() - lastBlk > 10000) {
      lastBlk = millis();
      mcPrintf("[motor] AVISO: zero da corrente FORA DE FAIXA FISICA — canal "
               "morto, em curto ou desligado.\n"
               "[motor]   nao e deriva: confira a fiacao do INA240 e rode "
               "'diag'.\n");
    }
  }
#endif
  if (wantEnable && !motor.enabled) {
    appliedTarget = 0; velTarget = 0; powerScale = 1.0f;
    motor.PID_velocity.reset();
#if PHASE_CUR_ENABLED
    // Zerar os integradores da malha de CORRENTE aqui é obrigatório: um
    // integrador carregado de antes aplica tensão no instante em que o motor
    // é habilitado — é o "ficou doendo querendo girar sozinho" ao ligar.
    currentRegulatorReset(0.0f);
    s_iqTarget = 0; fbBadMs = 0;
    // Trava o zero no valor DESTE instante (ainda parado e desabilitado): é a
    // última oportunidade de saber que a corrente real é nula.
    // Mesmo critério do rastreio: travar o zero com a roda ainda descendo em
    // curto grava a corrente de frenagem como offset, de uma vez só.
    if (s_wheelStill) phaseCurrentTrackZero(dt, true);
#endif
    motor.enable();
    mcPrintf("[motor] HABILITADO\n");
  } else if (!wantEnable && motor.enabled) {
    motor.disable();                 // duty 0 => fases ao GND (freio suave parado)
    appliedTarget = 0; velTarget = 0;
#if PHASE_CUR_ENABLED
    currentRegulatorReset(0.0f);
    s_iqTarget = 0; fbBadMs = 0;
#endif
    mcPrintf("[motor] DESABILITADO\n");
  }

  // ---- 6) Foldback de corrente + derate térmico ---------------------------
#if CUR_SENSE_ENABLED
  // Em vez de desligar ao encostar no limite, reduz progressivamente a
  // tensão disponível (powerScale). Vale para tração E para regeneração.
  float iLim = iLimEff;                       // já inclui modo/walk/trava
  float regenILim = prm(P_REGEN_I) * regenVAllow;   // sobretensão corta regen
  fsRegenILim = regenILim;                          // p/ data-log profundo
  float excess = 0;
  if      (iBatFilt >  iLim)      excess =  iBatFilt - iLim;
  else if (iBatFilt < -regenILim) excess = -iBatFilt - fmaxf(0.5f, regenILim);

  if (excess > 0) powerScale -= excess * FOLDBACK_GAIN * dt;
  else            powerScale += FOLDBACK_RECOVERY * dt;
  powerScale = constrain(powerScale, 0.0f, 1.0f);
#else
  // Shunt desativado: sem foldback — o único teto de potência é o
  // voltage_limit da dashboard (e o derate térmico abaixo).
  powerScale = 1.0f;
#endif

  // Throttling térmico GRADUAL (P_THERM_DERATE liga/desliga). Usa o PIOR dos
  // dois sensores: curva térmica dos MOSFETs E rampa linear do MOTOR.
  float tempScale = 1.0f;
#if NTC_SENSE_ENABLED || MOTOR_NTC_SENSE_ENABLED
  if (prm(P_THERM_DERATE) > 0.5f) {
#if NTC_SENSE_ENABLED
    tempScale = fminf(tempScale,
                      constrain(curveEval(CRV_THERMAL, tempFilt) * 0.01f, 0.0f, 1.0f));
#endif
#if MOTOR_NTC_SENSE_ENABLED
    // rampa linear: 1,0 no início do derate -> 0 no corte
    float md = prm(P_MT_DERATE), mc = prm(P_MT_CUT);
    if (mc > md + 1.0f)
      tempScale = fminf(tempScale,
                        constrain((mc - motorTempFilt) / (mc - md), 0.0f, 1.0f));
#endif
  }
  fsTempScale = (uint8_t)(tempScale * 100.0f);
#endif

  // Derate de TRAÇÃO por tensão/SoC (P_TRAC_DERATE liga/desliga) + aviso
  float tracVScale = 1.0f;
#if VBAT_SENSE_ENABLED
  if (prm(P_TRAC_DERATE) > 0.5f)
    tracVScale = constrain(curveEval(CRV_TRAC_V, vBatFilt * 10.0f) * 0.01f, 0.0f, 1.0f);
  fsTracVScale = (uint8_t)(tracVScale * 100.0f);
  uvWarn = (vBatFilt > 5.0f && vBatFilt < prm(P_UV_WARN));
#endif

  updateVelEst(dt);

  float scale = fminf(powerScale, tempScale) * tracVScale;

  // Teto físico de modulação: acima de ~Vsupply/sqrt(3) (SVPWM) a senoide já
  // satura — pedir mais só distorce a forma de onda. Corta aqui.
  // Teto de modulação = o menor entre o limite do BOOTSTRAP (P_MODCEIL, físico
  // do FD2103S) e o teto de OPERAÇÃO da janela de amostragem low-side. Usa o
  // duty de OPERAÇÃO, não o físico: operar exatamente no duty máximo deixa a
  // amostra típica na fronteira do portão e cada fase fica CEGA no arco do
  // próprio pico de duty — mistura de fase velha+nova no dq era o caos do
  // topo (caixas-pretas 0x01/0x08). Com a folga (IPH_OP_FOLGA_US) a medição
  // segue honesta a 100% do punho — que é onde o limitador de corrente mais
  // importa. 0,577 é o máximo da SVPWM; ambos os tetos saem da latência
  // MEDIDA no boot (ver phase_current.h).
  float modCeil = prm(P_MODCEIL);
#if PHASE_CUR_ENABLED
  modCeil = fminf(modCeil, 0.577f * phaseCurrentDutyOp());
#endif
#if VBAT_SENSE_ENABLED
  const float vModCeil = fmaxf(vBatFilt, 20.0f) * modCeil;   // tensão REAL
#else
  const float vModCeil = VOLTAGE_POWER_SUPPLY * modCeil;
#endif
  float userVLimit = constrain(prm(P_VLIM), 0.5f, fminf((float)VOLTAGE_LIMIT_MAX, vModCeil));
  float effVLimit  = userVLimit;

  // O TETO DE TENSÃO NÃO É MAIS LIMITE DE CORRENTE. Antes ele era a única
  // barreira contra a corrente de fase em baixa rotação, e por isso carregava
  // o termo Ke*ω + margem. Agora a corrente é medida e regulada: a tensão só
  // precisa respeitar a FÍSICA (teto de modulação do bootstrap) — quem limita
  // torque é o alvo de corrente, e quem limita potência é o derate sobre ele.
  fsEffVLim = effVLimit;                           // teto efetivo p/ data-log

#if OPEN_LOOP_TEST
  // Malha aberta: nunca deixar o limite da dashboard passar da tensão de teste
  effVLimit = fminf(effVLimit, OPEN_LOOP_VOLTAGE_LIMIT);
#endif

  // Tensão máxima que o regulador de corrente pode aplicar — é também o teto
  // do integrador dele (anti-windup).
  motor.voltage_limit = effVLimit;


  // ---- TETO DE CORRENTE (o que substituiu tudo que era back-EMF) ---------
  // iqMax é a corrente de fase que o punho cheio pede. Sofre:
  //   - o teto do modo de pilotagem (eco/normal/turbo)
  //   - scale = foldback de corrente de BATERIA x derate térmico x derate de
  //     tensão. Reduzir CORRENTE é o certo: reduzir tensão como antes mexia
  //     no torque por um caminho indireto e dependente da velocidade.
  float iqMax = fminf(prm(P_IQ_MAX), (float)IQ_MAX_ABS_A) * scale;
  // Walk e trava eletrônica são limites de FASE por natureza: acontecem a ~0
  // km/h, onde a corrente de bateria é quase nula e só a de fase existe.
  // Aplicá-los aqui é mais correto do que no foldback de bateria (onde
  // estavam), que nessa faixa é cego.
  if (ovr == 2) iqMax = fminf(iqMax, WALK_ILIM_A);
  if (ovr == 1) iqMax = fminf(iqMax, LOCK_ILIM_A);
#if PHASE_CUR_ENABLED
  // Sem sensor de fase válido não existe malha de corrente: o firmware NÃO
  // finge que existe. Cai para um teto conservador e avisa na telemetria.
  if (!s_iphOk) iqMax = fminf(iqMax, 5.0f);
#endif
  // Teto da saída do PID de velocidade — que agora produz AMPÈRES.
  motor.PID_velocity.limit = iqMax;

  // ---- REGULADOR DO LIMITE DE VELOCIDADE ---------------------------------
  // Produz o teto de corrente que mantém a velocidade no limite. Roda sempre,
  // mesmo longe do teto — aí o integrador satura em iqMax e ele não restringe
  // nada, ficando pronto para agir sem degrau quando o limite se aproximar.
  {
    // Velocidade RÁPIDA aqui, não velEst: ver velFastRads(). Fechar um teto
    // com um sinal atrasado 80 ms é o mesmo que não ter teto numa roda solta.
    const float errKmh = vmaxKmh - fabsf(radsToKmh(velFastRads()));
    const float band   = prm(P_SPD_BAND);
    // TRANSFERÊNCIA SEM SALTO: enquanto o teto não está segurando nada, o
    // integrador ACOMPANHA A CORRENTE EM USO — não o teto máximo.
    //
    // As duas versões anteriores erraram do mesmo jeito, por caminhos opostos.
    // Saturar o integrador em iqMax o deixava estacionado no máximo. Trocar por
    // retro-cálculo o manteve na BORDA DA SATURAÇÃO — que também é iqMax. Nos
    // dois casos, ao cruzar o limite o teto valia iqMax e só descia pelo termo
    // proporcional, precisando de err = −iqMax/P para chegar a zero: com
    // iqMax 10 A e P 3, são 3,3 km/h de excesso ANTES de o teto fechar. Some o
    // atraso de 80 ms do filtro de velocidade e um teto de 4 km/h vira 8,8 —
    // que é exatamente o que a bancada mediu.
    //
    // Acompanhando a corrente em uso, o teto chega ao limite valendo o que o
    // motor está de fato puxando (3 A, não 10), e a faixa de fechamento passa a
    // ser corrente_em_uso/P — cerca de 1 km/h. O limitador engata sem degrau
    // porque já estava colado no ponto de operação.
    const float capRaw = band * errKmh + s_spdLimInt;
    s_spdLimCap = constrain(capRaw, 0.0f, iqMax);
    if (s_spdLimCap >= s_iqDemand) {          // não está segurando nada
      s_spdLimInt = constrain(s_iqDemand - band * errKmh, 0.0f, iqMax);
    } else {                                  // segurando: integra de verdade
      s_spdLimInt += SPD_LIM_I * errKmh * dt;
      s_spdLimInt  = constrain(s_spdLimInt, 0.0f, iqMax);
    }
  }

  // ---- 7) Cálculo do alvo -------------------------------------------------

#if OPEN_LOOP_TEST
  // Malha aberta: slider vira velocidade em rad/s, tensão fixa baixa.
  {
    float desired = (cmd.brake > 0.02f) ? 0.0f : cmd.throttle * OPEN_LOOP_MAX_RADS;
    appliedTarget = slewLimit(appliedTarget, desired, OPEN_LOOP_SLEW, OPEN_LOOP_SLEW * 3.0f, dt);
  }
#else
  if (curMode == MODE_TORQUE) {
    // ======================= MALHA DE CORRENTE ==============================
    // O alvo é CORRENTE (Iq em ampères). Tudo abaixo calcula ampères; o
    // SimpleFOC fecha a malha contra os INA240 e decide a tensão sozinho.
    float desired;
    if (brkEff > 0.01f) {
      coastActive = false;
      ecoState = 0; ecoSteadyMs = 0;               // freio cancela o eco
      // FREIO = corrente NEGATIVA (regen). Antes era "tensão contrária ao
      // sentido de giro", que dependia da back-EMF para virar corrente e por
      // isso frenava diferente em cada velocidade. Agora o manete pede
      // diretamente a corrente de frenagem: força de freio constante.
      float regenIqMax = fminf(prm(P_REGEN_I) * regenVAllow, iqMax);
      desired = (fabsf(velEst) > BRAKE_MIN_SPEED_RADS)
                  ? -sgnf(velEst) * brkEff * regenIqMax
                  : 0.0f;
      s_iqTarget = slewLimit(s_iqTarget, desired,
                             prm(P_SLEW_UP) * rideSlew, prm(P_SLEW_REL), dt);
    } else if (prm(P_COAST) > 0.5f && thrEff < 0.06f && s_spinReqA < 0.0f
               && fabsf(velEst) > kmhToRads(2.0f)) {
      // (o teste 'gira' pede acelerador zero de propósito — sem esta guarda a
      //  costeira assumiria e o alvo do teste nunca chegaria à malha)
      // COSTEIRA: soltar o acelerador = roda SOLTA (Iq alvo = 0), COM GUARDA:
      // costeira de verdade NÃO ACELERA. A pulsação de setor gera torque
      // médio real que o iq medido não enxerga (ver caixa-preta/config) — se
      // a velocidade estiver SUBINDO com alvo ~0, o integrador sangra até
      // parar de subir. Rastrear BEMF na descida segue livre.
      coastActive = true;
      s_iqTarget = slewLimit(s_iqTarget, COAST_TARGET_A,
                             prm(P_SLEW_UP), prm(P_SLEW_REL), dt);
      // (a guarda "alvo zero não acelera" virou LEI GLOBAL do modo torque —
      //  ver logo antes do regulador. Vale para a costeira e para qualquer
      //  outro caminho que produza alvo ~0 em velocidade.)
    } else {
      coastActive = false;
      desired = thrEff * iqMax;
      // PUNHO SOLTO (coast desligado) = FREIO-MOTOR LEVE, nunca alvo zero.
      // A caixa-preta provou que "iq=0 em velocidade" se auto-acelera (torque
      // da pulsação de setor invisível ao iq medido — ver RELEASE_BRAKE_A no
      // config). Alvo levemente NEGATIVO é estável por construção e reproduz
      // o comportamento do firmware antigo que era comprovadamente bom.
      if (thrEff < 0.06f && fabsf(velEst) > kmhToRads(RELEASE_BRAKE_MIN_KMH))
        desired = -sgnf(velEst) * RELEASE_BRAKE_A;
      if (s_spinReqA >= 0.0f) desired = fminf(s_spinReqA, iqMax);   // teste 'spin'
      s_iqDemand = fabsf(desired);         // o que o piloto pede, ANTES do teto

      // LIMITE DE VELOCIDADE por realimentação de VELOCIDADE, afunilando a
      // CORRENTE permitida na última faixa antes do teto. O corte antigo era
      // um teto de tensão em Ke*vmax + margem: dependia da precisão do Ke e,
      // por atuar em tensão, respondia devagar e surtava ritmicamente ao
      // encostar no limite. Corrente responde em milissegundos, então o
      // afunilamento é suave e estável sem nenhuma constante de motor.
      // LIMITE DE VELOCIDADE por REGULADOR (ver SPD_LIM_P no config.h): a saída
      // do PI é um TETO de corrente, e o alvo do punho é limitado por ele. Com
      // ação integral o teto converge para a corrente que de fato segura a
      // velocidade, em vez de bater entre tudo e nada.
      desired = fminf(desired, s_spdLimCap);

      // ---- ECO AUTOMÁTICO: só no modo ECO, cruzeiro estável ---------------
      // Mesma ideia de antes (assume o comando em cruzeiro estável e desce
      // devagar até o mínimo que segura a velocidade), agora aparando
      // CORRENTE em vez de tensão. QUALQUER gesto do piloto devolve o
      // controle na hora.
      {
        ecoVelEma += (dt / 1.0f) * (velEst - ecoVelEma);
        ecoThrEma += (dt / 0.5f) * (thrEff - ecoThrEma);
        bool eligible = prm(P_ECO_AUTO) > 0.5f && cmd.ride == 0
                     && motor.enabled && thrEff > 0.10f
                     && velEst > kmhToRads(8.0f)
                     && fabsf(velEst - ecoVelEma) < 0.03f * fmaxf(ecoVelEma, 1.0f)
                     && fabsf(thrEff - ecoThrEma) < 0.05f;
        if (ecoState == 0) {
          ecoSteadyMs = eligible ? (uint16_t)(ecoSteadyMs + dt * 1000.0f) : 0;
          if (ecoSteadyMs >= ECO_AUTO_ENGAGE_MS) {
            ecoState = 1;
            ecoVRef = velEst; ecoV = s_iqTarget; ecoThrRef = thrEff;
            mcPrintf("[eco] cruzeiro economico engatado (%.1f km/h)\n",
                     radsToKmh(ecoVRef));
          }
        } else {
          bool exitEco = !eligible
                      || fabsf(thrEff - ecoThrRef) > 0.12f            // piloto agiu
                      || velEst < ecoVRef - kmhToRads(3.0f)           // subida
                      || velEst > ecoVRef + kmhToRads(3.0f);          // descida
          if (exitEco) {
            ecoState = 0; ecoSteadyMs = 0;
          } else {
            // trims em A/s: a escala de corrente é ~4x menor que a de tensão
            // que estava aqui, então os ganhos do config valem direto.
            if (velEst > ecoVRef - kmhToRads(ECO_AUTO_TOL_KMH))
              ecoV -= ECO_AUTO_TRIM_V_S * dt;      // sobra folga: economiza
            else
              ecoV += ECO_AUTO_RECOV_V_S * dt;     // afundou: recupera
            ecoV = constrain(ecoV, 0.0f, iqMax);
            desired = fminf(desired, ecoV);        // eco só REDUZ, nunca soma
          }
        }
      }

      // Aliviar o acelerador desce a corrente até o pedido, e ponto. O PISO de
      // back-EMF que existia aqui ("nunca abaixo de 0,85*Ke*ω") só existia
      // porque em modo tensão baixar o alvo abaixo da back-EMF vira frenagem
      // regenerativa forte. Com malha de corrente, pedir 2 A é 2 A: não há
      // freio escondido, então não há piso a defender.
      s_iqTarget = slewLimit(s_iqTarget, desired,
                             prm(P_SLEW_UP) * rideSlew, prm(P_SLEW_REL), dt);
    }
    // O alvo em ampères vira TENSÃO aqui — com UM caso especial, provado
    // necessário na bancada: alvo ~0 e roda ~parada. Nesse estado o "erro"
    // que o regulador vê é só ruído de medição, e o ângulo de Hall salta 60°
    // a cada arrastadinha do rotor: o PI persegue um alvo que gira, o motor
    // dá trancos para lá e para cá e não assenta. Torque zero PARADO se
    // realiza exatamente com TENSÃO zero (fases ao GND, sem back-EMF que
    // empurre corrente) — então abaixo de IQ0_STOP_KMH com alvo ~0 é isso
    // que se aplica, com o integrador zerado para o retorno ser limpo.
    // Acima do limiar (ou com alvo real) a malha assume — é ela que faz a
    // costeira, segurando iq=0 contra a back-EMF.
    // GUARDA GLOBAL "ALVO ZERO NÃO ACELERA" — vale para QUALQUER caminho do
    // modo torque que produza alvo ~0 em velocidade (costeira, dead-man,
    // borda da curva do punho). Física: com iq alvo 0 a velocidade só pode
    // DECAIR; se está subindo, a malha injeta energia que o iq medido não
    // enxerga (torque da pulsação de setor — provado na caixa-preta). A
    // derivada da velocidade é o sensor de torque que falta: subiu acima do
    // limiar, o integrador sangra até parar. Rastrear BEMF na descida
    // (costeira legítima) segue livre.
    {
      static float vfPrev = 0, accKmhS = 0;
      static bool  engaged = false;
      const float vfK = fabsf(radsToKmh(velFastRads()));
      const float raw = (vfK - vfPrev) / fmaxf(dt, 1e-4f);
      vfPrev = vfK;
      accKmhS += (dt / 0.15f) * (constrain(raw, -400.0f, 400.0f) - accKmhS);
      const bool zeroTgt = fabsf(s_iqTarget) < 0.3f &&
                           vfK > RELEASE_BRAKE_MIN_KMH;
      // histerese: engata subindo forte, solta quando a subida cessa
      if (!zeroTgt)                                engaged = false;
      else if (accKmhS > COAST_GUARD_ACC_KMH_S)    engaged = true;
      else if (accKmhS < COAST_GUARD_ACC_OFF)      engaged = false;
      if (engaged) {
        // decaimento PROPORCIONAL: autoridade cresce com o próprio Vq e
        // vence o rewind do integrador em qualquer ganho (ver config)
        curRegInt -= curRegInt * fminf(dt / COAST_GUARD_TAU_S, 1.0f);
      }
    }
    if (fabsf(s_iqTarget) < 0.1f &&
        fabsf(velFastRads()) < kmhToRads(IQ0_STOP_KMH)) {
      currentRegulatorReset(0.0f);
      appliedTarget = 0.0f;
    } else {
      appliedTarget = runCurrentLoops(s_iqTarget, dt, effVLimit);
    }

  } else {
    // MODE_VELOCITY (inclui trava/walk/cruise/step, que forçam este modo)
    float vmax = kmhToRads(vmaxKmh);
    motor.velocity_limit = vmax;

    if (ovr == 1) {
      // TRAVA: segura a roda em velocidade zero, ignora acelerador/freio
      velTarget = 0;
    } else if (ovr == 2) {
      // WALK: acelerador vira botão de "empurrar" a WALK_KMH
      float desired = (cmd.throttle > 0.05f) ? kmhToRads(WALK_KMH) : 0.0f;
      velTarget = slewLimit(velTarget, desired, prm(P_ACCEL) * 0.5f,
                            prm(P_DECEL), dt);
    } else if (ovr == 3) {
      // CRUISE: mantém a velocidade capturada no engate
      velTarget = slewLimit(velTarget, cruiseRads, prm(P_ACCEL), prm(P_DECEL), dt);
    } else if (ovr == 4) {
      // STEP RESPONSE: degrau controlado + captura a 1 kHz no buffer
      float target = capBase + ((capIdx >= 250) ? capAmpRads : 0.0f);
      velTarget = target;                       // degrau SECO de propósito
      if (capIdx < CAP_N) {
        capTgt[capIdx] = (int16_t)constrain(velTarget * 100.0f, -32000.0f, 32000.0f);
        capVel[capIdx] = (int16_t)constrain(velEst   * 100.0f, -32000.0f, 32000.0f);
        capCur[capIdx] = (int16_t)constrain(iBatFilt *  10.0f, -32000.0f, 32000.0f);
        capIdx = capIdx + 1;
      }
      if (capIdx >= CAP_N) { capState = 2; velTarget = capBase; }
    } else if (brkEff > 0.01f) {
      float decel = brkEff * prm(P_BRAKE_DEC);
      if (velTarget > 0) velTarget = fmaxf(0.0f, velTarget - decel * dt);
      else               velTarget = fminf(0.0f, velTarget + decel * dt);
    } else {
      float desired = thrEff * vmax;
      velTarget = slewLimit(velTarget, desired, prm(P_ACCEL) * rideSlew,
                            prm(P_DECEL), dt);
    }
    // CASCATA: velocidade -> corrente -> tensão. O PID de velocidade produz um
    // alvo de CORRENTE (ganhos em A), e o mesmo regulador de corrente do modo
    // torque o converte em tensão. Assim os dois modos compartilham a única
    // malha de corrente que existe, em vez de cada um ter a sua.
    motor.PID_velocity.limit = iqMax;
    if (ovr == 0 && fabsf(velTarget) < 0.5f &&
        fabsf(velFastRads()) < kmhToRads(IQ0_STOP_KMH)) {
      // PARADO DE VERDADE no modo velocidade (sem trava/walk/cruise, que
      // precisam segurar a roda com corrente): mesmo caso do modo torque —
      // deixar o PID regular ruído de velEst com o ângulo de setor parado é
      // o ciclo-limite do "treme parado". Torque zero parado se realiza com
      // TENSÃO zero; os integradores nascem limpos quando o alvo voltar.
      motor.PID_velocity.reset();
      currentRegulatorReset(0.0f);
      s_iqTarget    = 0.0f;
      appliedTarget = 0.0f;
    } else {
      s_iqTarget = motor.PID_velocity(velTarget - velEst);
      appliedTarget = runCurrentLoops(s_iqTarget, dt, effVLimit);
    }
  }
#endif

  if (!motor.enabled) { appliedTarget = 0; velTarget = 0; s_udCmd = 0; }

  // ---- 7b) Energia / instrumentos -----------------------------------------
  {
    float mps = fabsf(velEst) * wheelRadM();
    odoAcc += mps * dt;
    if (odoAcc >= 1.0f) {
      uint32_t m = (uint32_t)odoAcc;
      odoM += m; tripM += m; odoAcc -= m;
    }
    float pw = vBatFilt * iBatFilt;             // + tração / − regen
    if (pw > 0) tripWh      += pw * dt / 3600.0f;
    else        tripWhRegen += -pw * dt / 3600.0f;
    whNetTotal += pw * dt / 3600.0f;            // líquido (não zera com a trip)

    // MTPA: acumuladores da janela de 1 s (2 somas por ms — custo ~zero)
    sIsum += iBatFilt; sN++;
    if (sN == 1) { sVmin = velEst; sVmax = velEst; }
    else { sVmin = fminf(sVmin, velEst); sVmax = fmaxf(sVmax, velEst); }

    // Duty VETORIAL: |U| = √(Uq²+Ud²) — o que a modulação realmente usa.
    // Mostrar só o Vq escondia o Ud da malha de d (64% no app com o vetor
    // exatamente no teto de 67%).
    float vmag  = hypotf(motor.voltage.q, motor.voltage.d);
    float vbase = (vBatFilt > 5.0f ? vBatFilt : VOLTAGE_POWER_SUPPLY) * 0.577f;
    fsDuty   = (uint8_t)constrain(vmag / vbase * 100.0f, 0.0f, 100.0f);
    fsKmh    = radsToKmh(velEst);
    fsIbat   = iBatFilt;
    fsVbat   = vBatFilt;
    fsTarget = appliedTarget;
    fsTemp   = (int8_t)constrain(tempFilt, -127.0f, 127.0f);
    fsFw     = (uint8_t)constrain(fwIdA * 2.0f, 0.0f, 255.0f);  // id de FW [A]×2
    fsScale  = (uint8_t)constrain(powerScale * 100.0f, 0.0f, 100.0f);
    fsMtpa   = (int8_t)constrain(mtpaTrim * 572.958f, -127.0f, 127.0f);
    fsHead   = (uint8_t)constrain(tempScale * 100.0f, 0.0f, 100.0f);

    // --- bateria inteligente (1 Hz) ---------------------------------------
    if (sliceCount % 1000 == 3) {
#if VBAT_SENSE_ENABLED
      bool resting = fabsf(iBatFilt) < 0.5f && fabsf(velEst) < 0.5f
                     && vBatFilt > 5.0f;
      restSec = resting ? (uint16_t)(restSec + 1) : 0;

      if (restSec == 5) {                    // 5 s parado: tensão confiável
        float socRest = socFromRestV(vBatFilt);
        if (socAnchor < 0) {                 // primeira âncora (boot)
          socAnchor = socRest; whAnchor = whNetTotal; socLive = socRest;
        } else if (socRest > socAnchor + 0.03f) {
          // subiu >3%: foi carregada — re-ancora SEM aprender
          socAnchor = socRest; whAnchor = whNetTotal; socLive = socRest;
        } else {
          float dSoc = socAnchor - socRest;
          float dWh  = whNetTotal - whAnchor;
          if (prm(P_BATT_LEARN) > 0.5f && dSoc >= 0.08f && dWh > 5.0f) { // aprende
            float est = dWh / dSoc;
            if (est > 50.0f && est < 2000.0f) {
              if (capLearned <= 0) capLearned = est;
              else capLearned += 0.25f * (est - capLearned);
              capDirty = true;
              mcPrintf("[bat] capacidade aprendida: %.0f Wh (medida %.0f)\n",
                       capLearned, est);
            }
            socAnchor = socRest; whAnchor = whNetTotal;
          }
          socLive = socRest;                 // repouso = verdade
        }
      }
      // entre repousos: SoC por contagem de energia sobre a capacidade
      if (socAnchor >= 0 && capLearned > 0)
        socLive = constrain(socAnchor - (whNetTotal - whAnchor) / capLearned,
                            0.0f, 1.0f);
#endif
      // --- MTPA (1 Hz): avalia a janela do último segundo -----------------
      {
        float iAvg   = sN ? sIsum / sN : 0;
        float vAvg   = fabsf(velEst);
        bool  steady = motor.enabled && prm(P_MTPA) > 0.5f
                    && fwIdA < 0.05f                // nunca junto com FW
                    && cmd.brake < 0.02f && ovrNow == 0
                    && vAvg > 15.0f && iAvg > 1.5f
                    && (sVmax - sVmin) < fmaxf(0.5f, 0.03f * vAvg);
        if (!steady) {
          mtpaPhase = 0; mtpaSecs = 0; mtpaExplore = 0;
          mtpaAcc[0] = mtpaAcc[1] = mtpaAcc[2] = 0;
        } else {
          mtpaAcc[mtpaPhase] += iAvg;
          if (++mtpaSecs >= 2) {                    // 2 s por fase
            mtpaSecs = 0;
            if (mtpaPhase == 0)      { mtpaPhase = 1; mtpaExplore =  MTPA_DELTA; }
            else if (mtpaPhase == 1) { mtpaPhase = 2; mtpaExplore = -MTPA_DELTA; }
            else {
              // decide: menor corrente vence; passo pequeno na direção dela
              float b0 = mtpaAcc[0], bp = mtpaAcc[1], bm = mtpaAcc[2];
              if      (bp < b0 && bp <= bm) mtpaTrim += MTPA_STEP;
              else if (bm < b0 && bm <  bp) mtpaTrim -= MTPA_STEP;
              mtpaTrim = constrain(mtpaTrim, -MTPA_MAX, MTPA_MAX);
              mtpaDirty = true;
              mtpaPhase = 0; mtpaExplore = 0;
              mtpaAcc[0] = mtpaAcc[1] = mtpaAcc[2] = 0;
            }
          }
        }
        sIsum = 0; sN = 0;                          // nova janela de 1 s
      }

      // consumo recente: EWMA por trecho de 200 m
      float km = tripM / 1000.0f + odoAcc * 0.001f;
      if (ckM <= 0) { ckM = km; ckWh = whNetTotal; }
      if (km - ckM >= 0.2f) {
        float d = (whNetTotal - ckWh) / (km - ckM);
        whkmRecent += 0.3f * (constrain(d, 0.0f, 80.0f) - whkmRecent);
        whkmRecent = constrain(whkmRecent, 3.0f, 60.0f);
        ckM = km; ckWh = whNetTotal;
      }
    }
  }

  // ---- 8) Caixa-preta + telemetria ----------------------------------------
#if PHASE_CUR_ENABLED && IPH_DIAG_TOOLS
  bbTick();
#endif
  if (sliceCount % 50 == 0) publishTelemetry(cmd, linkOk, scale);
}

// ===========================================================================
// AUTO-CALIBRAÇÃO ("auto-learning") — adapta a controladora ao motor.
// Roda na task do motor (bloqueia segundos). RODA SUSPENSA/BANCADA + fonte
// LIMITADA em corrente. Todas as etapas em tensão baixa; as proteções de fase
// (teto duro + stall) do controlSlice não rodam aqui, então cada etapa tem seu
// próprio cuidado (tensão baixa, ângulos que nunca travam o rotor).
// ---------------------------------------------------------------------------

// As calibrações raciocinam em TENSÃO (aplicam um vetor e observam). Com o
// torque em malha de CORRENTE, motor.move(x) interpreta x como AMPÈRES — então
// cada rotina que gira o motor troca para torque por tensão enquanto dura e
// devolve o modo no fim. O sensor de corrente continua medindo o tempo todo:
// é ele que protege as etapas, no lugar do antigo teto por back-EMF.
static void calEnterVoltageMode() {
  // A calibração comanda tensão DIRETO, sem passar pelo regulador.
  motor.controller        = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  s_iqTarget = 0; fbBadMs = 0;
  currentRegulatorReset(0.0f);
}
static void calExitVoltageMode() {
  // O modo de torque é SEMPRE tensão agora; só o regulador precisa voltar
  // zerado, senão ele reassume com integrador de antes da calibração.
  currentRegulatorReset(0.0f);
  s_iqTarget = 0; appliedTarget = 0;
  // Toda rotina que passa por aqui BLOQUEOU a fatia de 1 kHz por segundos. Sem
  // avisar, a primeira fatia depois dela recebe um dt de segundos: integradores
  // dão um passo gigante e velEst sai de uma diferença de ângulo de 8 s. Ver o
  // resync no laço da task.
  s_sliceResync = true;
}

// Devolver o motor no fim de uma rotina que GIRA não pode ser um disable().
// O driver é 3-PWM SEM pino de enable: duty 0 nas três fases deixa os low-side
// conduzindo, ou seja, as três fases em CURTO (ver o disable() do repouso, que
// usa isso de propósito como freio suave). Com a roda parada é inofensivo; a
// 20 km/h é um freio de curto — 40-55 A REAIS e trip de sobrecorrente. Foi
// exatamente o 0x01 no fim do primeiro 'varre'.
// Enquanto houver rotação, então, segura CORRENTE ZERO: as fases seguem
// acionadas acompanhando a própria back-EMF, sem torque e sem corrente. Só
// desabilita quando a roda estiver devagar de verdade.
static void calReleaseSpinning(float vlim) {
  uint32_t t0 = micros();
  int ms = 0;
  while (ms < 4000) {
    motor.loopFOC();
    if ((int32_t)(micros() - t0) < (ms + 1) * 1000) continue;
    ms++;
    const float vf = fabsf(radsToKmh(velFastRads()));
    if (vf < 3.0f) break;
    phaseCurrentRead(&s_iph, motor.electrical_angle);
    motor.move(currentRegulator(0.0f, s_iph.iq, vlim, 0.001f));
  }
  motor.move(0.0f); motor.loopFOC(); motor.disable();
}

// Guarda de sobrecorrente das etapas que GIRAM o motor. Agora é a corrente de
// FASE MEDIDA — antes era a de BATERIA (INA226), que com o rotor parado nem
// enxerga a corrente que circula na fase. Deixou de ser "backstop" e virou a
// proteção principal da calibração.
static bool autoCalOverCurrent() {
#if PHASE_CUR_ENABLED
  float i = s_iphOk ? s_iph.imag : fabsf(ina226CurrentA());
#else
  float i = fabsf(ina226CurrentA());
#endif
  if (i > AUTOCAL_I_ABORT_A) {
    motor.move(0.0f); motor.disable();
    mcPrintf("[autocal] ABORTADO: sobrecorrente (>%.1f A) durante a calibracao\n",
             (double)AUTOCAL_I_ABORT_A);
    return true;
  }
  return false;
}

// Giro da calibração SEM TRANCO: rampa de tensão (soft-start) + teto por
// CORRENTE DE FASE MEDIDA. O teto antigo era V <= Ke*vel + margem, um modelo
// que dependia justamente do Ke que a calibração ainda ia medir — circular.
// Agora a rampa simplesmente para de subir (e recua) quando |I| encosta no
// limite de aborto. Chamar a cada iteração; calRampReset() zera a rampa antes
// de cada etapa que gira. Retorna a tensão REALMENTE aplicada (o cálculo do Ke
// precisa do valor certo).
static float calRampV = 0.0f;
static uint32_t calRampUs = 0;
static void  calRampReset() { calRampV = 0.0f; calRampUs = 0; }
static float calSpin(float targetV) {
  uint32_t now = micros();
  float dt = (calRampUs == 0) ? 0.0f : (now - calRampUs) * 1e-6f;
  calRampUs = now;
  if (dt > 0.02f) dt = 0.02f;                       // ignora hiato grande

#if PHASE_CUR_ENABLED
  // Regulador simples sobre a corrente medida: sobe enquanto houver folga,
  // recua (2x mais rápido) ao passar de 60% do limite de aborto.
  const float iCeil = AUTOCAL_I_ABORT_A * 0.6f;
  phaseCurrentRead(&s_iph, motor.electrical_angle);
  if (s_iphOk && s_iph.imag > iCeil)     calRampV -= AUTOCAL_RAMP_V_S * 2.0f * dt;
  else                                   calRampV += AUTOCAL_RAMP_V_S * dt;
#else
  calRampV += AUTOCAL_RAMP_V_S * dt;
#endif
  calRampV = constrain(calRampV, 0.0f, targetV);
  motor.loopFOC();
  motor.move(calRampV);
  return calRampV;
}

// ---------------------------------------------------------------------------
// TESTE DE DEGRAU DA MALHA DE CORRENTE — mostra se ela oscila
// ---------------------------------------------------------------------------
// Aplica um degrau pequeno de Iq e captura a corrente MEDIDA a 1 kHz. É o único
// jeito de separar as três coisas que produzem o mesmo sintoma no guidão
// ("motor enlouquecido"): malha oscilando, malha lenta demais, ou comutação
// errada. O gráfico responde qual é em três linhas de serial.
// RODA PRESA — aplica corrente de verdade.
#if PHASE_CUR_ENABLED && IPH_DIAG_TOOLS
// openLoop = aplica um degrau de TENSÃO fixa, sem o regulador. É o teste que
// separa de vez as duas hipóteses: se a ondulação continua com a tensão
// CONSTANTE, ela não vem da malha — vem da comutação, da mecânica ou da
// medição. Nenhum ajuste de ganho tocaria nela.
static void runCurrentStep(float amps, bool openLoop = false) {
  const int N = 250;                       // 250 ms a 1 kHz
  static int16_t cap[250];
  mcPrintf("[istep] degrau de %.1f %s — RODA PRESA%s\n", amps,
           openLoop ? "V (MALHA ABERTA)" : "A",
           openLoop ? " | tensao fixa: a malha NAO participa" : "");

  calEnterVoltageMode();                   // parte de um estado limpo
  motor.enable();

  uint32_t t0 = micros();
  float u = 0.0f;
  for (int i = 0; i < N; i++) {
    // O regulador roda UMA VEZ por milissegundo, igual à fatia de controle.
    // A versão anterior o chamava a cada iteração do loopFOC (vários kHz) com
    // dt fixo de 1 ms: o integrador avançava 5-8 ms de integração por
    // milissegundo real, ou seja, o teste media um I efetivo 5-8x maior que o
    // configurado. Todo ajuste feito contra isso mediu outra planta.
    phaseCurrentRead(&s_iph, motor.electrical_angle);
    u = openLoop ? amps                       // degrau de TENSÃO (malha fora)
                 : currentRegulator(i < 25 ? 0.0f : amps, s_iph.iq,
                                    motor.voltage_limit, 0.001f);
    if (openLoop && i < 25) u = 0.0f;
    while ((int32_t)(micros() - t0) < (i + 1) * 1000) {
      motor.loopFOC();
      motor.move(u);
    }
    cap[i] = (int16_t)constrain(s_iph.iq * 100.0f, -32000.0f, 32000.0f);
  }
  motor.move(0.0f);
  motor.loopFOC();
  motor.disable();
  calExitVoltageMode();

  // --- desenho + métricas ---
  float mx = 0;
  for (int i = 0; i < N; i++) mx = fmaxf(mx, fabsf(cap[i] / 100.0f));
  const float sc = fmaxf(mx, fabsf(amps)) * 1.2f;
  mcPrintf("[istep] escala vertical: +-%.1f A | '=' = alvo | cada linha = 50 ms\n", sc);
  for (int r = 0; r < N; r += 50) {
    char line[51];
    for (int c = 0; c < 50 && r + c < N; c++) {
      float v = cap[r + c] / 100.0f;
      int lvl = (int)(v / sc * 4.0f + 4.5f);   // 0..8
      line[c] = "v_-.=+^#A"[constrain(lvl, 0, 8)];
    }
    line[50] = 0;
    mcPrintf("[istep] %3d ms |%s\n", r, line);
  }
  // --- métricas ------------------------------------------------------------
  // Regime = média dos últimos 50 ms. Sobressinal e ondulação são medidos em
  // JANELAS SEPARADAS: o transitório logo após o degrau, e o regime depois de
  // assentado. Misturar os dois foi o que fez a versão anterior chamar RUÍDO de
  // oscilação — com ±0,4 A de ruído sobre 3 A de regime, o sinal cruza a
  // própria média o tempo todo, e contar esses cruzamentos dá "48 oscilações"
  // num laço que estava perfeitamente assentado.
  float ss = 0; for (int i = N - 50; i < N; i++) ss += cap[i] / 100.0f;
  ss /= 50.0f;

  float peak = 0;                          // pico do transitório
  for (int i = 25; i < N; i++) peak = fmaxf(peak, cap[i] / 100.0f);

  // Ondulação DEPOIS de assentado, e só o que exceder o ruído conta como
  // oscilação de verdade.
  const float noise = prm(P_CUR_DB);
  float ripple = 0; int rings = 0; bool above = false;
  for (int i = N - 150; i < N; i++) {
    float d = cap[i] / 100.0f - ss;
    ripple = fmaxf(ripple, fabsf(d));
    bool a = d > noise;                    // só excursões acima do ruído
    if (a && !above) rings++;
    if (fabsf(d) < noise * 0.5f) above = false;
    else if (a) above = true;
  }

  // Assentamento: primeiro instante a partir do qual não sai mais de ±20 %
  int settle = -1;
  for (int i = 25; i < N; i++) {
    bool ok = true;
    for (int j = i; j < N; j++)
      if (fabsf(cap[j] / 100.0f - ss) > fmaxf(0.2f * fabsf(ss), noise)) { ok = false; break; }
    if (ok) { settle = i - 25; break; }
  }

  mcPrintf("[istep] alvo %.2f A | regime %.2f A (erro %+.0f%%) | pico %.2f A "
           "(sobressinal %.0f%%)\n",
           amps, ss, ss != 0 ? 100.0f * (ss - amps) / amps : 0.0f, peak,
           ss > 0.1f ? 100.0f * (peak - ss) / ss : 0.0f);
  mcPrintf("[istep] assentou em %d ms | ondulacao no regime %.2f A "
           "(ruido do sensor %.2f A) | %d excursoes acima do ruido\n",
           settle >= 0 ? settle : 999, ripple, (double)noise, rings);

  // O veredito imprime o COMANDO a digitar. Dizer "reduza o ganho" é ambíguo:
  // baixar o P sozinho piora, porque derruba o cruzamento e deixa o zero do
  // compensador (I/P) ACIMA dele — tira margem de fase em vez de dar. Quem
  // amortece é baixar o I, que empurra o zero para baixo. Com o comando pronto
  // não há como seguir a direção errada.
  if (openLoop) {
    mcPrintf("[istep] MALHA ABERTA: ondulacao %.2f A com tensao CONSTANTE.\n"
             "[istep]   Se este numero for parecido com o do degrau de corrente,\n"
             "[istep]   a ondulacao NAO vem da malha — vem da comutacao, da\n"
             "[istep]   mecanica ou da medicao, e ganho nenhum a resolve.\n",
             ripple);
    return;
  }
  const float over = ss > 0.1f ? (peak - ss) / ss : 0.0f;
  const float P = prm(P_CUR_P), I = prm(P_CUR_I);
  if (rings > 5)
    mcPrintf("[istep] VEREDITO: ondulacao de %.2f A no regime (%d excursoes).\n"
             "[istep]   digite:  i %.0f  e repita.\n"
             "[istep]   SE A ONDULACAO NAO CAIR com o I menor, NAO e a malha: e\n"
             "[istep]   perturbacao entrando (comutacao, mecanica). Baixar ganho\n"
             "[istep]   nao resolve — volte o I e procure a fonte.\n",
             ripple, rings, I * 0.5f);
  else if (over > 0.6f)
    mcPrintf("[istep] VEREDITO: assenta em %d ms, mas com %.0f%% de sobressinal.\n"
             "[istep]   digite:  i %.0f     (so o I; baixar o P PIORA, ver nota)\n",
             settle >= 0 ? settle : 999, over * 100.0f, I * 0.5f);
  else if (fabsf(ss - amps) > 0.3f * fabsf(amps))
    mcPrintf("[istep] VEREDITO: NAO SEGUE O ALVO (%.2f A de %.2f A).\n"
             "[istep]   se nao esta oscilando, o ganho e baixo: digite  p %.3f\n"
             "[istep]   se esta, o problema e comutacao ou escala de corrente.\n",
             ss, amps, P * 2.0f);
  else
    mcPrintf("[istep] VEREDITO: estavel e seguindo o alvo. Digite 'save' para "
             "gravar P=%.3f I=%.0f.\n", P, I);

  // A roda TEM de estar presa: com ela livre o motor acelera durante o teste e
  // o que se mede deixa de ser a malha de corrente e passa a incluir a mecânica
  // e a back-EMF subindo. O teste avisa em vez de entregar número enganoso.
  const float kmhEnd = fabsf(radsToKmh(velEst));
  if (kmhEnd > 1.0f)
    mcPrintf("[istep] AVISO: a roda GIROU durante o teste (%.1f km/h no fim). "
             "Os numeros acima incluem a mecanica e a back-EMF, nao so a malha. "
             "Prenda a roda e repita.\n", kmhEnd);
}
#endif

// ---------------------------------------------------------------------------
// SCAN DE ÂNGULO SOB ROTAÇÃO — mede o resto que a calibração estática não vê
// ---------------------------------------------------------------------------
// O 'zea' alinha o rotor parado, e o Hall só resolve o SETOR — o resíduo δ
// (0..60°) dentro do setor fica invisível. Com o ângulo INTERPOLADO esse
// resíduo vira erro CONSTANTE de comutação: torque/ampère cai com cos(δ) e
// nasce id parasita — foi o "consome mais que o back-EMF antigo na mesma
// rotação". Este scan mede δ do único jeito honesto: girando a velocidade
// fixa e varrendo o offset de comutação, a corrente de BATERIA desenha um U
// cujo mínimo é o ângulo verdadeiro. Aplica e salva o mínimo.
// RODA LIVRE. Bloqueia ~15 s; guardas próprias (fatia parada).
#if PHASE_CUR_ENABLED && IPH_DIAG_TOOLS
static void runAngleScan(float amps, float kmh) {
  if (!s_iphOk) { motorReportCmdStatus(CMDERR_NO_IPHASE); return; }
  const float zeaBase = prm(P_ZEA);
  mcPrintf("[ang] varredura de comutacao: ate %.1f A | alvo %.1f km/h | "
           "RODA LIVRE (~15 s)\n", amps, kmh);
  calExitVoltageMode();
  motor.voltage_limit = 8.5f;
  motor.enable();

  float bestOff = 0.0f, bestI = 1e9f;
  bool  aborted = false;
  for (int s = 0; s < 13 && !aborted; s++) {
    const float offDeg = -36.0f + 6.0f * s;
    motor.zero_electric_angle = zeaBase + offDeg * 0.0174533f;
    const uint32_t t0us = micros();
    int   ms = 0, n = 0;
    float u = 0.0f, magF = 0.0f;
    double accI = 0, accV = 0;
    const int SETTLE = 500, MEAS = 400;      // [ms]
    while (ms < SETTLE + MEAS) {
      motor.loopFOC();
      motor.move(u);
      if ((int32_t)(micros() - t0us) >= (ms + 1) * 1000) {
        ms++;
        phaseCurrentRead(&s_iph, motor.electrical_angle);
        const float vf = fabsf(radsToKmh(velFastRads()));
        // mini-limitador de velocidade do scan (a fatia está parada)
        const float tgt = constrain(2.0f * (kmh - vf), 0.0f, amps);
        u = currentRegulator(tgt, s_iph.iq, motor.voltage_limit, 0.001f);
        magF += 0.3f * (s_iph.imag - magF);
        if (magF > AUTOCAL_I_ABORT_A || vf > kmh * 2.0f + 4.0f) {
          aborted = true; break;
        }
        if (ms > SETTLE) { accI += ina226CurrentA(); accV += vf; n++; }
      }
    }
    if (aborted) break;
    const float iB = n ? (float)(accI / n) : 0.0f;
    const float vA = n ? (float)(accV / n) : 0.0f;
    mcPrintf("[ang] %+4.0f graus: %.2f A de bateria @ %.1f km/h\n",
             offDeg, iB, vA);
    // só vale se a roda realmente girou perto do alvo (angulo muito errado
    // nem sustenta a velocidade — e ai a corrente baixa e mentirosa)
    if (vA > 0.6f * kmh && iB < bestI) { bestI = iB; bestOff = offDeg; }
  }
  motor.zero_electric_angle = zeaBase;       // angulo bom ANTES de soltar
  calReleaseSpinning(motor.voltage_limit);   // NUNCA disable() com a roda solta
  calExitVoltageMode();

  if (aborted) {
    mcPrintf("[ang] ABORTADO (corrente ou velocidade fora da guarda)\n");
    motorReportCmdStatus(CMDERR_CAL_FAIL);
    return;
  }
  if (bestI > 1e8f) {
    mcPrintf("[ang] FALHOU: nenhum ponto sustentou a velocidade — roda presa?\n");
    motorReportCmdStatus(CMDERR_CAL_FAIL);
    return;
  }
  mcPrintf("[ang] minimo em %+.0f graus (%.2f A de bateria). ", bestOff, bestI);
  if (fabsf(bestOff) >= 3.0f) {
    paramSet(P_ZEA, zeaBase + bestOff * 0.0174533f);
    paramsRequestSave();
    mcPrintf("APLICADO: P_ZEA = %.4f rad (salvo).\n", prm(P_ZEA));
  } else {
    mcPrintf("o angulo atual ja e o otimo — nada a mudar.\n");
  }
  motorReportCmdStatus(CMDERR_CAL_OK);
}
#endif

// ---------------------------------------------------------------------------
// VARREDURA EM RAMPA — de ONDE vem a aspereza, faixa de rotação por faixa
// ---------------------------------------------------------------------------
// O 'gira' só sabia dizer QUANTA ondulação existe num ponto. Isso nunca ia
// responder "por que não está liso", porque um número escalar não distingue as
// causas — e cada uma pede um conserto diferente. A separação é feita por
// DEMODULAÇÃO SÍNCRONA: multiplica id/iq pelo seno e cosseno de cada harmônica
// DO PRÓPRIO ÂNGULO e acumula. Como a referência é o ângulo real, e não um
// relógio, a medida não se desmancha quando a velocidade varia dentro da
// janela — o que é essencial aqui, porque o teste mede ACELERANDO.
//
//   1x eletrica  -> OFFSET (zero) desigual entre os canais de corrente: um
//                   valor fixo no referencial parado gira a 1x no dq. NÃO
//                   depende da corrente — some no ruído quando ela é alta.
//   2x eletrica  -> GANHO desigual entre os dois canais medidos (a 'cal'
//                   imprime as duas escalas). Vira sequência negativa, que no
//                   dq gira a 2x. É PROPORCIONAL à corrente.
//   6x eletrica  -> COMUTAÇÃO: o ângulo entre bordas de Hall é interpolado, e
//                   todo erro de geometria ou de atraso vira dente-de-serra
//                   com 6 dentes por volta elétrica.
//   1x mecanica  -> mecânica/ímãs: excentricidade, rolamento, pneu. Medida na
//                   rampa INTEIRA, não por janela: a 0,5-10 Hz não cabem dois
//                   ciclos em 250 ms, e demodular meio ciclo é inventar.
//
// POR QUE A CORRENTE CONSTANTE, E NÃO A VELOCIDADE CONSTANTE. A primeira
// versão segurava a velocidade em degraus — e numa roda LIVRE segurar
// velocidade não custa torque nenhum: a corrente real ficou em 0,1-0,3 A,
// abaixo do próprio ruído de medição (~0,9 A). Todas as harmônicas saíram no
// piso do estimador, e como a compensação do filtro é maior nas altas, o
// RUÍDO ranqueava a 2x em primeiro lugar. O teste elegia um culpado sem ter
// medido nada. Numa bancada a única carga que existe é a INÉRCIA da roda, e
// ela só aparece ACELERANDO — por isso agora a rampa é de corrente fixa, do
// repouso ao teto, e cada janela é medida com corrente de verdade.
//
// Todo número vem com o PISO DO ESTIMADOR ao lado: com ruído de banda larga
// σ e n amostras, a demodulação de uma harmônica INEXISTENTE ainda devolve
// ~1,77·σ/√n. Sem esse piso ao lado, qualquer tabela dessas parece cheia de
// achados. Quem fica abaixo de 2x o próprio piso sai marcado com '~'.
//
// RODA LIVRE, parte do repouso. Bloqueia ~6 s.
#if PHASE_CUR_ENABLED && IPH_DIAG_TOOLS
// Ganho do filtro dq NUMA harmônica. id/iq saem de um passa-baixa de 1ª ordem
// AMOSTRADO a 1 kHz (P_CUR_TF), e cada harmônica o atravessa com atenuação
// diferente — a 20 km/h a 6ª elétrica está em ~940 Hz. Comparar as harmônicas
// cruas seria comparar coisas atenuadas de formas diferentes, e a 6ª
// apareceria pequena por artefato. Acima de 500 Hz ela chega REBATIDA, o que
// não estraga a medida (sinal e referência saem do mesmo ângulo amostrado e
// rebatem juntos) mas muda o ganho: o certo é o do filtro DISCRETO na
// frequência rebatida. H(z) = a/(1-(1-a)z^-1) em z = e^(j*wps).
static float dqFiltGain(float wps, float alpha) {
  const float re = 1.0f - (1.0f - alpha) * cosf(wps);
  const float im = (1.0f - alpha) * sinf(wps);
  return alpha / fmaxf(sqrtf(re * re + im * im), 1e-6f);
}
static float wrapPi(float x) {
  while (x >  PI) x -= 2.0f * PI;
  while (x < -PI) x += 2.0f * PI;
  return x;
}
// Separável só se couberem 2 ciclos da frequência REBATIDA na janela. Menos
// que isso e a demodulação não distingue a harmônica do próprio nível médio.
static bool harmOk(float dPerSample, int n) {
  return (float)n * fabsf(wrapPi(dPerSample)) > 4.0f * PI;
}
// "--" = nesta rotação o teste não separa. "~" = existe, mas abaixo de 2x o
// piso do estimador, ou seja, indistinguível de ruído. Nunca imprime 0 no
// lugar: zero diria "não existe", e a verdade é "não dá para saber".
static const char *hFmt(char *b, float v, float piso) {
  if (v < 0.0f)          strcpy(b, "   --");
  else if (v < 2.0f * piso) snprintf(b, 8, "~%4.2f", (double)v);
  else                      snprintf(b, 8, "%5.2f", (double)v);
  return b;
}

// Seis faixas, não oito: numa roda LIVRE 8 A levam de 0 a 20 km/h em ~0,4 s
// (a inércia da roda é uma fração da do patinete carregado). Faixa estreita
// demais não junta amostra nenhuma — foi por isso que a primeira corrida
// mediu 3 subidas e não imprimiu uma linha sequer.
// Quatro faixas e teste longo. A subida em vazio dura ~0,25 s: e TUDO o que a
// bancada da de dado com carga, porque a unica carga e a inercia da roda. Ou
// se aceita faixa larga e varias subidas, ou nao ha estatistica nenhuma — foi
// por isso que quase tudo saiu marcado como ruido na primeira corrida boa.
#define SW_BINS         4
#define SW_MIN_N       40
#define SWEEP_TOTAL_MS 15000
#define SW_COAST_MS     6000
// O 4o slot e uma SENTINELA: demodula em 3x eletrica, onde nao pode existir
// nada. Ordem 3 no dq exigiria 2a ou 4a harmonica no referencial parado, e
// harmonica PAR nao existe em maquina trifasica com simetria de meia onda.
// Entao o que a sentinela mede e puro ruido — mas o ruido DE VERDADE, com a
// correlacao que ele tem. O piso teorico (1,77*sigma/raiz(n)) supoe ruido
// branco, e id/iq saem de um filtro de 5 ms: as amostras nao sao
// independentes, o n efetivo e uma fracao do n, e o piso teorico SUBESTIMA.
// Foi isso que deixou um "40x acima do piso" variar 5x entre corridas.
struct SwBin {
  int    n;
  double sk, sq, sq2, sd, sd2, sm, sb, sv;
  double hc[4],  hs[4],  rc[4], rs[4];   // id  (3 harmonicas + sentinela)
  double hcq[4], hsq[4];                 // iq — TORQUE, o que se sente
};
// [0] SUBIDA (com corrente) e [1] DESCIDA (roda livre, corrente ~0). A descida
// não é tempo perdido: ela é a medida mais limpa que existe das harmônicas que
// NÃO dependem de carga — o desequilíbrio de ZERO entre os canais é um valor
// fixo no referencial parado, então aparece igual com 8 A ou com 0 A. O que
// só existe na subida é o que depende de corrente (ganho desigual, comutação
// sob torque). Comparar as duas tabelas já separa metade das causas.
static SwBin s_bin[2][SW_BINS];

// Fecha uma tabela: imprime as faixas com amostras que bastem e devolve as
// harmônicas compensadas. Retorna o índice da faixa de maior corrente, ou -1.
static int swReport(const char *tag, SwBin *set, float vlim, float alpha,
                    float bh[][3], float bp[][3], float qh[][3], float qp[][3],
                    float *bi1, float *bkm, int *bn) {
  int best = -1;
  for (int i = 0; i < SW_BINS; i++) {
    SwBin &g = set[i];
    bi1[i] = -1.0f; bn[i] = g.n;
    if (g.n < SW_MIN_N) continue;
    const float km = g.sk / g.n, iq = g.sq / g.n, id = g.sd / g.n;
    const float sD = sqrtf(fmaxf((float)(g.sd2 / g.n - (double)id * id), 0.0f));
    const float sQ = sqrtf(fmaxf((float)(g.sq2 / g.n - (double)iq * iq), 0.0f));
    bkm[i] = km; bi1[i] = hypotf(id, iq);
    const float we = kmhToRads(km) * (float)MOTOR_POLE_PAIRS * 0.001f;
    const float mul[4] = { 1.0f, 2.0f, 6.0f, 3.0f };   // [3] = sentinela
    float rawD[4], rawQ[4], gain[4];
    for (int k = 0; k < 4; k++) {
      const float d = we * mul[k];
      gain[k] = dqFiltGain(wrapPi(d), alpha);
      const double cx = g.hc[k]  - (double)id * g.rc[k];
      const double sx = g.hs[k]  - (double)id * g.rs[k];
      const double cq = g.hcq[k] - (double)iq * g.rc[k];
      const double sq = g.hsq[k] - (double)iq * g.rs[k];
      rawD[k] = 2.0f * (float)sqrt(cx * cx + sx * sx) / g.n;   // SEM compensar
      rawQ[k] = 2.0f * (float)sqrt(cq * cq + sq * sq) / g.n;
    }
    for (int k = 0; k < 3; k++) {
      const float d  = we * mul[k];
      const bool  ok = harmOk(d, g.n) && harmOk(we * 3.0f, g.n);
      // Piso = o MAIOR entre o teorico (ruido branco) e o empirico (o que a
      // sentinela mediu, trazido para o ganho desta harmonica). O empirico
      // vence sempre que o ruido e correlacionado — que e o caso aqui.
      bp[i][k] = fmaxf(1.77f * sD / sqrtf((float)g.n), rawD[3]) / gain[k];
      qp[i][k] = fmaxf(1.77f * sQ / sqrtf((float)g.n), rawQ[3]) / gain[k];
      bh[i][k] = ok ? rawD[k] / gain[k] : -1.0f;
      qh[i][k] = ok ? rawQ[k] / gain[k] : -1.0f;
    }
    char f1[8], f2[8], f6[8], q1[8], q2[8], q6[8];
    mcPrintf("[var] %s | %5.1f km/h n=%-5d iq %5.2f id %5.2f |I| %4.2f "
             "Ibat %4.2f Vq %4.1f (%.0f%%)\n", tag, km, g.n, iq, id,
             (float)(g.sm / g.n), (float)(g.sb / g.n), (float)(g.sv / g.n),
             100.0f * (float)(g.sv / g.n) / vlim);
    mcPrintf("[var]        id 1x %s 2x %s 6x %s | iq 1x %s 2x %s 6x %s "
             "[piso %.2f/%.2f]\n",
             hFmt(f1, bh[i][0], bp[i][0]), hFmt(f2, bh[i][1], bp[i][1]),
             hFmt(f6, bh[i][2], bp[i][2]), hFmt(q1, qh[i][0], qp[i][0]),
             hFmt(q2, qh[i][1], qp[i][1]), hFmt(q6, qh[i][2], qp[i][2]),
             bp[i][2], qp[i][2]);
    if (best < 0 || bi1[i] > bi1[best]) best = i;
  }
  return best;
}

static void runSweep(float maxKmh, float ampMax) {
  if (!s_iphOk) { motorReportCmdStatus(CMDERR_NO_IPHASE); return; }
  const float vlim = (fsEffVLim > 1.0f) ? fsEffVLim : (float)VOLTAGE_LIMIT_DEFAULT;
  const float loKmh = 0.2f * maxKmh;
  mcPrintf("[var] rampas de %.1f A do repouso ate %.0f km/h, descendo em roda "
           "livre | teto %.1f V | ~%d s\n", ampMax, maxKmh, vlim,
           SWEEP_TOTAL_MS / 1000);
  mcPrintf("[var] so as SUBIDAS entram na conta, separadas em %d faixas de "
           "velocidade. RODA LIVRE.\n", SW_BINS);

  calExitVoltageMode();
  motor.voltage_limit = vlim;
  s_fwIdTarget = 0.0f;              // sem field weakening: mede o motor
  motor.enable();
  for (int i = 0; i < SW_BINS; i++) { s_bin[0][i] = SwBin(); s_bin[1][i] = SwBin(); }

  const float alpha = fminf(0.001f / fmaxf(prm(P_CUR_TF), 0.0002f), 1.0f);
  uint8_t stop = 0;                 // 0=fim normal 1=corrente 2=velocidade
  bool    up   = true;              // subindo (com corrente) ou descendo (livre)
  int     passes = 0, phaseMs = 0;
  float   u = 0.0f, magF = 0.0f, a0 = 0.0f;
  int     ms = 0;
  const uint32_t t0us = micros();
  // 1x mecânica: acumulada no teste inteiro. A 0,5-10 Hz não cabem dois ciclos
  // numa faixa de velocidade, e demodular meio ciclo é inventar.
  // id E iq. So o id nao responde a pergunta: torque e Kt*iq, entao a
  // ondulacao que se SENTE mora no iq. O id conta a historia magnetica (folga
  // de entreferro, ima mais fraco); o iq conta o solavanco.
  double mc = 0, msn = 0, mrc = 0, mrs = 0, mSum = 0, mSum2 = 0;
  double mcq = 0, msnq = 0, mSumQ = 0, mSum2Q = 0;
  // Sentinela mecânica em 2,5x a volta da roda: não é múltiplo inteiro, então
  // nenhum defeito mecânico mora ali. O que ela medir é o piso REAL.
  double gc = 0, gs = 0, gcq = 0, gsq = 0, grc = 0, grs = 0;
  int    mN = 0;

  noInterrupts();
  for (int i = 0; i < 6; i++) { s_secAcc[i] = 0; s_secCnt[i] = 0; }
  interrupts();

  while (ms < SWEEP_TOTAL_MS) {
    motor.loopFOC();
    motor.move(u);
    motor.voltage.d = s_udCmd;          // mesmo caminho da fatia normal
    if ((int32_t)(micros() - t0us) < (ms + 1) * 1000) continue;
    ms++;

    // Leitura com a MESMA projeção da fatia de 1 kHz — medir por outro caminho
    // aqui daria o diagnóstico de um sistema que não existe.
    phaseCurrentRead(&s_iph, motor.electrical_angle -
                     motor.shaft_velocity * (float)MOTOR_POLE_PAIRS *
                     ((s_commLeadUs + IPH_MEAS_AGE_US) * 1e-6f));
    const float vf = fabsf(radsToKmh(velFastRads()));

    // SOBE com corrente fixa, DESCE em roda livre. A descida é sem torque de
    // propósito: frear com corrente devolveria a energia da roda para a fonte,
    // e uma fonte de bancada não absorve — o barramento subiria. Custa tempo
    // de teste; não custa hardware.
    // Relógios de segurança das duas fases. Sem eles o teste degenera nos dois
    // extremos: se o teto de tensão chegar antes do teto pedido, a subida
    // nunca termina e o motor fica 8 s parado no topo com corrente ZERO (o
    // mesmo "sem carga" que estragou a primeira versão); e se a roda descer
    // devagar demais, a descida come o teste inteiro.
    if (up && (vf >= maxKmh || ms - phaseMs > 3000))  { up = false; passes++;
                                                        phaseMs = ms; }
    if (!up && (vf <= loKmh || ms - phaseMs > SW_COAST_MS)) { up = true;
                                                        phaseMs = ms; }
    const float tgt = up ? ampMax : 0.0f;
    u = runCurrentLoops(tgt, 0.001f, vlim);

    magF += 0.3f * (s_iph.imag - magF);
    if (magF > AUTOCAL_I_ABORT_A)  { stop = 1; break; }
    if (vf > maxKmh * 1.3f + 4.0f) { stop = 2; break; }

    if (ms == 1) a0 = motor.shaft_angle;
    // A mecânica entra sempre: ela não depende de haver torque.
    const float am = motor.shaft_angle - a0;
    const float cmm = cosf(am), smm = sinf(am);
    const float cg  = cosf(2.5f * am), sg = sinf(2.5f * am);
    mrc += cmm; mrs += smm;  grc += cg; grs += sg;
    mc  += s_iph.id * cmm; msn += s_iph.id * smm;
    mcq += s_iph.iq * cmm; msnq += s_iph.iq * smm;
    gc  += s_iph.id * cg;  gs   += s_iph.id * sg;
    gcq += s_iph.iq * cg;  gsq  += s_iph.iq * sg;
    mSum  += s_iph.id; mSum2  += (double)s_iph.id * s_iph.id;
    mSumQ += s_iph.iq; mSum2Q += (double)s_iph.iq * s_iph.iq;
    mN++;

    if (vf < loKmh) continue;           // abaixo da faixa medida não entra

    int bi = (int)((vf - loKmh) / (maxKmh - loKmh) * SW_BINS);
    bi = constrain(bi, 0, SW_BINS - 1);
    SwBin &g = s_bin[up ? 0 : 1][bi];
    g.n++;
    g.sk += vf;         g.sq  += s_iph.iq;
    g.sq2 += (double)s_iph.iq * s_iph.iq;
    g.sd += s_iph.id;   g.sd2 += (double)s_iph.id * s_iph.id;
    g.sm += s_iph.imag; g.sb  += ina226CurrentA();
    g.sv += motor.voltage.q;

    // --- demodulação síncrona ---------------------------------------------
    // Recorrência de ângulo duplo: 2 senos/cossenos por amostra em vez de 6.
    const float c1 = cosf(motor.electrical_angle);
    const float s1 = sinf(motor.electrical_angle);
    const float c2 = c1 * c1 - s1 * s1,  s2 = 2.0f * c1 * s1;
    const float c3 = c2 * c1 - s2 * s1,  s3 = s2 * c1 + c2 * s1;
    const float cc[4] = { c1, c2, c3 * c3 - s3 * s3, c3 };   // [3] = sentinela
    const float ss[4] = { s1, s2, 2.0f * c3 * s3,      s3 };
    for (int k = 0; k < 4; k++) {
      g.rc[k]  += cc[k];             g.rs[k]  += ss[k];
      g.hc[k]  += s_iph.id * cc[k];  g.hs[k]  += s_iph.id * ss[k];
      g.hcq[k] += s_iph.iq * cc[k];  g.hsq[k] += s_iph.iq * ss[k];
    }
  }

  calReleaseSpinning(vlim);                 // NUNCA disable() com a roda solta
  calExitVoltageMode();

  if (stop == 1) mcPrintf("[var] PAROU: corrente de fase passou de %.0f A\n",
                          (double)AUTOCAL_I_ABORT_A);
  if (stop == 2) mcPrintf("[var] PAROU: velocidade passou da guarda (%.0f km/h)"
                          " — glitch de Hall?\n", maxKmh * 1.3f + 4.0f);

  // ---- TABELAS -----------------------------------------------------------
  mcPrintf("[var] %d subida(s) completa(s)\n", passes);
  float bh[SW_BINS][3], bp[SW_BINS][3], qh[SW_BINS][3], qp[SW_BINS][3];
  float ch[SW_BINS][3], cp[SW_BINS][3], dh[SW_BINS][3], dp[SW_BINS][3];
  float bi1[SW_BINS], bkm[SW_BINS], ci1[SW_BINS], ckm[SW_BINS];
  int   bn[SW_BINS], cn[SW_BINS];
  mcPrintf("[var] SOBE = com corrente | DESC = roda livre, corrente ~0\n");
  mcPrintf("[var] piso = SENTINELA medida em 3xel/2,5xmec, onde nao existe "
           "sinal nenhum. So passa dele o que e real.\n");
  const int best = swReport("SOBE", s_bin[0], vlim, alpha, bh, bp, qh, qp,
                            bi1, bkm, bn);
  swReport("DESC", s_bin[1], vlim, alpha, ch, cp, dh, dp, ci1, ckm, cn);
  if (best < 0) {
    mcPrintf("[var] FALHOU: a subida nao juntou amostras (a roda arranca rapido "
             "demais em vazio). Tente 'varre %.0f 3'.\n", maxKmh);
    motorReportCmdStatus(CMDERR_CAL_FAIL);
    return;
  }
  // A descida cobre uma faixa de velocidade só (a roda não desce a tempo de
  // varrer as outras), então o cruzamento usa a faixa com MAIS amostras, não
  // a mesma da subida. É por isso que a comparação aparece com duas
  // velocidades diferentes ao lado.
  int cbest = -1;
  for (int i = 0; i < SW_BINS; i++)
    if (ci1[i] >= 0.0f && (cbest < 0 || cn[i] > cn[cbest])) cbest = i;

  // ---- geometria dos Halls (teste inteiro) -------------------------------
  uint32_t acc[6]; uint16_t cnt[6];
  noInterrupts();
  for (int i = 0; i < 6; i++) { acc[i] = s_secAcc[i]; cnt[i] = s_secCnt[i]; }
  interrupts();
  float mean[6], tot = 0; int have = 0;
  for (int i = 0; i < 6; i++) {
    mean[i] = cnt[i] ? (float)acc[i] / cnt[i] : 0.0f;
    if (cnt[i] >= 5) { tot += mean[i]; have++; }
  }
  float secDev = -1.0f, secDeg[6] = {0};
  if (have == 6) {
    secDev = 0.0f;
    for (int i = 0; i < 6; i++) {
      secDeg[i] = 360.0f * mean[i] / tot;
      secDev = fmaxf(secDev, fabsf(secDeg[i] - 60.0f));
    }
    mcPrintf("[var] setores de Hall [graus]: %.1f %.1f %.1f %.1f %.1f %.1f "
             "-> desvio max %.1f\n", secDeg[0], secDeg[1], secDeg[2],
             secDeg[3], secDeg[4], secDeg[5], secDev);
  } else {
    mcPrintf("[var] setores de Hall: bordas de menos para medir\n");
  }

  // ---- 1x mecânica -------------------------------------------------------
  float hMec = -1.0f, pMec = 0.0f, hMecQ = -1.0f, pMecQ = 0.0f;
  if (mN > 500) {
    const float turns = fabsf(motor.shaft_angle - a0) / (2.0f * PI);
    const float mD = (float)(mSum / mN), mQ = (float)(mSumQ / mN);
    const float sD = sqrtf(fmaxf((float)(mSum2  / mN - (double)mD * mD), 0.0f));
    const float sQ = sqrtf(fmaxf((float)(mSum2Q / mN - (double)mQ * mQ), 0.0f));
    const double gd = gc  - (double)mD * grc, gd2 = gs  - (double)mD * grs;
    const double gq = gcq - (double)mQ * grc, gq2 = gsq - (double)mQ * grs;
    pMec  = fmaxf(1.77f * sD / sqrtf((float)mN),
                  2.0f * (float)sqrt(gd * gd + gd2 * gd2) / mN);
    pMecQ = fmaxf(1.77f * sQ / sqrtf((float)mN),
                  2.0f * (float)sqrt(gq * gq + gq2 * gq2) / mN);
    if (turns > 3.0f) {
      const double cd = mc  - (double)mD * mrc, sd2 = msn  - (double)mD * mrs;
      const double cq = mcq - (double)mQ * mrc, sq2 = msnq - (double)mQ * mrs;
      hMec  = 2.0f * (float)sqrt(cd * cd + sd2 * sd2) / mN;
      hMecQ = 2.0f * (float)sqrt(cq * cq + sq2 * sq2) / mN;
      mcPrintf("[var] 1x mecanica (%.0f voltas): id %.3f A (piso %.3f) | "
               "iq %.3f A (piso %.3f)\n", turns, hMec, pMec, hMecQ, pMecQ);
    } else {
      mcPrintf("[var] 1x mecanica: voltas de menos para medir\n");
    }
  }

  // ---- VEREDITO ----------------------------------------------------------
  // Julga na faixa de MAIOR corrente fundamental: é onde as harmônicas que
  // dependem de carga existem de fato e onde a relação com o ruído é a melhor.
  mcPrintf("[var]\n[var] VEREDITO na faixa de maior corrente: %.1f km/h, "
           "%.2f A de fundamental\n", bkm[best], bi1[best]);
  if (secDev >= 0.0f)
    mcPrintf("[var]  Halls: desvio max %.1f graus (%s)\n", secDev,
             secDev < 1.5f ? "excelente" : secDev < 3.0f ? "aceitavel"
                                                         : "FORA DE ESQUADRO");
  // ---- CARGA x SEM CARGA: é isto que separa as causas --------------------
  // Cada harmônica tem uma dependência de corrente DIFERENTE, e é só por aí
  // que se distingue quem é quem. Ganho desigual entre canais e comutação sob
  // torque são PROPORCIONAIS à corrente: com a roda em roda livre têm de
  // sumir. Desequilíbrio de back-EMF entre as fases (sequência negativa) e
  // harmônica de back-EMF do próprio motor existem IGUAL com 0 A, porque quem
  // as produz é o ímã girando, não a corrente. A descida tem ordens de
  // grandeza mais amostras que a subida — quando as duas discordam, é nela
  // que se acredita.
  const char *hn[3] = { "1xel", "2xel", "6xel" };
  if (cbest >= 0) {
    // Utilizacao das DUAS pontas. Sem isso a comparacao confunde "existe sem
    // corrente" com "existe so no teto de tensao" — e as duas coisas nao tem
    // nada a ver: no teto a janela de amostragem da corrente e a mais
    // apertada que existe, entao harmonica que SO aparece ali e artefato de
    // medicao, nao propriedade do motor.
    const float uC = 100.0f * (float)(s_bin[0][best].sv / s_bin[0][best].n)
                     / vlim;
    const float uL = 100.0f * (float)(s_bin[1][cbest].sv / s_bin[1][cbest].n)
                     / vlim;
    mcPrintf("[var] CARGA (%.0f km/h, %.1f A, %.0f%%) x LIVRE (%.0f km/h, ~0 A,"
             " %.0f%%) — harmonicas do id:\n", bkm[best], bi1[best], uC,
             ckm[cbest], uL);
    for (int k = 0; k < 3; k++) {
      const bool okC = bh[best][k] >= 2.0f * bp[best][k];
      const bool okL = ch[cbest][k] >= 2.0f * cp[cbest][k];
      const char *v;
      if (!okC && !okL)      v = "nem uma nem outra passa do ruido";
      else if (okL && !okC && uL > 90.0f && uC < 80.0f)
        v = "so no TETO de tensao -> janela de amostragem, nao o motor";
      else if (okL && (!okC || ch[cbest][k] > 0.5f * bh[best][k]))
        v = "IGUAL SEM CORRENTE -> e do MOTOR (back-EMF), nao do controle";
      else if (okC && !okL)  v = "so com corrente -> ganho/comutacao";
      else                   v = "cai muito sem corrente -> depende de carga";
      char fa[8], fb[8];
      mcPrintf("[var]  %s carga %s livre %s  -> %s\n", hn[k],
               hFmt(fa, bh[best][k], bp[best][k]),
               hFmt(fb, ch[cbest][k], cp[cbest][k]), v);
    }
  }
  if (hMecQ > 2.0f * pMecQ)
    mcPrintf("[var]  1x MECANICA no iq = %.3f A (piso %.3f, %.0fx): ondulacao "
             "de TORQUE uma vez por volta da roda.\n", hMecQ, pMecQ,
             hMecQ / fmaxf(pMecQ, 1e-4f));
  else if (hMec > 2.0f * pMec)
    mcPrintf("[var]  1x mecanica so no id (%.3f A): entreferro/ima desigual, "
             "sem virar torque. Nao e o que se sente.\n", hMec);

  // Como a 6x evolui com a velocidade separa geometria de atraso.
  int lo = -1, hi = -1;
  for (int i = 0; i < SW_BINS; i++)
    if (bi1[i] >= 0.0f && bh[i][2] > 0.0f) { if (lo < 0) lo = i; hi = i; }
  // Só compara o que passou do piso nas DUAS pontas: dividir ruído por ruído
  // deu "x20 = ATRASO" e "x0.1 = geometria" em duas corridas seguidas.
  if (lo >= 0 && hi > lo && bh[lo][2] >= 2.0f * bp[lo][2]
      && bh[hi][2] >= 2.0f * bp[hi][2])
    mcPrintf("[var]  6xel de %.1f a %.1f km/h: x%.1f (%s)\n", bkm[lo], bkm[hi],
             bh[hi][2] / bh[lo][2],
             bh[hi][2] / bh[lo][2] > 2.0f ? "cresce = ATRASO"
                                          : "estavel = geometria");
  motorReportCmdStatus(CMDERR_CAL_OK);
}
#endif

// ---------------------------------------------------------------------------
// TESTE GIRANDO — mede a ondulação de VELOCIDADE, que é o que se sente
// ---------------------------------------------------------------------------
// O degrau com a roda presa já disse tudo que podia: a malha regula e a
// ondulação de corrente que ele mostra é o próprio ruído do sensor (o número
// dele é PICO sobre 150 amostras; o do 'diag' é DESVIO PADRÃO — 3,2x menor por
// definição, não por qualidade). Comparar os dois foi o que fez parecer que
// sobrava defeito onde não sobrava.
//
// O que sobrou — aceleração irregular, aspereza — só existe COM O ROTOR
// GIRANDO, e a grandeza correspondente é a ondulação de VELOCIDADE: torque
// irregular vira velocidade irregular numa roda solta. É também o único regime
// em que a interpolação de Hall faz alguma coisa; parado ela não age, e por
// isso o A/B com a roda presa deu 1,73 contra 1,74 A — não testava nada.
//
// O teste NÃO bloqueia o controle: ele vive dentro da fatia de 1 kHz e usa o
// caminho normal inteiro (rampa, malha de corrente, limitador de velocidade).
// O que se mede é o sistema real, não uma bancada paralela que só ele usa.
// RODA LIVRE — quem segura a velocidade é o limitador.
#if PHASE_CUR_ENABLED && IPH_DIAG_TOOLS
static void spinStart(float amps) {
  s_spinFail = 0; s_spinN = 0; s_spinT0 = millis(); s_spinSteady = 0;
  s_spinReqA = amps; s_spinPhase = SPIN_SPINUP;
  mcPrintf("[gira] ate %.1f km/h com no maximo %.1f A | ~2 s, RODA LIVRE\n",
           s_spinKmh, amps);
}

// Chamada no fim de cada fatia de controle.
static void spinTick() {
  const float kmh = fabsf(radsToKmh(velFastRads()));
  if (s_spinPhase == SPIN_SPINUP) {
    // Guarda de disparo: com a roda na bancada, velocidade muito acima do alvo
    // significa que o limitador não está segurando — corta em vez de insistir.
    if (kmh > s_spinKmh * 2.0f + 3.0f) {
      s_spinFail = 3; s_spinPhase = SPIN_ABORT; s_spinReqA = -1.0f; return;
    }
    // Captura assim que a velocidade assenta, em vez de esperar um tempo fixo:
    // na bancada o que importa é a roda girar o mínimo necessário.
    if (fabsf(kmh - s_spinKmh) < 0.15f * s_spinKmh) s_spinSteady++;
    else                                            s_spinSteady = 0;
    if (s_spinSteady >= SPIN_STEADY_MS || millis() - s_spinT0 >= SPIN_SPINUP_MS) {
      s_spinSettled = (s_spinSteady >= SPIN_STEADY_MS);
      s_spinPhase = SPIN_CAP; s_spinN = 0;
    }
  } else if (s_spinPhase == SPIN_CAP) {
    s_spinV[s_spinN] = (int16_t)constrain(radsToKmh(velEst) * 100.0f, -32000.0f, 32000.0f);
    s_spinI[s_spinN] = (int16_t)constrain(s_iph.iq * 100.0f, -32000.0f, 32000.0f);
    // Teto e alvo junto com a medida: sem os três lado a lado não dá para dizer
    // se o limitador não fechou, se fechou e a corrente não obedeceu, ou se os
    // dois estavam certos e a velocidade subiu assim mesmo.
    s_spinC[s_spinN] = (int16_t)constrain(s_spdLimCap * 100.0f, -32000.0f, 32000.0f);
    s_spinT[s_spinN] = (int16_t)constrain(s_iqTarget  * 100.0f, -32000.0f, 32000.0f);
    if (++s_spinN >= SPIN_N) {
      s_spinPhase = SPIN_COOL; s_spinReqA = -1.0f; s_spinT0 = millis();
    }
  } else if (s_spinPhase == SPIN_COOL) {
    // Espera a roda parar DE VERDADE, não só "quase": é nesse intervalo que o
    // zero da corrente se reassenta, e a passada seguinte depende dele. Sair
    // daqui cedo demais foi o que entregou a segunda passada com 2,6 A
    // fantasmas. Teto de 5 s para não travar se a roda ficar girando.
    if ((s_wheelStill && millis() - s_spinT0 >= SPIN_COOL_MS) ||
        millis() - s_spinT0 >= 5000)
      s_spinPhase = SPIN_DONE;
  }
}

// Calcula as métricas da captura. quiet = só preenche (o A/B imprime a tabela).
static void spinReport(bool quiet) {
  SpinRes &r = s_spinRes;
  r = SpinRes{};
  if (s_spinFail) {
    if (s_spinFail == 3)
      mcPrintf("[gira] ABORTADO: a roda passou de %.1f km/h — o limitador nao "
               "segurou.\n[gira]   e exatamente a falha do 'dispara com roda "
               "solta'. Rode 'step %.1f' com a roda PRESA:\n[gira]   se a malha "
               "de corrente estiver boa la, o problema e o limitador.\n",
               s_spinKmh * 2.0f + 3.0f, s_spinReqA > 0 ? s_spinReqA : 3.0f);
    else
      mcPrintf("[gira] ABORTADO: e-stop ou falta travada durante o teste\n");
    return;
  }
  double sv = 0, si = 0;
  for (int i = 0; i < SPIN_N; i++) { sv += s_spinV[i] / 100.0; si += s_spinI[i] / 100.0; }
  r.kmh = sv / SPIN_N; r.amps = si / SPIN_N;
  double qv = 0, qi = 0;
  for (int i = 0; i < SPIN_N; i++) {
    float dv = s_spinV[i] / 100.0f - r.kmh, di = s_spinI[i] / 100.0f - r.amps;
    qv += dv * dv; qi += di * di; r.pkKmh = fmaxf(r.pkKmh, fabsf(dv));
  }
  r.sdKmh = sqrtf(qv / SPIN_N); r.sdA = sqrtf(qi / SPIN_N);
  double sc = 0, st = 0;
  for (int i = 0; i < SPIN_N; i++) { sc += s_spinC[i] / 100.0; st += s_spinT[i] / 100.0; }
  r.cap = sc / SPIN_N; r.tgt = st / SPIN_N;
  r.feHz    = fabsf(kmhToRads(r.kmh)) * MOTOR_POLE_PAIRS / (2.0f * PI);
  r.settled = s_spinSettled;
  r.ok      = fabsf(r.kmh) > 1.0f && s_spinSettled;

  if (fabsf(r.kmh) > 1.0f && !s_spinSettled) {
    // Capturar no meio do transitório e chamar de "ondulação" foi o que produziu
    // a comparação de 4,25 contra 8,83 km/h. Sem regime não há número.
    mcPrintf("[gira] NAO ASSENTOU em %d s: %.2f km/h com teto de %.1f km/h.\n"
             "[gira]   teto de corrente %.2f A | alvo %.2f A | medida %.2f A\n"
             "[gira]   se o teto esta alto, o limitador nao fechou; se esta "
             "baixo e a medida nao, a malha de corrente nao obedeceu.\n",
             SPIN_SPINUP_MS / 1000, r.kmh, s_spinKmh, r.cap, r.tgt, r.amps);
    return;
  }

  if (!r.ok) {
    mcPrintf("[gira] O MOTOR NAO GIROU (%.2f km/h, %.2f A).\n"
             "[gira]   confira: roda LIVRE? calibracao ok ('diag')? "
             "corrente suficiente (tente 'gira 5')?\n", r.kmh, r.amps);
    return;
  }
  if (quiet) return;
  mcPrintf("[gira] velocidade %.2f km/h | ondulacao %.3f km/h (pico %.3f) = %.1f%%\n",
           r.kmh, r.sdKmh, r.pkKmh, 100.0f * r.sdKmh / fabsf(r.kmh));
  mcPrintf("[gira] corrente %.2f A (alvo %.2f, teto %.2f) | ondulacao %.2f A | "
           "eletrica %.0f Hz (comutacao %.0f Hz)\n",
           r.amps, r.tgt, r.cap, r.sdA, r.feHz, r.feHz * 6.0f);
}

#endif

// ---------------------------------------------------------------------------
// TASK PRINCIPAL — criada com xTaskCreatePinnedToCore(..., núcleo 1)
// ---------------------------------------------------------------------------
void motorControlTask(void *arg) {
  // Deixa o setup() do núcleo 1 terminar (esta task tem prioridade maior
  // que a loopTask — sem este delay o setup ficaria preso no meio).
  vTaskDelay(pdMS_TO_TICKS(100));

  // ---- ADCs ----------------------------------------------------------------
  analogReadResolution(12);
#if VBAT_SENSE_ENABLED
  vBatFilt = sensVbatV();            // task de sensores já está rodando
#endif
#if NTC_SENSE_ENABLED
  tempFilt = sensTempC();
  tempDisp = tempFilt;               // display nasce assentado, sem rampa
#else
  tempFilt = 0.0f;                   // 0 °C na dashboard = sensor desativado
  tempDisp = 0.0f;
#endif
#if MOTOR_NTC_SENSE_ENABLED
  motorTempFilt = sensMotorTempC();
  motorTempDisp = motorTempFilt;
#else
  motorTempFilt = 0.0f;
  motorTempDisp = 0.0f;
#endif
  Serial.printf("[motor] Vbat=%.1f V  Temp=%.0f C\n", vBatFilt, tempFilt);

  // ---- Driver 3-PWM ---------------------------------------------------------
  // voltage_power_supply é o NORMALIZADOR de duty da SimpleFOC (duty = U/Vs).
  // Nasce no nominal e passa a seguir o Vbat MEDIDO na fatia de controle —
  // fixá-lo em 36 V furava o teto de amostragem com o pack cheio (ver o
  // comentário na fatia). voltage_limit fica no nominal: é só um clamp de
  // sanidade do driver, o limite útil é motor.voltage_limit.
  driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
  driver.voltage_limit        = VOLTAGE_POWER_SUPPLY;
  driver.pwm_frequency        = (long)prm(P_PWM_FREQ);   // parâmetro do app
                                                          // (aplica no boot)
  int drvOk = driver.init();         // ESP32: usa MCPWM por baixo dos panos
  mcPrintf("[init] driver=%d PWM=%ld Hz (param %.0f) Vcc=%.1fV\n",
           drvOk, driver.pwm_frequency, prm(P_PWM_FREQ),
           driver.voltage_power_supply);
  motor.linkDriver(&driver);

  // ---- Sensor Hall (pulado no teste em malha aberta) ------------------------
#if !OPEN_LOOP_TEST
  hallSensor.pullup = Pullup::USE_EXTERN;  // GPIO 34..39 não têm pullup interno;
                                           // o conversor de nível já dirige o
                                           // sinal ativamente em 3,3 V
  hallSensor.init();
  hallSensor.velocity_max = 150.0f;  // [rad/s] rejeita velocidade absurda vinda
                                     // de par de bordas coladas (glitch residual);
                                     // máx. físico real ~80 rad/s (28 km/h)
  // Interrupções próprias (com filtro de glitch) em vez de enableInterrupts()
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_A), isrHallA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_B), isrHallB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_C), isrHallC, CHANGE);
#if HALL_INTERP
  motor.linkSensor(&smoothHall);   // Hall interpolado (ver HALL_INTERP)
#else
  motor.linkSensor(&hallSensor);
#endif
#endif

  // ---- Configuração do motor ------------------------------------------------
  motor.voltage_sensor_align = VOLTAGE_SENSOR_ALIGN;
  motor.voltage_limit        = VOLTAGE_LIMIT_DEFAULT;
  motor.velocity_limit       = kmhToRads(MAX_KMH_DEFAULT);
  motor.foc_modulation       = FOCModulationType::SpaceVectorPWM; // ~15% mais tensão útil
  // Modulação NÃO-centrada: a fase mais baixa fica em duty 0 (em vez das
  // três baterem em 50%). Com alvo 0 V => duties 0/0/0 = idêntico ao estado
  // desligado: some o chaveamento em repouso que queimava ~10 W e dava
  // "dentes" ao girar a roda na mão com o motor habilitado (diagnóstico
  // 23/jul: I=0,2-0,3 A parado ligado vs 0,0 A desligado, dc=0.5/0.5/0.5).
  // Bônus: low-side conduz mais => bootstraps do FD2103S recarregam melhor.
  motor.modulation_centered  = 0;

  motor.PID_velocity.P           = VEL_PID_P;
  motor.PID_velocity.I           = VEL_PID_I;
  motor.PID_velocity.D           = VEL_PID_D;
  motor.PID_velocity.output_ramp = VEL_PID_RAMP;
  motor.PID_velocity.limit       = IQ_MAX_DEFAULT_A;   // saída em AMPÈRES
  motor.LPF_velocity.Tf          = VEL_LPF_TF;   // Halls = baixa resolução => filtrar

#if OPEN_LOOP_TEST
  motor.controller    = MotionControlType::velocity_openloop;
  motor.voltage_limit = OPEN_LOOP_VOLTAGE_LIMIT;
  Serial.println("[motor] *** MODO TESTE MALHA ABERTA (OPEN_LOOP_TEST=1) ***");
#else
  motor.controller = MotionControlType::torque;
#if PHASE_CUR_ENABLED
  // ---- Corrente de fase: 2x INA240A1 low-side sincronizados ao MCPWM -------
  // Tem que vir DEPOIS de driver.init() (phaseCurrentInit lê a geometria do
  // timer MCPWM e registra a ISR nele) e ANTES de motor.initFOC().
  {
    CurrentSense *cs = phaseCurrentInit(&driver, &motor);
    if (!cs) {
      faults |= FAULT_SENSOR;
      Serial.println("[motor] ERRO: sensor de corrente de fase ausente — "
                     "malha de corrente impossivel. Motor BLOQUEADO.");
    } else {
      motor.linkCurrentSense(cs);
      // A malha de corrente SÓ é habilitada com calibração válida na NVS. Sem
      // ela não se sabe o sinal do sensor, e sinal invertido é realimentação
      // positiva: o motor dispara e não para. Prefiro o motor não ligar.
      s_iphOk = phaseCurrentCalValid();
      if (!s_iphOk) {
        faults |= FAULT_SENSOR;
        Serial.println("[motor] SEM CALIBRACAO da corrente de fase — motor "
                       "BLOQUEADO. Rode 'Calibrar corrente de fase' no app "
                       "com a RODA SUSPENSA.");
      }
    }
  }
  // Comutação por ÂNGULO em modo tensão — a que este motor executa bem. O
  // torque vem do regulador de corrente (currentRegulator), que produz o Uq
  // perseguindo a corrente medida. Ver o comentário do regulador.
  motor.torque_controller = TorqueControlType::voltage;
  // Imprimir os ganhos QUE ESTÃO VALENDO. Parâmetro salvo na NVS tem prioridade
  // sobre o config.h, então "mudei o default" não significa "está em uso" — e
  // sem isto no log não há como saber se um teste rodou com o ajuste certo.
  Serial.printf("[motor] malha de corrente EM USO: P=%.3f I=%.0f Tf=%.4f s "
                "(defaults do config: P=%.3f I=%.0f)\n",
                prm(P_CUR_P), prm(P_CUR_I), prm(P_CUR_TF),
                (double)CUR_PID_P, (double)CUR_PID_I);
  if (prm(P_CUR_P) > 2.0f * CUR_PID_P)
    Serial.println("[motor] AVISO: ganho da malha de corrente MUITO acima do "
                   "default — provavel valor antigo na NVS. Restaure os padroes "
                   "de fabrica no app, senao a malha oscila.");
#else
  motor.torque_controller = TorqueControlType::voltage;   // fallback de bancada
#endif
#endif

  motor.init();

#if !OPEN_LOOP_TEST
  // Calibração conhecida? Pula o giro de alinhamento do initFOC.
  // A base vem do PARÂMETRO persistido (default = MOTOR_ZERO_ELEC_ANGLE).
  if (!isnan(prm(P_ZEA))) motor.zero_electric_angle = prm(P_ZEA);
  // Direção do sensor: PARÂMETRO MEDIDO pela calibração 'zea' (fato de
  // fiação, como o mapa de fases dos INA240). O antigo MOTOR_SENSOR_DIR
  // forçado no config pulava a detecção — e uma direção chutada errada faz o
  // ângulo elétrico CONTRA-GIRAR em movimento: parado alinha, girando o id
  // dispara sem controle (foi exatamente o que a verificação do zea pegou).
  motor.sensor_direction = (prm(P_SENS_DIR) < 0.0f) ? Direction::CCW
                                                    : Direction::CW;
  // Inversão de sentido por PARÂMETRO (app -> aplica no próximo boot)
  if (prm(P_DIR_INVERT) > 0.5f)
    motor.sensor_direction = (motor.sensor_direction == Direction::CW)
                               ? Direction::CCW : Direction::CW;

  // ATENÇÃO: se a calibração não estiver fixada acima, o initFOC GIRA o motor
  // (roda suspensa + fonte limitada em corrente na primeira vez!).
  if (motor.initFOC()) {
    Serial.printf("[motor] initFOC OK. ZEA %.4f rad | direcao %s "
                  "(parametros persistidos — recalibre com 'zea')\n",
                  motor.zero_electric_angle,
                  motor.sensor_direction == Direction::CW ? "CW" : "CCW");
  } else {
    faults |= FAULT_SENSOR;
    Serial.println("[motor] ERRO: initFOC falhou! Confira Halls/fases. Motor bloqueado.");
  }
#endif

  // Começa SEMPRE desabilitado — só liga pelo botão da dashboard.
  motor.disable();
  Serial.println("[motor] Pronto. Aguardando enable pela dashboard.");
  writeCommand(readCommand());   // (no-op p/ garantir estado coerente pós-init)

  // ---- Loop de controle -----------------------------------------------------
  uint32_t lastSliceUs = micros();
  uint32_t lastRateMs  = millis();
  uint32_t loopCounter = 0;
  uint32_t prevIterUs  = micros();
  uint32_t maxGapUs    = 0;

  for (;;) {
    // ---- CALIBRAÇÕES REMOTAS (disparadas pelo app; rodam AQUI, fora da
    // fatia, porque bloqueiam segundos — roda SUSPENSA, exigido pelo app) ----
    if (calZeaReq) {
      calZeaReq = false;
      mcPrintf("[cal] direcao do sensor + angulo zero — RODA LIVRE "
               "(o motor gira uma volta eletrica p/ cada lado)\n");
      motor.disable(); appliedTarget = 0;
      float zeaOld = prm(P_ZEA);
      Direction dirOld = motor.sensor_direction;
      // A DIREÇÃO também é MEDIDA (varredura de uma volta elétrica em cada
      // sentido observando o acúmulo dos Halls). Chutá-la era o furo que a
      // verificação pegou: direção errada faz o ângulo contra-girar em
      // movimento — parado alinha, girando o id dispara sem controle.
      motor.sensor_direction    = Direction::UNKNOWN;
      motor.zero_electric_angle = NOT_SET;      // força realinhar
      motor.enable();
      if (!motor.initFOC()) {
        mcPrintf("[cal] FALHOU — confira Halls/roda livre\n");
        // O initFOC que falha pode deixar o angulo em NOT_SET (NaN) — e
        // comutar com NaN vira duty NaN no PWM. Restaura os anteriores SEMPRE.
        motor.zero_electric_angle = isnan(zeaOld) ? MOTOR_ZERO_ELEC_ANGLE : zeaOld;
        motor.sensor_direction    = dirOld;
        motor.disable();
        calExitVoltageMode();
        motorReportCmdStatus(CMDERR_CAL_FAIL);
      } else {
        float zeaNew = motor.zero_electric_angle;
        const int8_t dirMeas = (motor.sensor_direction == Direction::CW) ? 1 : -1;
        mcPrintf("[cal] medido: angulo %.4f rad | direcao %s (anterior: "
                 "%.4f, %s)\n", zeaNew, dirMeas > 0 ? "CW" : "CCW", zeaOld,
                 dirOld == Direction::CW ? "CW" : "CCW");

        // ---- VERIFICAÇÃO DE TORQUE ----------------------------------------
        // O initFOC devolve um número mesmo quando o rotor não pôde girar (roda
        // presa, fase aberta): ele lê o ângulo onde o rotor ESTÁ, não onde
        // deveria ter alinhado. Gravar isso sem conferir é como este firmware
        // ficou com 0,0000 no lugar de 2,0944 — 120° elétricos de erro, que
        // fazem o torque virar NEGATIVO e o motor lutar contra si mesmo.
        // O teste: com o ângulo certo, Iq positivo acelera para FRENTE.
#if PHASE_CUR_ENABLED
        calExitVoltageMode();
        motor.zero_electric_angle = zeaNew;
        // Teto de tensao CURTO para a verificacao: 3 A pedem ~2 V (R~0,2 ohm
        // + back-EMF baixa). 6,5 V limitam qualquer estado patologico a
        // ~30 A. O controlSlice restaura o teto normal ao retomar.
        motor.voltage_limit = 6.5f;
        motor.enable();
        // O regulador integra a 1 kHz DE VERDADE (passo cadenciado por
        // micros). Roda-lo a cada iteracao do loopFOC com dt fixo de 1 ms
        // multiplicava o I efetivo por 5-10x — o mesmo erro ja corrigido no
        // 'step' — deixava a malha tocando e podia RECUSAR um angulo certo.
        // E como este laco e bloqueante (as protecoes da fatia estao
        // paradas), a guarda de corrente tem de viver aqui dentro.
        // A guarda usa |I| FILTRADO (~3 ms) e o teste TERMINA assim que a
        // velocidade de aprovação é atingida. Motivo (aprendido na bancada):
        // com ângulo de Hall em degraus e Ud=0, o eixo d PULSA de verdade
        // dentro de cada setor — id_pico = E·sin(30°)/R cresce com a
        // velocidade e a ~11 rad/s os picos LEGÍTIMOS já encostam nos 15 A.
        // Comparar pico cru com o limiar abortava um ângulo CERTO; seguir
        // acelerando além da prova só fabricava o falso positivo.
        bool over = false;
        float u = 0.0f, magF = 0.0f;
        int ms = 0;
        const uint32_t t0us = micros();
        while (ms < 400) {
          motor.loopFOC();
          motor.move(u);
          if ((int32_t)(micros() - t0us) >= (ms + 1) * 1000) {
            ms++;
            phaseCurrentRead(&s_iph, motor.electrical_angle);
            u = currentRegulator(ZEA_VERIFY_A, s_iph.iq,
                                 motor.voltage_limit, 0.001f);
            magF += 0.3f * (s_iph.imag - magF);
            if (magF > AUTOCAL_I_ABORT_A) { over = true; break; }
            // já provou que acelera para frente: não há o que testar acima
            if (motor.shaft_velocity > 2.0f * ZEA_VERIFY_RADS) break;
          }
        }
        float vFwd = motor.shaft_velocity;
        motor.move(0.0f); motor.loopFOC();
        motor.disable();
        delay(600);                              // deixa parar

        mcPrintf("[cal] verificacao: %.1f A de Iq -> %.1f rad/s%s\n",
                 (double)ZEA_VERIFY_A, vFwd,
                 over ? "  [ABORTADA POR SOBRECORRENTE]" : "");
        if (!over && vFwd > ZEA_VERIFY_RADS) {
          paramSet(P_ZEA, zeaNew);
          // Persiste a direção MEDIDA. O boot aplica P_DIR_INVERT por cima,
          // então o valor gravado compensa o invert ativo — assim o próximo
          // boot reproduz exatamente o estado que esta verificação aprovou.
          paramSet(P_SENS_DIR, (prm(P_DIR_INVERT) > 0.5f) ? -dirMeas : dirMeas);
          paramsRequestSave();
          mcPrintf("[cal] OK: torque no sentido certo. angulo %.4f rad + "
                   "direcao %s SALVOS\n", zeaNew, dirMeas > 0 ? "CW" : "CCW");
          motorReportCmdStatus(CMDERR_CAL_OK);
        } else {
          if (over)
            mcPrintf("[cal] RECUSADO: |I| passou de %.0f A com iq regulado em "
                     "%.0f A — o eixo d disparou sem controle, assinatura de "
                     "angulo/direcao ainda errados OU fiacao Hall<->fase "
                     "incoerente.\n",
                     (double)AUTOCAL_I_ABORT_A, (double)ZEA_VERIFY_A);
          else
            mcPrintf("[cal] RECUSADO: Iq positivo nao acelerou para frente "
                     "(%.1f rad/s).\n", vFwd);
          mcPrintf("[cal] Mantidos os anteriores (%.4f rad, %s). Causas: roda "
                   "PRESA (exige roda LIVRE), fase aberta, ou fiacao de "
                   "Hall/fase trocada.\n",
                   zeaOld, dirOld == Direction::CW ? "CW" : "CCW");
          if (!isnan(zeaOld)) {
            paramSet(P_ZEA, zeaOld);
            motor.zero_electric_angle = zeaOld;
          } else {
            // sem anterior valido: fica o medido em RAM (nunca NaN no PWM)
            motor.zero_electric_angle = zeaNew;
          }
          motor.sensor_direction = dirOld;
          motorReportCmdStatus(CMDERR_CAL_FAIL);
        }
#else
        paramSet(P_ZEA, zeaNew);
        paramsRequestSave();
        motorReportCmdStatus(CMDERR_CAL_OK);
#endif
        motor.disable();
        calExitVoltageMode();
      }
    }
    if (iphCalReq) {
      iphCalReq = false;
#if PHASE_CUR_ENABLED
      if (motor.enabled || fabsf(radsToKmh(velEst)) > 0.5f) {
        motorReportCmdStatus(CMDERR_NEED_STOP);
      } else {
        s_iphOk = phaseCurrentCalibrate();
        if (s_iphOk) {
          // Calibração nova = sensor confiável de novo: a falta de sensor sai
          // e a malha de corrente pode voltar a ser habilitada.
          faults &= ~FAULT_SENSOR;
    currentRegulatorReset(0.0f);
          s_iqTarget = 0; fbBadMs = 0;
        } else {
          faults |= FAULT_SENSOR;
        }
        motorReportCmdStatus(s_iphOk ? CMDERR_CAL_OK : CMDERR_NO_IPHASE);
      }
#else
      motorReportCmdStatus(CMDERR_NO_IPHASE);
#endif
    }
#if PHASE_CUR_ENABLED && IPH_DIAG_TOOLS
    if (spinReq) {
      spinReq = false;
      const float a = constrain(spinAmps, 0.5f, 10.0f);
      s_spinKmh = constrain(spinKmh, 1.0f, 15.0f);
      if (!s_iphOk)                     motorReportCmdStatus(CMDERR_NO_IPHASE);
      else if (s_spinPhase != SPIN_OFF) mcPrintf("[gira] ja tem teste rodando\n");
      else { spinStart(a); motorReportCmdStatus(CMDERR_OK); }
    }
    if (s_spinPhase == SPIN_DONE || s_spinPhase == SPIN_ABORT) {
      s_spinPhase = SPIN_OFF;
      spinReport(false);
      if (s_spinRes.ok) mcPrintf("[gira] fim. o motor volta ao comando do app.\n");
    }
#endif
    if (iStepReq) {
      iStepReq = false;
#if PHASE_CUR_ENABLED
#if IPH_DIAG_TOOLS
      if (!s_iphOk) motorReportCmdStatus(CMDERR_NO_IPHASE);
      else if (fabsf(radsToKmh(velEst)) > 0.5f) motorReportCmdStatus(CMDERR_NEED_STOP);
      else { runCurrentStep(constrain(iStepAmps, 0.5f, 10.0f), iStepOpen);
             motorReportCmdStatus(CMDERR_OK); }
#else
      motorReportCmdStatus(CMDERR_BAD_STATE);
#endif
#endif
    }
    if (iphDiagReq) {
      iphDiagReq = false;
#if PHASE_CUR_ENABLED
      if (motor.enabled) motorReportCmdStatus(CMDERR_NEED_STOP);
      else { phaseCurrentDiag(5000); motorReportCmdStatus(CMDERR_OK); }
#endif
    }
#if PHASE_CUR_ENABLED && IPH_DIAG_TOOLS
    if (crashDumpReq) {
      crashDumpReq = false;
      if (motor.enabled) motorReportCmdStatus(CMDERR_NEED_STOP);
      else bbDump();
    }
    if (angScanReq) {
      angScanReq = false;
      if (motor.enabled || fabsf(radsToKmh(velEst)) > 0.5f)
        motorReportCmdStatus(CMDERR_NEED_STOP);
      else
        runAngleScan(constrain(angScanA, 1.0f, 8.0f),
                     constrain(angScanKmh, 3.0f, 12.0f));
    }
    if (sweepReq) {
      sweepReq = false;
      if (motor.enabled || fabsf(radsToKmh(velEst)) > 0.5f)
        motorReportCmdStatus(CMDERR_NEED_STOP);
      else
        runSweep(constrain(sweepKmh, 8.0f, 30.0f),
                 constrain(sweepA,   1.0f, 12.0f));
    }
#endif
    if (iphZeroReq) {
      iphZeroReq = false;
#if PHASE_CUR_ENABLED
      // Só faz sentido com o motor parado e desabilitado: o "zero" é medido
      // assumindo que a corrente real é zero. Com a roda girando, o motor
      // gerando ou o estágio habilitado, o offset sairia contaminado — e um
      // zero errado vira corrente fantasma permanente.
      if (motor.enabled || fabsf(radsToKmh(velEst)) > 0.5f) {
        motorReportCmdStatus(CMDERR_NEED_STOP);
      } else {
        bool z = phaseCurrentZero();
        // Zerar não libera a malha: quem libera é a calibração completa.
        s_iphOk = z && phaseCurrentCalValid();
        motorReportCmdStatus(z ? CMDERR_OK : CMDERR_NO_IPHASE);
      }
#else
      motorReportCmdStatus(CMDERR_NO_IPHASE);
#endif
    }
    if (calKeReq) {
      calKeReq = false;
      // RAMPA em vez de degrau seco (herdada da auto-cal removida, que nesta
      // parte era melhor): sobe até 6 V com teto por corrente MEDIDA, aborta
      // em sobrecorrente e corta na hora se não girar — nunca fica segurando
      // corrente num motor parado. Usa a tensão REALMENTE aplicada na conta.
      mcPrintf("[cal] medindo Ke: rampa ate 6 V, 4 s (RODA LIVRE!)...\n");
      calEnterVoltageMode();
      motor.voltage_limit = 6.5f;
      motor.enable();
      calRampReset();
      float vApplied = 0;
      bool  aborted  = false;
      uint32_t t0 = millis();
      while (millis() - t0 < 4000) {
        vApplied = calSpin(6.0f);
        if (autoCalOverCurrent()) { aborted = true; break; }
        if (millis() - t0 > 1200 && fabsf(motor.shaft_velocity) < 3.0f) {
          mcPrintf("[cal] Ke: motor NAO partiu — corte imediato\n");
          break;
        }
      }
      const float w = fabsf(motor.shaft_velocity);
      motor.move(0.0f);
      motor.disable(); appliedTarget = 0;
      if (!aborted && w > 8.0f && vApplied > 4.0f) {
        const float ke = vApplied / w * 1.05f;   // +5% de folga
        paramSet(P_KE, ke);
        paramsRequestSave();
        mcPrintf("[cal] Ke = %.3f V/(rad/s) (V=%.1f w=%.1f) — salvo\n",
                 ke, vApplied, w);
        motorReportCmdStatus(CMDERR_CAL_OK);
      } else {
        mcPrintf("[cal] FALHOU (%.1f rad/s @ %.1f V) — mantido o anterior\n",
                 w, vApplied);
        motorReportCmdStatus(CMDERR_CAL_FAIL);
      }
      calExitVoltageMode();
    }

    motor.loopFOC();               // comutação FOC (Halls por interrupção) +
                                   // bookkeeping de ângulo e velocidade
    motor.move(appliedTarget);     // alvo em VOLTS: quem converte corrente (e
                                   // velocidade, em cascata) para tensão é o
                                   // runCurrentLoops, na fatia de 1 kHz
#if !OPEN_LOOP_TEST
    motor.voltage.d = s_udCmd;     // o move() ZERA voltage.d a cada iteração;
                                   // a malha do eixo d reinjeta aqui — o
                                   // próximo loopFOC aplica Uq e Ud juntos
#endif
    loopCounter++;

    uint32_t nowUs = micros();

    // Detector de bloqueio: maior intervalo entre iterações no último segundo.
    // Se algo travar a task de controle (ms em vez de µs), aparece aqui —
    // e um buraco desses vira solavanco de torque.
    uint32_t gap = nowUs - prevIterUs;
    prevIterUs = nowUs;
    if (gap > maxGapUs) maxGapUs = gap;

    // Fatia de 1 kHz para o trabalho lento
    if (s_sliceResync) { s_sliceResync = false; lastSliceUs = nowUs;
                         prevAngleInit = false; }
    if (nowUs - lastSliceUs >= 1000) {
      float dt = (nowUs - lastSliceUs) * 1e-6f;   // dt real (pode passar de 1 ms)
      // TETO no dt. A fatia é de 1 ms; qualquer coisa acima de 50 ms é a task
      // tendo ficado bloqueada, e integrar o buraco inteiro num passo só joga
      // todo integrador no trilho de uma vez. O tempo parado não foi
      // controlado — não há o que integrar dele.
      if (dt > 0.05f) dt = 0.05f;
      lastSliceUs = nowUs;
      controlSlice(dt);
#if PHASE_CUR_ENABLED && IPH_DIAG_TOOLS
      spinTick();                       // teste 'gira'/'ab' anda junto da fatia
#endif
    }

    // Medição da frequência real do loop (diagnóstico na dashboard)
    uint32_t nowMs = millis();
    if (nowMs - lastRateMs >= 1000) {
      loopHz = loopCounter;
      loopCounter = 0;
      loopGapUs = (uint16_t)min((uint32_t)65535, maxGapUs);
      maxGapUs = 0;
      lastRateMs = nowMs;
    }
  }
}

// ---------------------------------------------------------------------------
// Pedidos de calibração remota (chamados pela task BLE, núcleo 0)
// ---------------------------------------------------------------------------
void motorRequestCalZea() { calZeaReq = true; }
void motorRequestCalKe()  { calKeReq  = true; }
void motorRequestIphZero() { iphZeroReq = true; }
void motorRequestIphCal()  { iphCalReq  = true; }
void motorRequestIphDiag() { iphDiagReq = true; }
void motorRequestCrash() {
#if PHASE_CUR_ENABLED && IPH_DIAG_TOOLS
  crashDumpReq = true;
#endif
}
void motorRequestIStep(float a) { iStepAmps = a; iStepOpen = false; iStepReq = true; }
void motorRequestVStep(float v) { iStepAmps = v; iStepOpen = true;  iStepReq = true; }
void motorRequestSpin(float a, float kmh) { spinAmps = a; spinKmh = kmh; spinReq = true; }
void motorRequestAngScan(float a, float kmh) {
  angScanA = a; angScanKmh = kmh; angScanReq = true;
}

void motorRequestSweep(float kmh, float amps) {
  sweepKmh = kmh; sweepA = amps; sweepReq = true;
}

// Forense da leitura dos Halls. Os contadores já existiam e nunca saíam no
// serial — e é exatamente o que decide se a aspereza sob carga é BORDA PERDIDA:
// uma borda que não chega deixa o ângulo 60° errado até a próxima, o que dá um
// solavanco irregular, dependente de corrente. Ler depois de andar.
void motorPrintHalls() {
  mcPrintf("[halls] bordas/s: total %u | A %u B %u C %u  (os tres tem de bater)\n",
           hallEps, epsA, epsB, epsC);
  mcPrintf("[halls] sequencia: pulos %u | inversoes %u | estado invalido %u\n",
           seqSkips, seqRevs, seqInvalid);
  mcPrintf("[halls] descartes por janela de %d us: A %u B %u C %u | nivel "
           "instavel: A %u B %u C %u\n", HALL_GLITCH_US,
           rejFiltA, rejFiltB, rejFiltC, rejConfA, rejConfB, rejConfC);
  mcPrintf("[halls] PULOS crescendo enquanto anda = borda perdida = comutacao "
           "60 graus errada por instante. E a causa mais provavel de aspereza "
           "que so aparece com carga.\n");
}

// ---------------------------------------------------------------------------
// API p/ o núcleo 0 (BLE): energia, captura de step response, amostra rápida
// ---------------------------------------------------------------------------
void motorEnergyInit() {
  Preferences pr; pr.begin("energy", false);
  odoM = pr.getULong("odo", 0); odoSavedM = odoM;
  capLearned = pr.getFloat("cap", 0.0f);
  if (capLearned <= 0) capLearned = prm(P_BATT_WH);   // semente: parâmetro
  mtpaTrim = constrain(pr.getFloat("mtpa", 0.0f), -MTPA_MAX, MTPA_MAX);
  pr.end();
  Serial.printf("[energy] odometro %.1f km | capacidade %.0f Wh\n",
                odoM / 1000.0f, capLearned);
}

// Persiste o odômetro (chamada periódica do núcleo 0; grava só parado e a
// cada >=100 m novos — flash congela os núcleos, nunca em movimento)
void motorEnergyTick(float kmhAbs) {
  if (kmhAbs > 2.0f) return;
  bool odoDue = (odoM - odoSavedM) >= 100;
  if (!odoDue && !capDirty && !mtpaDirty) return;
  Preferences pr; pr.begin("energy", false);
  if (odoDue)  { pr.putULong("odo", odoM); odoSavedM = odoM; }
  if (capDirty) { pr.putFloat("cap", capLearned); capDirty = false; }
  if (mtpaDirty) { pr.putFloat("mtpa", mtpaTrim); mtpaDirty = false; }
  pr.end();
}

void motorTripReset() { tripM = 0; tripWh = 0; tripWhRegen = 0; odoAcc = 0; }

// Step response: degrau de amplitude ampKmh (1..10) capturado a 1 kHz.
// Exige motor habilitado. RODA SUSPENSA — o app confirma antes.
bool motorStartStep(float ampKmh) {
  if (capState == 1 || !motor.enabled) return false;
  capBase    = velEst;
  capAmpRads = kmhToRads(constrain(ampKmh, 1.0f, 10.0f));
  capIdx = 0;
  capState = 1;
  return true;
}

// Lê amostras da captura (6 B cada: alvo, velocidade, corrente) p/ o BULK
int motorCaptureRead(uint16_t startSample, uint8_t *buf, int maxBytes) {
  if (capState != 2) return 0;
  int n = 0;
  uint16_t i = startSample;
  while (i < CAP_N && n + 6 <= maxBytes) {
    memcpy(buf + n,     (const void *)&capTgt[i], 2);
    memcpy(buf + n + 2, (const void *)&capVel[i], 2);
    memcpy(buf + n + 4, (const void *)&capCur[i], 2);
    n += 6; i++;
  }
  return n;
}
void motorCaptureFree() { if (capState == 2) capState = 0; }

// Amostra rápida p/ o STREAM BLE (leituras de word — sem lock)
void motorFastSample(float *kmh, float *ibat, float *vbat, float *target,
                     uint8_t *duty, int8_t *temp) {
  *kmh = fsKmh; *ibat = fsIbat; *vbat = fsVbat; *target = fsTarget;
  *duty = fsDuty; *temp = fsTemp;
}
void motorPerfSample(uint8_t *fw2, uint8_t *scale, int8_t *mtpa10,
                     uint8_t *head) {
  *fw2 = fsFw; *scale = fsScale; *mtpa10 = fsMtpa; *head = fsHead;
}
void motorCurSample(float *iqMeas, float *iqTarget) {
#if PHASE_CUR_ENABLED
  *iqMeas = s_iph.iq; *iqTarget = s_iqTarget;
#else
  *iqMeas = 0; *iqTarget = 0;
#endif
}
