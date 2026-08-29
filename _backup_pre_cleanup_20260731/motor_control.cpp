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
//   - O shunt é único, no barramento DC: NÃO dá para fazer FOC de corrente
//     por fase. Usamos torque por tensão + limite/proteção pela corrente
//     de bateria (foldback + trip).
// ============================================================================
#include <Arduino.h>
#include <math.h>
#include <SimpleFOC.h>
#include "config.h"
#include "shared_state.h"
#include "motor_control.h"
#include "ina226.h"
#include "params.h"
#include "curves.h"
#include "faultlog.h"
#include <Preferences.h>

// ---------------------------------------------------------------------------
// Objetos SimpleFOC
// ---------------------------------------------------------------------------
static BLDCMotor       motor(MOTOR_POLE_PAIRS);
static BLDCDriver3PWM  driver(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C);

#if HALL_INTERP_ENABLED
// Velocidade mecânica (rad/s) para a interpolação — atualizada a cada fatia
// pelo estimador próprio (velEst), já filtrado. Só interpola p/ frente.
static volatile float g_hallInterpVel = 0.0f;

// HallSensor que INTERPOLA o ângulo entre transições: em vez de segurar o
// setor (degrau de 60° elétricos), avança suave = comutação senoidal. Como
// pulse_timestamp é private na lib, detectamos a troca de setor aqui e
// carimbamos o tempo. Erro limitado a 1 setor => nunca pior que o cru.
class HallInterp : public HallSensor {
 public:
  HallInterp(int a, int b, int c, int pp) : HallSensor(a, b, c, pp) {}
  float getSensorAngle() override {
    long pos = electric_rotations * 6 + electric_sector;
    unsigned long now = micros();
    if (pos != lastPos_) { lastPos_ = pos; tSector_ = now; }
    float base = ((float)pos / (float)cpr) * _2PI;
    float v = g_hallInterpVel;
    if (v > HALL_INTERP_MIN_VEL) {
      float sector = _2PI / (float)cpr;                 // 1 setor [rad mec]
      float raw = v * (float)(now - tSector_) * 1e-6f;
      if (raw > sector) raw = sector;                   // não passa de 1 setor
      // CENTRADO na convenção da calibração: a média TEM que ser 'base' (o
      // valor que o zero_electric_angle espera). Sem o -meio_setor, a
      // interpolação empurra a fase +meio setor => erro de fase => torque
      // fraco, áspero, perde sincronismo. Com o -0,5 fica contínuo nas
      // transições (base+0,5 setor dos dois lados) e a fase média não muda.
      return base + raw - sector * 0.5f;
    }
    return base;
  }
 private:
  long lastPos_ = -999999;
  unsigned long tSector_ = 0;
};
static HallInterp      hallSensor(PIN_HALL_A, PIN_HALL_B, PIN_HALL_C, MOTOR_POLE_PAIRS);
#else
static HallSensor      hallSensor(PIN_HALL_A, PIN_HALL_B, PIN_HALL_C, MOTOR_POLE_PAIRS);
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

// Leitura de GPIO por registrador (segura e rápida dentro de ISR)
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
  uint8_t s = (uint8_t)((fastRead(PIN_HALL_A) << 2) |
                        (fastRead(PIN_HALL_B) << 1) |
                         fastRead(PIN_HALL_C));
  if (s == 0 || s == 7) { seqInvalid = seqInvalid + 1; return true; }
  return false;
}

static inline void IRAM_ATTR hallSeqCheck() {
  int8_t sec = ELECTRIC_SECTORS[hallSensor.hall_state & 0x07];
  if (sec < 0) { seqInvalid = seqInvalid + 1; seqPrevSector = -2; return; }
  if (seqPrevSector >= 0 && sec != seqPrevSector) {
    int8_t d = (int8_t)((sec - seqPrevSector + 6) % 6);
    if (d == 1)      { if (seqPrevDir == -1) seqRevs = seqRevs + 1; seqPrevDir =  1; }
    else if (d == 5) { if (seqPrevDir ==  1) seqRevs = seqRevs + 1; seqPrevDir = -1; }
    else             { seqSkips = seqSkips + 1; }   // pulou setor = leitura errada
  }
  seqPrevSector = sec;
}

// --- Mapa de duração de setor POR POSIÇÃO MECÂNICA (detector de defeito
// preso à posição: ímã fraco/danificado, hall deslocado) -------------------
// Uma volta da roda = 90 setores (15 ciclos elétricos x 6). Em velocidade
// constante TODOS devem durar o mesmo tempo. Um slot/grupo sempre fora =
// anomalia geométrica naquele ponto do rotor. Rode 10 s em velocidade
// constante e abra http://<ip>/hallmap (a leitura zera o mapa p/ novo teste).
static volatile uint32_t mapSumUs[90]  = {0};
static volatile uint16_t mapCnt[90]    = {0};
static volatile uint32_t lastAnyEdgeUs = 0;

static inline void IRAM_ATTR hallMapRecord(uint32_t now) {
  uint32_t d = now - lastAnyEdgeUs;
  lastAnyEdgeUs = now;
  if (d > 100000) return;               // muito lento/parado: fora do mapa
  int slot = (int)(((hallSensor.electric_rotations % 15) + 15) % 15) * 6
           + hallSensor.electric_sector;
  if (slot >= 0 && slot < 90) {
    mapSumUs[slot] = mapSumUs[slot] + d;
    mapCnt[slot]   = mapCnt[slot] + 1;
  }
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
  hallSensor.handleA();
  edgeCntA = edgeCntA + 1;   // taxa por pino p/ data-log (sempre)
  hallSeqCheck();                  // verificador de comutação (sempre)
#if HALL_DIAG
  hallMapRecord(now);
#endif
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
  hallSensor.handleB();
  edgeCntB = edgeCntB + 1;   // taxa por pino p/ data-log (sempre)
  hallSeqCheck();                  // verificador de comutação (sempre)
#if HALL_DIAG
  hallMapRecord(now);
#endif
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
  hallSensor.handleC();
  edgeCntC = edgeCntC + 1;   // taxa por pino p/ data-log (sempre)
  hallSeqCheck();                  // verificador de comutação (sempre)
#if HALL_DIAG
  hallMapRecord(now);
#endif
}

#if COMMANDER_ENABLED
// Interface serial p/ o SimpleFOC Studio (ajuste de PID/filtros + gráficos).
// O loop de controle reescreve alvo/enable/limites a 1 kHz — pelo Studio,
// mexa só em ganhos (PID, LPF) e monitoramento.
static Commander commander = Commander(Serial);
static void onMotorCmd(char *cmd) { commander.motor(&motor, cmd); }

// Ajuste AO VIVO do ângulo zero pela serial (achar o ponto de torque máximo
// sem regravar): digite p.ex. "Z4.19", "Z4.71", "Z3.66" no monitor serial e
// sinta qual dá torque mais forte/liso. "Z" sozinho mostra o valor atual.
// A calibração por Hall tem erro inerente de até ±30° elétricos (±0.52 rad) —
// teste o valor calibrado e ele ± 0.52. Fixe o melhor em MOTOR_ZERO_ELEC_ANGLE.
static void onZeroAngle(char *cmd) {
  if (cmd[0] != '\0') motor.zero_electric_angle = atof(cmd);
  mcPrintf("zero_electric_angle = %.4f rad\n", motor.zero_electric_angle);
}
#endif

// ---------------------------------------------------------------------------
// Estado local da task (não compartilhado — só o núcleo 1 toca aqui)
// ---------------------------------------------------------------------------
static float    curOffsetMv   = CUR_OFFSET_MV_DEFAULT; // offset do ADC de corrente a 0 A
static float    iBatFilt      = 0.0f;   // corrente filtrada [A] (proteções)
static float    iBatDisp      = 0.0f;   // corrente EXIBIDA (filtro lento)
static float    tempDisp      = 25.0f;  // temperatura EXIBIDA (filtro lento)
static float    vBatFilt      = 0.0f;   // tensão filtrada [V]
static float    tempFilt      = 25.0f;  // temperatura filtrada [°C]
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

// --- captura de 1 kHz p/ step response (BULK descarrega depois) --------------
#define CAP_N 1500
static int16_t capTgt[CAP_N], capVel[CAP_N], capCur[CAP_N];   // 9 KB
static volatile uint16_t capIdx = 0;
static volatile uint8_t  capState = 0;    // 0 livre | 1 capturando | 2 pronto
static float capAmpRads = 0, capBase = 0;
static float fwAdvRad = 0;    // avanço de fase ATUAL do field weakening [rad]

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
static float    coastRegV   = 0;

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
static inline float radsToRpm(float rads)  { return rads * 60.0f / (2.0f * PI); }

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

// ---------------------------------------------------------------------------
// Leituras analógicas (todas em ADC1 — compatível com WiFi ativo)
// analogReadMilliVolts() usa a calibração de fábrica do ESP32 (eFuse),
// bem melhor que converter a contagem crua na mão.
// ---------------------------------------------------------------------------
#if CUR_SENSE_ENABLED && !CUR_SENSE_INA226
static float readCurrentA() {
  float mv = (float)analogReadMilliVolts(PIN_ADC_CURRENT);
  float a = (mv - curOffsetMv) / CUR_MV_PER_A;
#if CUR_INVERT
  a = -a;
#endif
  return a;
}
#endif

// Leituras de tensão/NTC agora vêm da task de sensores no núcleo 0
// (sensVbatV/sensTempC em ina226.h): analogRead tem seção crítica interna
// e no núcleo 1 atrasava as ISRs dos Halls (skips em baixa rotação).

// Mede o offset do amplificador de corrente com o sistema em repouso.
// PRÉ-REQUISITO: motor parado e sem carga no boot.
static void calibrateCurrentOffset() {
#if CUR_SENSE_ENABLED && !CUR_SENSE_INA226 && CUR_AUTOZERO
  float acc = 0;
  for (int i = 0; i < 200; i++) {
    acc += (float)analogReadMilliVolts(PIN_ADC_CURRENT);
    delay(1);
  }
  curOffsetMv = acc / 200.0f;
  Serial.printf("[motor] Offset do ADC de corrente: %.1f mV\n", curOffsetMv);
#endif
}

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
  velEst += (dt / 0.08f) * (velRaw - velEst);
}

// ---------------------------------------------------------------------------
// Publica a telemetria para o núcleo 0 (dashboard)
// ---------------------------------------------------------------------------
static void publishTelemetry(const Command &cmd, bool linkOk, float scale) {
  Telemetry t;
  float vel     = velEst;   // estimador próprio: estável mesmo em baixa rotação
  t.rpm         = radsToRpm(vel);
  t.kmh         = radsToKmh(vel);
  t.ibat_a      = iBatDisp;      // exibição calma; proteções usam iBatFilt
  t.vbat_v      = vBatFilt;
  t.temp_c      = tempDisp;      // idem (tempFilt segue nas proteções)
  t.target      = appliedTarget;
  t.power_scale = scale;
  t.i_limit_a   = prm(P_ILIM_A);   // limites agora são parâmetros
  t.max_kmh     = (s_vmaxEffKmh > 0.5f) ? s_vmaxEffKmh : prm(P_MAX_KMH);
  t.v_limit     = prm(P_VLIM);
  t.loop_hz     = loopHz;
  t.mode        = curMode;
  t.enabled     = motor.enabled;
  t.open_loop   = (OPEN_LOOP_TEST != 0);
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
  t.range_km  = 0.0f;
  t.cap_wh    = (uint16_t)constrain(capLearned, 0.0f, 3000.0f);
  t.soc_pct   = (socLive >= 0) ? (uint8_t)(socLive * 100.0f + 0.5f) : 0;
  t.fw_x2     = (uint8_t)constrain(fwAdvRad * 57.2958f * 2.0f, 0.0f, 255.0f);
  t.mtpa_x10  = (int8_t)constrain(mtpaTrim * 57.2958f * 10.0f, -127.0f, 127.0f);
  t.eco_auto  = (ecoState == 1);
  // --- CORE em tempo real (data-log profundo) ---
  t.vel_est      = velEst;
  t.v_q          = motor.voltage.q;
  t.v_d          = motor.voltage.d;
  t.i_raw        = fsIRaw;
  t.eff_v_limit  = fsEffVLim;
  t.bemf_est     = prm(P_KE) * fabsf(velEst);
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
  t.coast_v      = coastRegV;
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
  t.hall_bad    = hallResyncs;
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
// Fatia de 1 kHz: ADCs, proteções, comando e cálculo do alvo
// ---------------------------------------------------------------------------
#if !BENCH_TEST
static void controlSlice(float dt) {
  sliceCount++;

  hallRateTick();               // bordas/s dos halls (sempre, p/ data-log)
#if HALL_DIAG
  hallDiagTick();
#endif

  // ---- 1) Leituras analógicas (escalonadas p/ não pesar na fatia) --------
#if CUR_SENSE_ENABLED
  // Corrente: toda fatia (1 kHz) — é a proteção mais crítica.
#if CUR_SENSE_INA226
  float iRaw = ina226CurrentA();   // lido a ~1 kHz pela task do núcleo 0
#else
  float iRaw = readCurrentA();
#endif
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
  }
#endif
#if NTC_SENSE_ENABLED
  if (sliceCount % 25 == 12) {
    float tc = sensTempC();
    tempFilt += 0.05f * (tc - tempFilt);                    // proteções (τ~0,5 s)
    tempDisp += (0.025f / TEMP_DISP_TAU_S) * (tc - tempDisp);   // display
  }
#endif

  // ---- 2) Proteções (com filtro temporal p/ imunidade a ruído) -----------
  uint16_t before = faults;

#if CUR_SENSE_ENABLED
  // Sobrecorrente (usa a leitura crua — reação rápida)
  if (fabsf(iRaw) > prm(P_OC_TRIP)) {
    ocMs += dt * 1000.0f;
    if (ocMs >= OC_TRIP_MS) faults |= FAULT_OVERCURRENT;
  } else ocMs = 0;
#else
  (void)iRaw;   // shunt desativado: sem proteção de sobrecorrente por firmware
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

#if NTC_SENSE_ENABLED
  // Sobretemperatura
  if (tempFilt > prm(P_T_CUT)) {
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

  // Limite de corrente efetivo (modo + walk/trava por cima do ajuste do app)
  float iLimEff = fminf(constrain(prm(P_ILIM_A), 1.0f, I_LIMIT_MAX_A), rideIlim);
  if (ovr == 2) iLimEff = fminf(iLimEff, WALK_ILIM_A);
  if (ovr == 1) iLimEff = fminf(iLimEff, LOCK_ILIM_A);
  float vmaxKmh = fminf(constrain(prm(P_MAX_KMH), 1.0f, MAX_KMH_ABS), rideVmax);
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

  // FIELD WEAKENING ADAPTATIVO por UTILIZAÇÃO DE TENSÃO (estilo VESC):
  // enquanto o controlador consegue produzir a tensão pedida, FW = 0.
  // Quando o PWM satura acima do joelho (P_FW_START, % de duty), um
  // integrador aplica avanço progressivamente até P_FW_MAX_DEG; com folga,
  // decai. Auto-adapta a bateria cheia/descarregada, motor e carga.
  // Só no TURBO. Sinal de P_FW_MAX_DEG invertível (se perder velocidade).
  // Freio/regen zera rápido (FW durante regen é indesejável).
  {
    float fwMax  = fwAllowed ? prm(P_FW_MAX_DEG) : 0.0f;
    float maxRad = fabsf(fwMax) * 0.0174533f;
    if (maxRad > 0.0001f && velEst > 1.0f && cmd.brake < 0.02f) {
      float knee = prm(P_FW_START);              // [% de duty] início do FW
      float err  = (float)fsDuty - knee;         // >0 = saturando
      // sobe até o máximo em ~1,5 s de saturação plena; solta 2x mais rápido
      fwAdvRad += err * (err > 0 ? 0.045f : 0.090f) * dt;   // [rad/s por % de erro]
      fwAdvRad  = constrain(fwAdvRad, 0.0f, maxRad);
    } else {
      fwAdvRad = fmaxf(0.0f, fwAdvRad - 2.0f * dt);   // sem condição: decai
    }
    float applied = (fwMax >= 0.0f) ? fwAdvRad : -fwAdvRad;
    motor.zero_electric_angle = prm(P_ZEA) - applied
                                - mtpaTrim - mtpaExplore;   // MTPA por busca
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
      // com a tensão que o torque aplicava AGORA. Com reset puro a saída
      // partia de 0 V = curto na back-EMF = parada instantânea (era isso
      // que fazia a soltura "agarrar" mesmo em malha de velocidade).
      velTarget = fmaxf(0.0f, velEst); appliedTarget = 0;
      static_cast<PIDSeed&>(motor.PID_velocity).seedOutput(
          constrain(motor.voltage.q, 0.0f, motor.PID_velocity.limit));
    } else {
      // volta ao torque partindo da tensão que o PID aplicava agora; a rampa
      // de subida (P_SLEW_UP) assume a partir daí
      motor.PID_velocity.reset();
      appliedTarget = fmaxf(0.0f, motor.voltage.q); velTarget = 0;
    }
    motor.controller = (curMode == MODE_VELOCITY) ? MotionControlType::velocity
                                                  : MotionControlType::torque;
    mcPrintf("[motor] Modo -> %s\n", curMode == MODE_VELOCITY ? "VELOCIDADE" : "TORQUE");
  }
#endif

  // ---- 5) Habilita/desabilita ---------------------------------------------
  // A TRAVA liga o motor sozinha: segurar a roda exige energia (PID em
  // velocidade 0, corrente limitada a LOCK_ILIM_A). Sem isso, travar com o
  // motor desligado só bloqueava o acelerador e a roda ficava livre.
  // E-STOP e faltas continuam mandando mais que a trava.
  bool wantEnable = (cmd.enable || lockActive)
                    && !cmd.estop && (faults & FAULT_LATCHING_MASK) == 0;
  if (wantEnable && !motor.enabled) {
    appliedTarget = 0; velTarget = 0; powerScale = 1.0f;
    motor.PID_velocity.reset();
    motor.enable();
    mcPrintf("[motor] HABILITADO\n");
  } else if (!wantEnable && motor.enabled) {
    motor.disable();                 // duty 0 => fases ao GND (freio suave parado)
    appliedTarget = 0; velTarget = 0;
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

  // Throttling térmico GRADUAL pela curva configurável (headroom = este valor)
  float tempScale = 1.0f;
#if NTC_SENSE_ENABLED
  tempScale = constrain(curveEval(CRV_THERMAL, tempFilt) * 0.01f, 0.0f, 1.0f);
  fsTempScale = (uint8_t)(tempScale * 100.0f);
#endif

  // Derate de TRAÇÃO por tensão/SoC (bateria fraca) + aviso progressivo
  float tracVScale = 1.0f;
#if VBAT_SENSE_ENABLED
  tracVScale = constrain(curveEval(CRV_TRAC_V, vBatFilt * 10.0f) * 0.01f, 0.0f, 1.0f);
  fsTracVScale = (uint8_t)(tracVScale * 100.0f);
  uvWarn = (vBatFilt > 5.0f && vBatFilt < prm(P_UV_WARN));
#endif

  updateVelEst(dt);
#if HALL_INTERP_ENABLED
  // Interpolação liga/desliga AO VIVO por parâmetro: vel=0 => HallInterp cai no
  // ângulo cru (base) sozinho, sem recompilar.
  g_hallInterpVel = (prm(P_HALL_INTERP) > 0.5f) ? velEst : 0.0f;
#endif

  float scale = fminf(powerScale, tempScale) * tracVScale;

  // Teto físico de modulação: acima de ~Vsupply/sqrt(3) (SVPWM) a senoide já
  // satura — pedir mais só distorce a forma de onda. Corta aqui.
#if VBAT_SENSE_ENABLED
  const float vModCeil = fmaxf(vBatFilt, 20.0f) * prm(P_MODCEIL);   // tensão REAL
#else
  const float vModCeil = VOLTAGE_POWER_SUPPLY * prm(P_MODCEIL);
#endif
  float userVLimit = constrain(prm(P_VLIM), 0.5f, fminf((float)VOLTAGE_LIMIT_MAX, vModCeil));
  float effVLimit  = userVLimit * scale;

  // LIMITE DE CORRENTE POR FEEDFORWARD (produção, com sensor): o teto de
  // tensão acompanha a back-EMF NA HORA + uma margem fixa (headroom). Limita
  // o PICO de corrente de forma suave e contínua; o foldback MEDIDO (INA226)
  // faz só o ajuste fino — acaba o liga-desliga que travava em ~15 km/h. Sem
  // slew (a rampa antiga atrasava atrás da back-EMF e dava aspereza).
  // P_HEADROOM ~ I_pico x R_fase => é o torque de arranque (parado, V=headroom).
  // Liga/desliga por P_ILIM_FF: é o ÚNICO limite de corrente de FASE em baixa
  // rotação (o foldback do INA226 só vê a corrente de BATERIA). Desligado =
  // confia só no foldback => mais torque bruto, porém risco de pico de fase.
  if (prm(P_ILIM_FF) > 0.5f)
    effVLimit = fminf(effVLimit, prm(P_HEADROOM) + prm(P_KE) * fabsf(velEst));
  fsEffVLim = effVLimit;                           // teto efetivo p/ data-log

#if OPEN_LOOP_TEST
  // Malha aberta: nunca deixar o limite da dashboard passar da tensão de teste
  effVLimit = fminf(effVLimit, OPEN_LOOP_VOLTAGE_LIMIT);
#endif

  // Aplica o limite efetivo nos dois caminhos (torque e PID de velocidade)
  motor.voltage_limit      = effVLimit;
  motor.PID_velocity.limit = effVLimit;

  // ---- 7) Cálculo do alvo -------------------------------------------------

#if OPEN_LOOP_TEST
  // Malha aberta: slider vira velocidade em rad/s, tensão fixa baixa.
  {
    float desired = (cmd.brake > 0.02f) ? 0.0f : cmd.throttle * OPEN_LOOP_MAX_RADS;
    appliedTarget = slewLimit(appliedTarget, desired, OPEN_LOOP_SLEW, OPEN_LOOP_SLEW * 3.0f, dt);
  }
#else
  if (curMode == MODE_TORQUE) {
    float desired;
    if (brkEff > 0.01f) {
      coastActive = false;
      ecoState = 0; ecoSteadyMs = 0;               // freio cancela o eco
      // Freio = tensão CONTRÁRIA ao sentido de giro (regen). Perto de parar,
      // alvo 0 — senão o motor aceleraria para trás. O foldback de regen
      // (P_REGEN_I via INA226) é quem limita a corrente de frenagem.
      float vBrakeMax = effVLimit;
      desired = (fabsf(velEst) > BRAKE_MIN_SPEED_RADS)
                  ? -sgnf(velEst) * brkEff * vBrakeMax
                  : 0.0f;
      appliedTarget = slewLimit(appliedTarget, desired,
                                prm(P_SLEW_UP) * rideSlew, prm(P_SLEW_REL), dt);
    } else if (prm(P_COAST) > 0.5f && thrEff < 0.06f && fabsf(velEst) > kmhToRads(2.0f)) {
      // COSTEIRA (P_COAST): soltar o acelerador = roda SOLTA, ZERO tração.
      // Desligada => cai no ramo normal com desejo ~0 e piso de back-EMF
      // (freio-motor levíssimo, sem regulador de corrente). Ao entrar,
      // a tensão CAI JÁ para abaixo da back-EMF (corta a tração no ato, sem
      // delay); depois o regulador sobe medindo a corrente até ela ir a ZERO
      // (INA226) — roda livre de verdade. Só o manete de freio freia.
      if (!coastActive) {
        coastActive = true;
        coastRegV = 0.85f * prm(P_KE) * fmaxf(velEst, 0.0f);  // corta tração já
      }
#if CUR_SENSE_ENABLED
      // alvo = -COAST_TARGET_A (regen levíssimo): garante ZERO tração mesmo
      // com offset do sensor. erro = iBat + alvo; >0 baixa V, <0 sobe V.
      coastRegV -= (iBatFilt + COAST_TARGET_A) * COAST_I_GAIN * dt;
#else
      coastRegV = 0.88f * prm(P_KE) * fmaxf(velEst, 0.0f);   // sem shunt: modelo
#endif
      // TETO DE SEGURANÇA: nunca acima da back-EMF estimada (0,97 Ke ω). Isso
      // GARANTE que a costeira nunca traciona, mesmo se o regulador de
      // corrente ou o sinal do INA estiverem errados — no pior caso, regen.
      float coastCeil = fminf(effVLimit, 0.97f * prm(P_KE) * fmaxf(velEst, 0.0f));
      coastRegV = constrain(coastRegV, 0.0f, coastCeil);
      appliedTarget = coastRegV;
    } else {
      coastActive = false;
      desired = thrEff * effVLimit;
      // Velocidade máxima por TETO DE TENSÃO (malha aberta — não oscila):
      // a tensão não passa da back-EMF correspondente à vel. máx + margem,
      // então o motor converge suavemente para o limite. O corte proporcional
      // antigo realimentava velocidade e, com a rampa assimétrica do alvo,
      // oscilava em surtos rítmicos exatamente ao encostar na vel. máx.
      float vmax = kmhToRads(vmaxKmh);
      desired = fminf(desired, prm(P_KE) * vmax + prm(P_CAP_MARGIN));

      // ---- ECO AUTOMÁTICO: só no modo ECO, cruzeiro estável ---------------
      // Detecta velocidade E acelerador estáveis por ECO_AUTO_ENGAGE_MS,
      // então assume o comando: desce a tensão bem devagar até o mínimo que
      // mantém a velocidade dentro de ECO_AUTO_TOL_KMH — elimina o
      // micro-ajuste humano (que desperdiça) e paga ~1 km/h pelo desconto.
      // QUALQUER gesto do piloto devolve o controle na hora.
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
            ecoVRef = velEst; ecoV = appliedTarget; ecoThrRef = thrEff;
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
            if (velEst > ecoVRef - kmhToRads(ECO_AUTO_TOL_KMH))
              ecoV -= ECO_AUTO_TRIM_V_S * dt;      // sobra folga: economiza
            else
              ecoV += ECO_AUTO_RECOV_V_S * dt;     // afundou: recupera
            ecoV = constrain(ecoV, 0.5f, effVLimit);
            desired = fminf(desired, ecoV);        // eco só REDUZ, nunca soma
          }
        }
      }
      // PISO de back-EMF: aliviar o acelerador desce a tensão só até perto
      // da costeira, NUNCA freia forte; soltar de vez cai no ramo da
      // COSTEIRA. Fator 0,85 = seguro (sempre abaixo da back-EMF real, nunca
      // empurra), no máximo regen levíssimo ao aliviar.
      desired = fmaxf(desired, 0.85f * prm(P_KE) * fmaxf(velEst, 0.0f));
      appliedTarget = slewLimit(appliedTarget, desired,
                                prm(P_SLEW_UP) * rideSlew, prm(P_SLEW_REL), dt);
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
    appliedTarget = velTarget;
  }
#endif

  if (!motor.enabled) { appliedTarget = 0; velTarget = 0; }

#if COMMANDER_ENABLED
  // Alimenta o Commander por um buffer próprio que aceita TANTO \n quanto \r
  // como fim de linha (o monitor serial costuma mandar \r\n e o Commander
  // original rejeita o \r reclamando). Poll a 50 Hz: Serial.available() pega
  // o mutex da UART (compartilhado com o núcleo 0) — não pertence ao 1 kHz.
  if (sliceCount % 20 == 0) {
    static char    cmdBuf[32];
    static uint8_t cmdLen = 0;
    while (Serial.available()) {
      char ch = (char)Serial.read();
      if (ch == '\r' || ch == '\n') {
        if (cmdLen > 0) { cmdBuf[cmdLen] = '\0'; commander.run(cmdBuf); cmdLen = 0; }
      } else if (cmdLen < sizeof(cmdBuf) - 1) {
        cmdBuf[cmdLen++] = ch;
      }
    }
  }
#endif

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

    float vq = fabsf(motor.voltage.q);
    float vbase = (vBatFilt > 5.0f ? vBatFilt : VOLTAGE_POWER_SUPPLY) * 0.577f;
    fsDuty   = (uint8_t)constrain(vq / vbase * 100.0f, 0.0f, 100.0f);
    fsKmh    = radsToKmh(velEst);
    fsIbat   = iBatFilt;
    fsVbat   = vBatFilt;
    fsTarget = appliedTarget;
    fsTemp   = (int8_t)constrain(tempFilt, -127.0f, 127.0f);
    fsFw     = (uint8_t)constrain(fwAdvRad * 57.2958f * 2.0f, 0.0f, 255.0f);
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
                    && fwAdvRad < 0.009f            // nunca junto com FW
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

  // ---- 7d) Diagnóstico serial (1 Hz) --------------------------------------
#if DIAG_SERIAL
  // Como ler: TRQ/VEL = malha ativa | ovr 1 trava 2 walk 3 cruise 4 step |
  // trava=1 com dentes na roda = comportamento da TRAVA (ela liga o motor).
  // en=1 alvo~0 vq~0 girando na mão = freio de curto da ponte (físico,
  // esperado). vq muito diferente de alvo = PID/limites agindo. I negativa
  // sem freio = regen indevida.
  if (sliceCount % 1000 == 0 && (motor.enabled || fabsf(velEst) > 0.5f)) {
    mcPrintf("[diag] %s ovr=%u en=%d trava=%d thr=%.2f brk=%.2f | alvo=%.2fV "
             "vq=%.2fV velTgt=%.1f | %.1fkm/h I=%.1fA scale=%.2f flt=0x%02X "
             "| mtpa=%+.1f fw=%.1f zea=%.3f rejF=%u/%u/%u\n",
             curMode == MODE_VELOCITY ? "VEL" : "TRQ", ovrNow,
             (int)motor.enabled, (int)lockActive, cmd.throttle, cmd.brake,
             appliedTarget, motor.voltage.q, velTarget,
             radsToKmh(velEst), iBatFilt, scale, faults,
             (mtpaTrim + mtpaExplore) * 57.2958f, fwAdvRad * 57.2958f,
             motor.zero_electric_angle, rejFiltA, rejFiltB, rejFiltC);
    // Duties REAIS por fase + eixo D: a 0 V os três TÊM que ser iguais
    // (curto simétrico). Diferentes = firmware aplicando vetor (bug);
    // iguais mas com dentes na roda = assimetria na PONTE (hardware).
    mcPrintf("[diag]   dc=%.3f/%.3f/%.3f vd=%.2f angEl=%.2f\n",
             driver.dc_a, driver.dc_b, driver.dc_c,
             motor.voltage.d, motor.electrical_angle);
  }
#endif

  // ---- 8) Telemetria a 20 Hz ----------------------------------------------
  if (sliceCount % 50 == 0) publishTelemetry(cmd, linkOk, scale);
}

#endif  // !BENCH_TEST

#if BENCH_TEST
// ---------------------------------------------------------------------------
// MODO BANCADA ISOLADO — fatia de 1 kHz mínima e SEM dependências externas:
// alvo FIXO em tensão, sem dead-man, sem comandos da dashboard (só o E-STOP
// passa quando BENCH_BLE=1). Serve para caçar os congelamentos (gap) com o
// mínimo de variáveis no caminho. RODA SUSPENSA — para parar: E-STOP (se
// houver WiFi) ou desligar a alimentação.
// ---------------------------------------------------------------------------
static bool benchStarted = false;
static bool benchEstop   = false;

static void benchSlice(float dt) {
  sliceCount++;
#if HALL_DIAG
  hallDiagTick();
#endif
  updateVelEst(dt);

  // E-STOP da dashboard (única entrada externa aceita) + telemetria a 20 Hz
  if (sliceCount % 50 == 0) {
    Command cmd = readCommand();
    if (cmd.estop && !benchEstop) {
      benchEstop = true;
      motor.disable();
      appliedTarget = 0;
      mcPrintf("[bench] E-STOP — reinicie a alimentacao para rodar de novo\n");
    }
    publishTelemetry(cmd, true, 1.0f);
  }
  if (benchEstop) { appliedTarget = 0; return; }

  // Partida automática após a contagem regressiva
  if (!benchStarted) {
    static uint32_t lastSec = 0;
    uint32_t sec = millis() / 1000;
    if (sec != lastSec && sec < BENCH_START_S) {
      lastSec = sec;
      mcPrintf("[bench] ligando em %lu s... (RODA SUSPENSA!)\n",
               (unsigned long)(BENCH_START_S - sec));
    }
    if (millis() >= (uint32_t)BENCH_START_S * 1000) {
      benchStarted = true;
      motor.enable();
      mcPrintf("[bench] MOTOR LIGADO — alvo fixo %.1f V\n", (double)BENCH_TARGET_V);
    }
  }

  // Segurança mínima 100% interna: teto de modulação + teto de corrente por
  // feedforward de back-EMF (o mesmo do controlSlice, se P_ILIM_FF ligado).
  float effVLimit = fminf((float)BENCH_TARGET_V, VOLTAGE_POWER_SUPPLY * MODULATION_CEIL_FACTOR);
  if (prm(P_ILIM_FF) > 0.5f)
    effVLimit = fminf(effVLimit, prm(P_HEADROOM) + prm(P_KE) * fabsf(velEst));
  motor.voltage_limit = effVLimit;
  appliedTarget = benchStarted
                    ? slewLimit(appliedTarget, effVLimit, TORQUE_SLEW_V_S,
                                TORQUE_RELEASE_V_S, dt)
                    : 0.0f;

  // Status na serial 1x/s (não-bloqueante): o "gap" é o número que caçamos
  if (sliceCount % 1000 == 500) {
    mcPrintf("[bench] hz=%lu gap=%uus vel=%.1fkm/h alvo=%.2fV skip=%lu inv=%lu\n",
             (unsigned long)loopHz, loopGapUs, (double)radsToKmh(velEst),
             (double)appliedTarget, (unsigned long)seqSkips,
             (unsigned long)seqInvalid);
  }
}
#endif  // BENCH_TEST

#if SELF_TEST
// ---------------------------------------------------------------------------
// AUTOTESTE FASES + HALLS — gira um vetor de tensão em malha aberta bem
// devagar e observa os Halls. Separa fase morta x hall ruim x calibração.
// Roda uma vez no boot (antes do initFOC) e imprime o veredito no serial.
// ---------------------------------------------------------------------------
// Teste de SIMETRIA da ponte: vetor DC parado em cada setor elétrico, corrente
// medida pelo INA226. Ponte saudável = 6 correntes iguais (mesmo R de
// enrolamento). Perna com MOSFET/driver fraco = corrente BAIXA nos 2 setores
// onde aquela fase domina (A: 0/180, C: 60/240, B: 120/300). RODA LIVRE!
static void runPhaseSymmetryTest() {
  Serial.println("[selftest] --- simetria da ponte (corrente por setor) ---");
  motor.enable();
  float iSec[6];
  for (int sct = 0; sct < 6; sct++) {
    float ang = sct * (PI / 3.0f);
    motor.setPhaseVoltage(0, SELF_TEST_I_V, ang);   // vetor DC (eixo D) parado
    delay(200);                                     // assenta rotor + filtro
    float acc = 0; int n = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 300) { acc += ina226CurrentA(); n++; delay(5); }
    iSec[sct] = acc / (float)n;
    Serial.printf("[selftest] setor %d (%3.0f gr): %.3f A\n",
                  sct, (double)(ang * 57.2958f), (double)iSec[sct]);
  }
  motor.setPhaseVoltage(0, 0, 0);
  motor.disable();

  float avg = 0, mn = iSec[0], mx = iSec[0]; int iMn = 0;
  for (int k = 0; k < 6; k++) {
    avg += iSec[k] / 6.0f;
    if (iSec[k] < mn) { mn = iSec[k]; iMn = k; }
    if (iSec[k] > mx)   mx = iSec[k];
  }
  Serial.printf("[selftest] media=%.3f A  min=%.3f  max=%.3f  desvio=%.0f%%\n",
                (double)avg, (double)mn, (double)mx,
                (double)((avg > 0.01f) ? (mx - mn) / avg * 100.0f : 0));
  if (avg < 0.05f) {
    Serial.println("[selftest] VEREDITO: corrente ~ZERO em todos os setores —");
    Serial.println("[selftest]   suba SELF_TEST_I_V ou ha problema geral de fase.");
  } else if ((mx - mn) / avg > 0.25f) {
    static const char *culp[6] = { "A", "C", "B", "A", "C", "B" };
    Serial.printf("[selftest] VEREDITO: ASSIMETRIA! Setor %d fraco -> perna %s\n",
                  iMn, culp[iMn]);
    Serial.println("[selftest]   MOSFET/FD2103S/bootstrap dessa fase suspeito.");
    Serial.println("[selftest]   (repita QUENTE: defeito termico so aparece quente)");
  } else {
    Serial.println("[selftest] VEREDITO: ponte SIMETRICA (6 setores iguais) —");
    Serial.println("[selftest]   MOSFETs/drivers OK nesta temperatura.");
  }
}

// ---------------------------------------------------------------------------
// TESTE CRU DE HARDWARE (HW_TEST): FOC básico + tensão fixa, nada mais.
// A régua é o INTERVALO ENTRE BORDAS DE HALL: em tensão constante e roda
// livre, ele é ~constante — estralo/travadinha aparece como intervalo
// esticado (comutação perdida / fase falhando) ou encolhido (borda falsa),
// impresso na hora com corrente e estado dos Halls. Relatório no fim.
// ---------------------------------------------------------------------------
#if HW_TEST && HALL_DIAG
static void runHardwareTest() {
  Serial.println("[hwtest] ===== TESTE CRU DE HARDWARE =====");
  Serial.printf("[hwtest] %.1f V fixos por %d s — RODA LIVRE!\n",
                (double)HW_TEST_V, HW_TEST_S);
  for (int k = 3; k > 0; k--) { Serial.printf("[hwtest] %d...\n", k); delay(1000); }

  motor.controller        = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.enable();

  uint32_t t0 = millis();
  uint32_t eA0 = edgeCntA, eB0 = edgeCntB, eC0 = edgeCntC;
  uint32_t lastEc = hallEdgeCount;
  uint32_t lastEdgeUs = micros(), lastLoopUs = micros();
  float    medMs = 0;                       // mediana móvel do intervalo
  float    intMin = 1e9f, intMax = 0, intSum = 0; uint32_t intN = 0;
  float    iMin = 1e9f, iMax = -1e9f, iSum = 0; uint32_t iN = 0;
  uint32_t anom = 0, loopGapMax = 0;
  bool     aborted = false;

  while (millis() - t0 < (uint32_t)HW_TEST_S * 1000) {
    uint32_t nowUs = micros();
    if (nowUs - lastLoopUs > loopGapMax) loopGapMax = nowUs - lastLoopUs;
    lastLoopUs = nowUs;

    motor.loopFOC();
    uint32_t el = millis() - t0;
    float v = (el < 1500) ? HW_TEST_V * (float)el / 1500.0f : HW_TEST_V;
    motor.move(v);

    // iBatFilt e filtrada pelo loop de controle (parado agora): le o INA cru
    float iNow = ina226CurrentA();
    bool steady = el > 2500;                // rampa + assentamento fora
    if (steady) {
      iSum += iNow; iN++;
      if (iNow < iMin) iMin = iNow;
      if (iNow > iMax) iMax = iNow;
    }

    uint32_t ec = hallEdgeCount;
    if (ec != lastEc) {
      uint32_t n = ec - lastEc;
      float dtMs = (float)(nowUs - lastEdgeUs) / 1000.0f / (float)n;
      lastEdgeUs = nowUs; lastEc = ec;
      if (steady) {
        intSum += dtMs * n; intN += n;
        if (dtMs < intMin) intMin = dtMs;
        if (dtMs > intMax) intMax = dtMs;
        if (medMs > 0.1f && (dtMs > 1.6f * medMs || dtMs < 0.5f * medMs)) {
          anom++;
          if (anom <= 20) {
            uint8_t hb = (uint8_t)((digitalRead(PIN_HALL_A) << 2) |
                                   (digitalRead(PIN_HALL_B) << 1) |
                                    digitalRead(PIN_HALL_C));
            Serial.printf("[hwtest] t=%.3fs ANOMALIA: intervalo %.2fms "
                          "(mediana %.2f) hall=%d%d%d I=%.2fA\n",
                          (double)el / 1000.0, (double)dtMs, (double)medMs,
                          (hb >> 2) & 1, (hb >> 1) & 1, hb & 1,
                          (double)iNow);
          }
        }
        medMs = (medMs < 0.1f) ? dtMs : medMs + 0.05f * (dtMs - medMs);
      }
    } else if (steady && (nowUs - lastEdgeUs) > 400000UL) {
      Serial.println("[hwtest] ABORTADO: motor PAROU no meio do teste!");
      aborted = true; break;
    }
    if (iNow > 10.0f) {
      Serial.printf("[hwtest] ABORTADO: sobrecorrente %.1f A\n", (double)iNow);
      aborted = true; break;
    }
    delayMicroseconds(200);
  }
  motor.move(0); motor.loopFOC();
  motor.disable();

  uint32_t dA = edgeCntA - eA0, dB = edgeCntB - eB0, dC = edgeCntC - eC0;
  Serial.println("[hwtest] ===== RELATORIO =====");
  Serial.printf("[hwtest] bordas A=%lu B=%lu C=%lu (devem ser iguais +-2%%)\n",
                (unsigned long)dA, (unsigned long)dB, (unsigned long)dC);
  if (intN > 0)
    Serial.printf("[hwtest] intervalo: med=%.2fms min=%.2f max=%.2f (n=%lu)\n",
                  (double)(intSum / intN), (double)intMin, (double)intMax,
                  (unsigned long)intN);
  if (iN > 0)
    Serial.printf("[hwtest] corrente: med=%.2fA min=%.2f max=%.2f\n",
                  (double)(iSum / iN), (double)iMin, (double)iMax);
  Serial.printf("[hwtest] anomalias=%lu | maior gap do loop=%lu us\n",
                (unsigned long)anom, (unsigned long)loopGapMax);

  uint32_t eMax = max(dA, max(dB, dC)), eMin = min(dA, min(dB, dC));
  if (aborted) {
    Serial.println("[hwtest] VEREDITO: FALHA GRAVE durante teste cru => HARDWARE.");
  } else if (anom == 0 && eMax - eMin <= eMax / 50 + 2) {
    Serial.println("[hwtest] VEREDITO: comutacao PERFEITA neste regime.");
    Serial.println("[hwtest]   Se o sintoma persiste na pilotagem, e dependente");
    Serial.println("[hwtest]   de CARGA/TEMPERATURA: repita o teste QUENTE e");
    Serial.println("[hwtest]   com a roda freada de leve na mao (carga).");
  } else if (eMax - eMin > eMax / 50 + 2) {
    Serial.printf("[hwtest] VEREDITO: fase de HALL desigual (A=%lu B=%lu C=%lu)\n",
                  (unsigned long)dA, (unsigned long)dB, (unsigned long)dC);
    Serial.println("[hwtest]   => canal com MENOS bordas esta perdendo leitura.");
  } else {
    Serial.println("[hwtest] VEREDITO: ANOMALIAS de comutacao em teste cru =>");
    Serial.println("[hwtest]   HARDWARE (fase/conector/FET). Veja os prints acima:");
    Serial.println("[hwtest]   intervalo esticado + corrente CAINDO = fase abriu;");
    Serial.println("[hwtest]   intervalo encolhido = borda falsa de Hall.");
  }
}
#endif  // HW_TEST && HALL_DIAG

static void runSelfTest() {
  runPhaseSymmetryTest();
  Serial.println("[selftest] ===== AUTOTESTE FASES + HALLS =====");
  Serial.printf("[selftest] vetor de %.1f V girando devagar — RODA LIVRE!\n",
                (double)SELF_TEST_V);
  motor.enable();

  uint8_t seen = 0, lastHb = 0xFF;
  int transitions = 0, fwd = 0, back = 0;
  int8_t lastSec = -2;
  const float step = 0.02f;                        // rad elétrico por passo
  const int total = (int)(4.0f * PI / step);       // 2 voltas elétricas

  for (int i = 0; i < total; i++) {
    motor.setPhaseVoltage(SELF_TEST_V, 0, i * step);
    delay(6);                                      // ~2 s por volta elétrica
    uint8_t hb = (uint8_t)((digitalRead(PIN_HALL_A) << 2) |
                           (digitalRead(PIN_HALL_B) << 1) |
                            digitalRead(PIN_HALL_C));
    if (hb == lastHb) continue;
    if (hb != 0 && hb != 7) {
      seen |= (uint8_t)(1 << (hb - 1));
      int8_t sec = ELECTRIC_SECTORS[hb];
      if (lastSec >= 0) {
        int8_t d = (int8_t)((sec - lastSec + 6) % 6);
        if (d == 1) fwd++; else if (d == 5) back++;
      }
      lastSec = sec;
      transitions++;
    }
    Serial.printf("[selftest] ang=%5.2f rad -> hall A=%d B=%d C=%d\n",
                  i * step, (hb >> 2) & 1, (hb >> 1) & 1, hb & 1);
    lastHb = hb;
  }
  motor.setPhaseVoltage(0, 0, 0);
  motor.disable();

  int nSeen = __builtin_popcount(seen);
  Serial.println("[selftest] ===== RESULTADO =====");
  Serial.printf("[selftest] transicoes=%d (esperado ~12) | estados=%d/6 | frente=%d re=%d\n",
                transitions, nSeen, fwd, back);
  if (transitions == 0) {
    Serial.println("[selftest] VEREDITO: rotor NAO SE MOVEU -> problema de FASE:");
    Serial.println("[selftest]   fase morta, FD2103S/MOSFET queimado, conector de");
    Serial.println("[selftest]   fase solto, ou tensao baixa (suba SELF_TEST_V p/ 4-5).");
    Serial.println("[selftest]   Os Halls NAO sao o problema neste cenario.");
  } else if (nSeen < 6) {
    Serial.printf("[selftest] VEREDITO: rotor girou mas so %d/6 estados de Hall\n", nSeen);
    Serial.printf("[selftest]   apareceram (mascara 0x%02X) -> problema de HALL:\n", seen);
    Serial.println("[selftest]   fio/canal do conversor do(s) estado(s) ausente(s).");
  } else if ((fwd > 1 && back > 1) || transitions < 9) {
    // direções MISTURADAS ou poucas transições = movimento errático de verdade
    Serial.println("[selftest] VEREDITO: girou IRREGULAR (vai-e-volta / pulos) ->");
    Serial.println("[selftest]   tipico de UMA FASE FRACA/MORTA: o rotor so 'pula'");
    Serial.println("[selftest]   entre pares de posicoes. Cheque solda/MOSFET/driver");
    Serial.println("[selftest]   de cada fase (compare resistencia fase-fase).");
  } else {
    // sequência completa e consistente numa direção só = hardware OK
    // (a direção é só convenção de fiação — o initFOC compensa)
    Serial.printf("[selftest] VEREDITO: FASES OK e HALLS OK — sequencia completa e\n");
    Serial.printf("[selftest]   uniforme no sentido %s da tabela.\n",
                  fwd >= back ? "NORMAL" : "REVERSO");
    Serial.println("[selftest]   Se a malha fechada nao parte, a CALIBRACAO esta");
    Serial.println("[selftest]   velha p/ a fiacao atual: MOTOR_ZERO_ELEC_ANGLE NAN");
    Serial.println("[selftest]   + MOTOR_SENSOR_DIR 0 por um boot e anote os novos.");
  }
  Serial.println("[selftest] =====================");
}
#endif  // SELF_TEST

// ---------------------------------------------------------------------------
// TASK PRINCIPAL — criada com xTaskCreatePinnedToCore(..., núcleo 1)
// ---------------------------------------------------------------------------
void motorControlTask(void *arg) {
  // Deixa o setup() do núcleo 1 terminar (esta task tem prioridade maior
  // que a loopTask — sem este delay o setup ficaria preso no meio).
  vTaskDelay(pdMS_TO_TICKS(100));

  // ---- ADCs ----------------------------------------------------------------
  analogReadResolution(12);
#if CUR_SENSE_ENABLED && !CUR_SENSE_INA226
  analogSetPinAttenuation(PIN_ADC_CURRENT, ADC_11db);  // faixa ~0..3,1 V
#endif

  calibrateCurrentOffset();          // motor parado no boot! (inerte se shunt off)
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
  Serial.printf("[motor] Sensores ativos: corrente=%d  vbat=%d  ntc=%d\n",
                CUR_SENSE_ENABLED, VBAT_SENSE_ENABLED, NTC_SENSE_ENABLED);
  Serial.printf("[motor] Vbat=%.1f V  Temp=%.0f C\n", vBatFilt, tempFilt);

  // ---- Driver 3-PWM ---------------------------------------------------------
  driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
  driver.voltage_limit        = VOLTAGE_POWER_SUPPLY;  // teto físico; o limite
                                                       // útil é motor.voltage_limit
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
  motor.linkSensor(&hallSensor);
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
  motor.PID_velocity.limit       = VOLTAGE_LIMIT_DEFAULT;
  motor.LPF_velocity.Tf          = VEL_LPF_TF;   // Halls = baixa resolução => filtrar

#if OPEN_LOOP_TEST
  motor.controller    = MotionControlType::velocity_openloop;
  motor.voltage_limit = OPEN_LOOP_VOLTAGE_LIMIT;
  Serial.println("[motor] *** MODO TESTE MALHA ABERTA (OPEN_LOOP_TEST=1) ***");
#else
  motor.controller        = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;  // sem shunt por fase =>
                                                         // torque por tensão
#endif

  motor.init();

#if SELF_TEST
  runSelfTest();   // roda uma vez e segue para a operação normal
#endif

#if COMMANDER_ENABLED
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 0;   // sem spam na serial; o Studio liga quando quiser
  commander.add('M', onMotorCmd, (char *)"motor");
  commander.add('Z', onZeroAngle, (char *)"zero elec angle");
  Serial.println("[motor] Commander ativo na serial — Studio (motor 'M') | Z<rad> ajusta angulo zero");
#endif

#if !OPEN_LOOP_TEST
  // Calibração conhecida? Pula o giro de alinhamento do initFOC.
  // A base vem do PARÂMETRO persistido (default = MOTOR_ZERO_ELEC_ANGLE).
  if (!isnan(prm(P_ZEA))) motor.zero_electric_angle = prm(P_ZEA);
#if MOTOR_SENSOR_DIR == 1
  motor.sensor_direction = Direction::CW;
#elif MOTOR_SENSOR_DIR == -1
  motor.sensor_direction = Direction::CCW;
#endif
  // Inversão de sentido por PARÂMETRO (app -> aplica no próximo boot)
  if (prm(P_DIR_INVERT) > 0.5f)
    motor.sensor_direction = (motor.sensor_direction == Direction::CW)
                               ? Direction::CCW : Direction::CW;

  // ATENÇÃO: se a calibração não estiver fixada acima, o initFOC GIRA o motor
  // (roda suspensa + fonte limitada em corrente na primeira vez!).
  if (motor.initFOC()) {
    Serial.printf("[motor] initFOC OK. ANOTE NO config.h -> "
                  "MOTOR_ZERO_ELEC_ANGLE %.4ff | MOTOR_SENSOR_DIR %d\n",
                  motor.zero_electric_angle,
                  motor.sensor_direction == Direction::CW ? 1 : -1);
  } else {
    faults |= FAULT_SENSOR;
    Serial.println("[motor] ERRO: initFOC falhou! Confira Halls/fases. Motor bloqueado.");
  }
#endif

#if HW_TEST && HALL_DIAG
  runHardwareTest();   // roda uma vez apos o initFOC e segue para a operacao
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
      mcPrintf("[cal] alinhamento do angulo zero (roda livre!)...\n");
      motor.disable(); appliedTarget = 0;
      motor.zero_electric_angle = NOT_SET;      // força realinhar
      motor.enable();
      if (motor.initFOC()) {
        paramSet(P_ZEA, motor.zero_electric_angle);
        paramsRequestSave();
        mcPrintf("[cal] zero_electric_angle = %.4f rad (salvo)\n",
                 motor.zero_electric_angle);
      } else {
        mcPrintf("[cal] FALHOU — confira Halls/roda livre\n");
      }
      motor.disable();
    }
    if (calKeReq) {
      calKeReq = false;
      mcPrintf("[cal] medindo Ke: 6 V fixos por 4 s (roda livre!)...\n");
      motor.enable();
      motor.voltage_limit = 6.5f;
      uint32_t t0 = millis();
      while (millis() - t0 < 4000) {            // regime permanente
        motor.loopFOC();
        motor.move(6.0f);
      }
      float w = fabsf(motor.shaft_velocity);
      motor.move(0.0f);
      motor.disable(); appliedTarget = 0;
      if (w > 8.0f) {
        float ke = 6.0f / w * 1.05f;            // +5% de folga
        paramSet(P_KE, ke);
        paramsRequestSave();
        mcPrintf("[cal] Ke medido = %.3f V/(rad/s) (salvo, c/ folga)\n", ke);
      } else {
        mcPrintf("[cal] FALHOU — motor nao girou (%.1f rad/s)\n", w);
      }
    }

    motor.loopFOC();               // comutação FOC (usa os Halls via interrupção)
    motor.move(appliedTarget);     // aplica o alvo (V ou rad/s conforme o modo)
#if COMMANDER_ENABLED
    // monitor() imprime na serial quando o Studio liga o streaming. Checa o
    // downsample ANTES de tocar na UART: availableForWrite() pega um MUTEX
    // compartilhado com o núcleo 0 — no caminho quente (~8 kHz) isso era um
    // canal de bloqueio cruzado (picos no gap). Com streaming desligado
    // (default), a UART nem é tocada.
    if (motor.monitor_downsample > 0 && Serial.availableForWrite() > 200)
      motor.monitor();
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
    if (nowUs - lastSliceUs >= 1000) {
      float dt = (nowUs - lastSliceUs) * 1e-6f;   // dt real (pode passar de 1 ms)
      lastSliceUs = nowUs;
#if BENCH_TEST
      benchSlice(dt);
#else
      controlSlice(dt);
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
// Relatório do mapa de setores (chamado pelo endpoint /hallmap, núcleo 0).
// Monta JSON com a duração média de cada um dos 90 slots + o pior desvio,
// e ZERA o mapa para o próximo teste. Interpretação: em velocidade constante,
// todos os slots ~iguais (±10-15%). Um slot/grupo sempre fora, nos MESMOS
// slots em leituras repetidas = defeito preso à posição do rotor.
// ---------------------------------------------------------------------------
int hallMapReport(char *buf, size_t n) {
  uint32_t sum[90];
  uint16_t cnt[90];
  for (int i = 0; i < 90; i++) {          // copia e zera (corrida com a ISR é
    sum[i] = mapSumUs[i];  mapSumUs[i] = 0;   // tolerável — é diagnóstico)
    cnt[i] = mapCnt[i];    mapCnt[i]   = 0;
  }

  uint32_t total = 0, edges = 0, mean[90];
  for (int i = 0; i < 90; i++) {
    mean[i] = cnt[i] ? sum[i] / cnt[i] : 0;
    total += sum[i];
    edges += cnt[i];
  }
  uint32_t avg = edges ? total / edges : 0;

  int worst = -1, worstDev = 0;
  for (int i = 0; i < 90; i++) {
    if (cnt[i] >= 3 && avg > 0) {
      int dev = (int)((100 * ((int64_t)mean[i] - (int64_t)avg)) / (int64_t)avg);
      if (abs(dev) > abs(worstDev)) { worstDev = dev; worst = i; }
    }
  }

  int off = snprintf(buf, n,
      "{\"edges\":%lu,\"avg_us\":%lu,\"worst_slot\":%d,\"worst_dev_pct\":%d,"
      "\"mean_us\":[",
      (unsigned long)edges, (unsigned long)avg, worst, worstDev);
  for (int i = 0; i < 90 && off < (int)n - 12; i++)
    off += snprintf(buf + off, n - off, i ? ",%lu" : "%lu", (unsigned long)mean[i]);
  off += snprintf(buf + off, n - off, "]}");
  return off;
}

// ---------------------------------------------------------------------------
// Pedidos de calibração remota (chamados pela task BLE, núcleo 0)
// ---------------------------------------------------------------------------
void motorRequestCalZea() { calZeaReq = true; }
void motorRequestCalKe()  { calKeReq  = true; }

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
uint8_t motorCaptureState() { return capState; }

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
