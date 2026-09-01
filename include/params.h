// ============================================================================
// params.h — parâmetros de TUNING ajustáveis pelo app, persistidos em NVS
// ----------------------------------------------------------------------------
// Os DEFAULTS são os #define do config.h (fonte da verdade de fábrica).
// O app lê o pacote (id, valor, default) pela characteristic PARAMS, edita,
// e ao APLICAR grava tudo + comando SAVE (o commit em flash espera a roda
// parar — escrita de NVS congela os núcleos por ms).
//
// Leitura no caminho de controle: prm(id) é um load de float alinhado
// (atômico no Xtensa) — sem lock, custo ~zero no loop FOC.
// ============================================================================
#pragma once
#include <Arduino.h>

enum : uint8_t {
  P_KE = 1,       // [V/(rad/s)] back-EMF. Não gera torque, MAS a guarda de
                  //     realimentação o usa para saber quando a tensão
                  //     simplesmente acabou (BEMF ≈ teto = topo de velocidade)
                  //     em vez de acusar sensor quebrado. Ke muito errado =
                  //     falso positivo ou guarda cega.
  P_IQ_MAX,       // [A] corrente de fase que o punho cheio pede
  P_MODCEIL,      // [-] teto de modulação (máx. 0.56 — bootstrap FD2103S!)
  P_SLEW_UP,      // [A/s] rampa do acelerador (alvo de corrente)
  P_SLEW_REL,     // [A/s] rampa ao soltar (alvo de corrente)
  P_ACCEL,        // [rad/s²] aceleração (modo velocidade)
  P_DECEL,        // [rad/s²] freio-motor ao soltar (modo velocidade)
  P_BRAKE_DEC,    // [rad/s²] desaceleração com freio 100%
  P_SPD_BAND,     // [A/(km/h)] ganho P do regulador de limite de velocidade
  P_PID_P,        // [-] PID velocidade P  (saída em AMPÈRES agora)
  P_PID_I,        // [-] PID velocidade I  (saída em AMPÈRES agora)
  P_LPF_TF,       // [s] filtro da velocidade medida
  P_OC_TRIP,      // [A] trip de sobrecorrente
  P_REGEN_I,      // [A] limite de corrente de regeneração
  P_UV_CUT,       // [V] corte de subtensão
  P_RESERVADO_16, // VAGO. Era P_T_DERATE (joelho do derate dos MOSFETs), que
                  //     virou letra morta quando o derate passou a sair da
                  //     CURVA térmica (CRV_THERMAL já traz o joelho). O slot
                  //     FICA: o protocolo BLE indexa parâmetro por id, e
                  //     renumerar aqui desalinharia em silêncio todo app que
                  //     não fosse atualizado no mesmo instante.
  P_T_CUT,        // [°C] corte térmico
  P_WHEEL_MM,     // [mm] diâmetro da roda
  P_FW_START,     // [% de UTILIZAÇÃO do teto real de tensão] joelho do FW
                  //     adaptativo (|U|/teto acima disto -> enfraquece). Era
                  //     % de duty absoluto: com o teto de operação em ~78%,
                  //     o joelho de 92% ficou inalcançável e o FW morreu.
  P_FW_MAX_A,     // [A] id NEGATIVO máximo de field weakening (0 = off).
                  //     Reprojetado p/ a era dos sensores de fase: alvo de id
                  //     direto na malha de d (o avanço de ângulo da era
                  //     back-EMF produzia id ∝ iq — inútil em roda livre — e
                  //     girava o frame de medição junto).
  P_SHUNT_MOHM,   // [mΩ] shunt do INA226 de BARRAMENTO (corrente de bateria)
  P_ZEA,          // [rad] ângulo zero elétrico (persistido; cal grava aqui)
  P_BATT_WH,      // [Wh] capacidade do pack (p/ autonomia estimada)
  P_DIR_INVERT,   // 0/1 inverte o sentido de giro (aplica no BOOT + recal)
  P_MTPA,         // 0/1 auto-otimização MTPA (busca do mínimo de corrente)
  P_ECO_AUTO,     // 0/1 eco automático no modo ECO (cruzeiro econômico)
  P_PWM_FREQ,     // [Hz] frequência do PWM (aplica no PRÓXIMO BOOT)
  P_MAX_KMH,      // [km/h] velocidade máxima (persistente; antes cmd volátil)
  P_ILIM_A,       // [A] corrente máxima (persistente; antes cmd volátil)
  P_VLIM,         // [V] tensão máxima do modo torque (persistente)
  P_UV_WARN,      // [V] aviso de bateria fraca (badge FRACA + derate começa)
  P_UV_RECOVER,   // [V] histerese para limpar a falta de subtensão
  P_ECO_VMAX,     // [km/h] teto de velocidade do modo ECO
  P_ECO_ILIM,     // [A] teto de corrente do modo ECO
  P_NOR_VMAX,     // [km/h] teto de velocidade do modo NORMAL
  P_NOR_ILIM,     // [A] teto de corrente do modo NORMAL
  P_BATT_VMAX,    // [V] tensão máxima do pack: regen corta seco acima disso
  // --- TOGGLES DE FEATURE (0/1; app esconde a feature quando 0) --------------
  P_IPH_TRIP,     // [A] trip duro por MAGNITUDE de corrente de fase |I|
  P_COAST,        // 0/1 costeira (roda-solta ao soltar = Iq alvo 0)
  P_BATT_LEARN,   // 0/1 bateria inteligente (aprende capacidade real do pack)
  P_TRAC_DERATE,  // 0/1 derate de tração por tensão/SoC (curva TRAC_V)
  P_THERM_DERATE, // 0/1 derate térmico gradual (curva THERMAL)
  P_MT_DERATE,    // [°C] início do derate térmico do MOTOR (NTC interno)
  P_MT_CUT,       // [°C] corte térmico do MOTOR (falta)
  // --- MALHA DE CORRENTE DE FASE (INA240 low-side) --------------------------
  P_CUR_P,        // [-] ganho P do PI de corrente (saída em volts)
  P_CUR_I,        // [-] ganho I do PI de corrente
  P_CUR_TF,       // [s] filtro de Iq/Id
  P_PH_SHUNT,     // [mΩ] shunt de FASE dos INA240
  P_PH_GAIN,      // [-] calibração fina da escala de corrente de fase
  P_SENS_DIR,     // ±1: direção do sensor Hall — MEDIDA pela calibração do
                  //     ângulo (zea), nunca chutada. Fato de fiação, como o
                  //     mapa de fases dos INA240. P_DIR_INVERT é a escolha do
                  //     usuário POR CIMA desta medida.
  P_COMM_LEAD,    // [µs] avanço de comutação: latência ângulo→tensão nos FETs
                  //     compensada extrapolando o ângulo do Hall interpolado.
                  //     Console 'lead <us>'. Ver COMM_LEAD_US_DEFAULT.
  P_UD_MAX,       // [0..0.5] fração do teto de tensão que a malha do eixo d
                  //     pode usar. É o A/B decisivo quando a suspeita é
                  //     "o d está bombeando": 0 desliga a malha de d
                  //     (Ud=0, comutação pura) sem recompilar. Console
                  //     'dmax <fr>'.
  P_CUR_DB,       // [A] zona morta do INTEGRADOR das malhas de corrente.
                  //     Tem de cobrir o RUIDO de id/iq medido, senao o
                  //     integrador persegue ruido e vira corrente REAL —
                  //     no eixo d isso aparece como "id sobe sozinho com
                  //     picos". O proprio 'diag' imprime o veredito
                  //     (compara com o dp medido). Console 'db <A>'.
  P_MEAS_AGE,     // [us] ATRASO TOTAL da medida de corrente em relacao ao
                  //     angulo usado para projetar em dq. Nao e so a idade
                  //     da amostra (~95 us pela taxa do ADC): soma tudo que
                  //     atrasa o angulo — estimativa de velocidade do Hall,
                  //     fatia de 1 kHz, filtro. Se estiver CURTO, o
                  //     referencial de medicao fica girado de w.tau e
                  //     aparece um id FANTASMA que CRESCE com a velocidade,
                  //     que a malha de d entao tenta corrigir injetando
                  //     corrente de verdade. Ajuste: varra ate o id medido
                  //     ficar ~0 em TODA a faixa com FW desligado. Se um
                  //     valor so nao serve para todas as velocidades, nao e
                  //     atraso — procure outra causa. Console 'age <us>'.
  P__COUNT        // sentinela
};
#define PARAM_N (P__COUNT - 1)

void  paramsInit();                   // carrega NVS (defaults = config.h)
float prm(uint8_t id);                // leitura rápida (qualquer task)
bool  paramSet(uint8_t id, float v);  // aplica em RAM (com clamp); marca dirty
void  paramsRequestSave();            // pede commit em NVS (feito parado)
void  paramsTick(float kmhAbs);       // chamada periódica: commit adiado
void  paramsFactory();                // volta TUDO ao default do config.h
int   paramsPack(uint8_t *buf, int max);  // [n][id u8, val f32, def f32]*n
void  paramsDump();                   // despeja TODOS os vivos no serial
const char *paramKey(uint8_t id);     // chave curta de um id ("?" se não há)
int   paramIdByKey(const char *k);    // id pela chave (−1 se não existe)
