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
  P_KE = 1,       // [V/(rad/s)] back-EMF do motor (limitador/vel. máx.)
  P_HEADROOM,     // [V] margem acima da back-EMF (torque de arranque)
  P_MODCEIL,      // [-] teto de modulação (máx. 0.56 — bootstrap FD2103S!)
  P_SLEW_UP,      // [V/s] rampa do acelerador (torque)
  P_SLEW_REL,     // [V/s] rampa ao soltar (torque)
  P_ACCEL,        // [rad/s²] aceleração (modo velocidade)
  P_DECEL,        // [rad/s²] freio-motor ao soltar (modo velocidade)
  P_BRAKE_DEC,    // [rad/s²] desaceleração com freio 100%
  P_CAP_MARGIN,   // [V] margem do limite de velocidade por tensão
  P_PID_P,        // [-] PID velocidade P
  P_PID_I,        // [-] PID velocidade I
  P_LPF_TF,       // [s] filtro da velocidade medida
  P_OC_TRIP,      // [A] trip de sobrecorrente
  P_REGEN_I,      // [A] limite de corrente de regeneração
  P_UV_CUT,       // [V] corte de subtensão
  P_T_DERATE,     // [°C] início do derate térmico
  P_T_CUT,        // [°C] corte térmico
  P_WHEEL_MM,     // [mm] diâmetro da roda
  P_FW_START,     // [% duty] joelho do FW adaptativo (satura acima -> avança)
  P_FW_MAX_DEG,   // [° elétricos] avanço máximo (0 = desligado; sinal invertível)
  P_SHUNT_MOHM,   // [mΩ] shunt do INA226 (calibração de corrente)
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
  P_HALL_INTERP,  // 0/1 interpolação do ângulo dos Halls (comutação senoidal)
  P_ILIM_FF,      // 0/1 teto de corrente por feedforward de back-EMF (V<=hd+Ke·ω)
  P_COAST,        // 0/1 costeira (roda-solta ao soltar o acelerador)
  P_BATT_LEARN,   // 0/1 bateria inteligente (aprende capacidade real do pack)
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
