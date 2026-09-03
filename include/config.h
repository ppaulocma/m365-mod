// ============================================================================
// config.h — TODOS os pinos, limiares e calibrações do controlador
// ----------------------------------------------------------------------------
// Tudo que você precisa ajustar está NESTE arquivo. Procure por "AJUSTE AQUI".
//
// Hardware alvo:
//   - ESP32 substituindo o LKS32MC numa controladora M365/Xbot
//   - 3x FD2103S em modo 3-PWM (HIN+LIN unidos => 1 sinal por fase,
//     dead-time e anti-shoot-through são feitos DENTRO do driver)
//   - Halls 5 V passando por conversor de nível -> 3,3 V no ESP32
//   - 1 shunt único no barramento DC (corrente de bateria, NÃO por fase)
// ============================================================================
#pragma once

// ============================================================================
// 1) MODO DE TESTE EM MALHA ABERTA (primeiro teste de bancada!)
// ----------------------------------------------------------------------------
// Coloque 1, grave, e o motor roda em velocity_openloop com tensão baixa,
// SEM usar os Halls. Serve para validar potência + ordem de fases antes de
// fechar a malha. Se o motor "range"/vibra e não gira: troque duas fases
// (inverta dois PIN_PWM_* abaixo). Quando girar liso, volte para 0.
// ============================================================================
#define OPEN_LOOP_TEST            0

#define OPEN_LOOP_VOLTAGE_LIMIT   2.0f   // [V] tensão máx. no teste aberto (baixa!)
#define OPEN_LOOP_MAX_RADS        10.0f  // [rad/s mecânico] alvo com slider a 100%
#define OPEN_LOOP_SLEW            5.0f   // [rad/s²] rampa do alvo em malha aberta

// ============================================================================
// 2) PINOS
// ----------------------------------------------------------------------------
// PWM: 1 sinal por fase (FD2103S com HIN+LIN unidos). Pinos seguros do ESP32
//      (sem função de strapping). Para INVERTER O SENTIDO DE GIRO sem
//      ressoldar: troque dois PIN_PWM_* entre si (ex.: A<->B) E troque os
//      dois PIN_HALL_* correspondentes — ou deixe o initFOC() detectar a
//      direção sozinho (ele compensa por software).
// ADC: TODOS em ADC1 (GPIO 32..39) — ADC2 não funciona com WiFi ligado!
// ============================================================================
#define PIN_PWM_A       19    // fase A -> entrada (HIN+LIN) do FD2103S da fase A
#define PIN_PWM_B       18    // fase B
#define PIN_PWM_C       5    // fase C

// Halls em 33/32/35: pinos FÍSICOS vizinhos no DevKit (35-32-33 em sequência).
// GPIO 35 é entrada-somente e SEM pullup interno — funciona porque o conversor
// de nível empurra o sinal ativamente (por isso o firmware usa pullup EXTERNO
// no SimpleFOC; se algum Hall ficar flutuando em repouso, adicione pullup
// físico de ~10k para 3,3 V).
#define PIN_HALL_A      33    // Hall A (já em 3,3 V via conversor de nível)
#define PIN_HALL_B      35    // Hall B
#define PIN_HALL_C      32    // Hall C (entrada-somente)

// Filtro de glitch dos Halls: rejeita bordas < esta janela após a última aceita
// no pino. CUIDADO: aumentar demais COME BORDAS REAIS quando o ruído é denso (um
// pulso de ruído aceito atualiza o relógio e a borda real seguinte cai na janela)
// — foi por isso que reverti uma tentativa de 500 µs. 100 µs é o equilíbrio
// testado. O ruído de corrente alta se resolve no HARDWARE (ver nota abaixo).
#define HALL_GLITCH_US  100   // [µs] janela mínima entre bordas aceitas por pino

// Confirmação de borda: a ISR espera e RELÊ o pino — nível oscilou e voltou =
// pulso falso vira no-op (a lib ignora estado repetido). Defesa contra ruído
// oscilante. Subido de 3 -> 5 p/ aguentar bursts um pouco mais longos (só ajuda
// ruído OSCILANTE; nível deslocado SUSTENTADO passa igual — isso é hardware).
// Não come bordas reais (só adiciona settle). 0 = DESLIGA.
// PROVADO (1/ago): CONFIRM 0 PIOROU => a confirmação filtra ruído REAL que oscila
// no pino (é ELÉTRICO — ground bounce sob corrente alta — não artefato de SW).
// Como MAIS confirmação = mais filtro, ESCALANDO p/ 8 (era 3). Custo: +settle na
// ISR (só adiciona espera; nunca come borda real). Se melhorar, dá p/ subir mais;
// se voltar aspereza no arranque MUITO lento, baixe. A cura de raiz é o GND do ESP.
#define HALL_CONFIRM_US 8     // [µs] espera de confirmação dentro da ISR

// TRAVA DE DIREÇÃO (rejeição de borda de Hall "de ré") — REMOVIDA (3/ago).
// Duas tentativas derrubaram e QUEIMARAM fase: rejeitar borda de Hall na ISR é
// frágil demais aqui (dessincroniza a comutação com velEst alto, e sem sensor de
// corrente por FASE o teto de tensão não protege). NÃO REABRIR. O tratamento de
// Hall agora é SÓ o básico: filtro de glitch + confirmação + bloqueio de estado
// inválido (000/111). A cura de verdade é hardware (corrente por fase + GND).

// Diagnóstico dos Halls: 1 = forense completa (verificador de sequência,
// contadores por pino, reconciliação, card da dashboard); 0 = MODO MÍNIMO:
// interrupção -> filtro -> espera assentar -> biblioteca, e nada mais. Com o
// hardware saneado (pullup 1k + 1 nF) o mínimo basta; religue o 1 quando
// quiser investigar qualquer sintoma novo de Hall.
#define HALL_DIAG       0


// INTERPOLAÇÃO DO ÂNGULO DE HALL — DE VOLTA, e desta vez com mandato.
// Ela já foi removida daqui com um A/B que deu empate (0,104 vs 0,101 km/h de
// ondulação) — mas aquele teste foi a 4 km/h (BEMF ~1 V, pulsação ridícula),
// com o sensoriamento antigo quebrado, olhando só velocidade (que a inércia
// filtra). Com a corrente MEDIDA de verdade, a caixa-preta mostrou o que o
// vetor saltando 60° faz em velocidade: a BEMF gira contra um vetor parado e
// o eixo d (sem malha) pulsa E·sin(30°)/|Z| — ±20 A REAIS a 15 km/h. É essa
// pulsação que: dispara sobrecorrente "do nada", gera o torque médio
// invisível que fazia a costeira acelerar sozinha, e fura o limitador de
// velocidade (empurra no vão em que o alvo já está em zero).
// A implementação NOVA não tem o defeito da antiga (fração pelo setor
// ANTERIOR que nunca fechava e saltava): o avanço usa a velocidade do
// próprio Hall (período, auto-zerada quando velha) e é TRAVADO na largura do
// setor — nunca invade o próximo; sem salto por construção.
// ATENÇÃO: mudar isto muda o ângulo médio de comutação (~30°) — RECALIBRE o
// 'zea' após gravar (a verificação valida a cadeia completa com a
// interpolação ativa).
#define HALL_INTERP     1

// LEITURAS LENTAS no ADC2 (Vbat + 2 NTCs) — funcionam porque o firmware é BLE
// (sem WiFi, que é o que travaria o ADC2). Isso deixa o ADC1 (34/39) livre
// pra corrente de FASE (2x INA240) com amostragem rápida sincronizada ao PWM.
// (O GPIO 36 ficou LIVRE: o canal de referência foi substituído pelo zero
// vivo amostrado no próprio INA240 — ver phase_current.h.)
// Pinos 25/26/27 são ADC2, no header e não-strapping.
#define PIN_ADC_VBAT      25  // divisor da tensão do pack (ADC2)
#define PIN_ADC_NTC       26  // NTC da CONTROLADORA (junto aos MOSFETs) (ADC2)
#define PIN_ADC_MOTOR_NTC 27  // NTC INTERNO do motor (ADC2)

// ============================================================================
// 3) MOTOR / PWM
// ============================================================================
#define MOTOR_POLE_PAIRS        15       // hub M365 = 15 pares de polo
// PWM ESCOLHIDO PELA JANELA DE AMOSTRAGEM low-side, não por conforto acústico.
// A janela em que os MOSFETs de baixo conduzem é (1 − duty)·T_pwm, CENTRADA
// no evento TEP do timer — e o S&H do ADC só acontece DEPOIS do evento, então
// a conta usa a METADE da janela:
//     duty_max = 1 − 2·(latência_ISR + S&H + margem)/T_pwm
// Com ~5,5 µs de necessidade total:
//     20 kHz (50 µs)   -> duty 0,78 -> teto 0,450
//     16 kHz (62,5 µs) -> duty 0,82 -> teto 0,475   <- escolhido
//     12 kHz (83 µs)   -> duty 0,87 -> teto 0,501  (chiado audível)
// 16 kHz fica na borda da audição. Ajustável em P_PWM_FREQ pelo app; o duty
// máximo REAL é recalculado no boot a partir da latência MEDIDA e aparece no
// serial e no card DUTY MÁX do app.
#define PWM_FREQUENCY_HZ        16000

// AJUSTE AQUI: tensão nominal do pack (10s Li-Ion: ~36 V nominal, 42 V cheio).
// Use a tensão TÍPICA de operação; é usada no cálculo do duty do PWM.
#define VOLTAGE_POWER_SUPPLY    36.0f    // [V]

// Teto de modulação (fração de Vsupply que vira tensão de fase).
// IMPORTANTE — limite de HARDWARE: os FD2103S são drivers BOOTSTRAP: o gate
// do MOSFET de cima é alimentado por um capacitor que SÓ recarrega quando o
// de baixo conduz. Modulação a 100% (0.577 = máximo do seno) deixa o
// high-side ligado tempo demais no pico da senoide -> o capacitor descarrega,
// o gate afunda e a fase falha — é o "se perde/tranca/faz barulho" perto da
// velocidade máxima.
// 0.56 é o TETO DESTE ESTÁGIO, não uma sugestão: dá ~97% de duty máximo, o
// que a 16 kHz deixa ~1,9 µs de condução do lado baixo por ciclo para
// recarregar o bootstrap. Não passe daqui sem trocar o driver ou baixar a
// frequência do PWM — acima disso o capacitor não fecha a conta.
// Com shunt EM SÉRIE COM A FASE (IPH_INLINE) este é o ÚNICO teto que sobra:
// a amostragem deixou de impor limite de duty, então quem define a
// velocidade máxima é este número.
#define MODULATION_CEIL_FACTOR  0.56f

// Limite de tensão de fase aplicado ao motor (controla o "torque máximo").
// Começa BAIXO por segurança; o valor é ajustável ao vivo pela dashboard
// até VOLTAGE_LIMIT_MAX. Suba aos poucos conforme ganhar confiança.
#define VOLTAGE_LIMIT_DEFAULT   42.0f     // [V] uso REAL: autoridade cheia do punho.
                                          // O teto físico é o MODULATION_CEIL x Vbat
                                          // (~22 V), então isto não sobre-limita —
                                          // só evita cortar o torque abaixo do HW.
#define VOLTAGE_LIMIT_MAX       42.0f    // [V] teto absoluto da dashboard

// Tensão usada na calibração do initFOC (alinhamento do sensor).
// AJUSTE AQUI: o hub da M365 tem resistência de fase baixa (~0,2 Ω) =>
// 1,5 V já dá vários ampères. Se a calibração falhar, suba um pouco.
// Faça a 1ª calibração com fonte de bancada limitada em ~5 A e RODA SUSPENSA.
#define VOLTAGE_SENSOR_ALIGN    1.5f     // [V]

// Ke (back-EMF) — DIAGNÓSTICO APENAS desde a malha de corrente. NÃO entra em
// nenhum limite, teto ou alvo: sobrevive só para o comando "medir Ke" do app e
// para conferir a saúde do motor (Ke caindo = ímã desmagnetizando). Se você se
// pegar usando Ke num caminho de controle de novo, é sinal de que a corrente
// medida parou de chegar — conserte isso, não volte ao modelo.
#define MOTOR_KE_V_PER_RADS     0.32f    // [V/(rad/s)] back-EMF medida

// --- MEDIÇÃO DE CORRENTE DE FASE (2x INA240A1, shunts low-side) --------------
// Esta seção SUBSTITUIU o antigo teto por feedforward de back-EMF. Não existe
// mais VOVER_HARD_MAX_V, STALL_V_HEADROOM nem P_ILIM_FF: a corrente de fase é
// MEDIDA e regulada por uma malha de corrente própria (ver currentRegulator em
// motor_control.cpp). O que era estimativa virou realimentação.
//
// LIGAÇÃO (ver HARDWARE.md §4b):
//   shunt low-side (source do MOSFET de baixo) -> INA240A1 IN+/IN-
//   INA240 V+ = 3,3 V | REF = 1,65 V | OUT -> ADC1 do ESP32
//   Saída = 1,65 V + 20 * Vshunt  =>  swing de ±1,65 V cabe no ADC.
//   ATENÇÃO: alimentar o INA240 em 5 V colocaria até ~4,8 V nos GPIO 34/39,
//   que aguentam 3,3 V. Se um dia mudar p/ 5 V, é OBRIGATÓRIO divisor antes
//   do ADC (e ajustar PHASE_CUR_MV_PER_COUNT proporcionalmente).
// ============================================================================
#define PHASE_CUR_ENABLED       1     // A malha de corrente É a arquitetura:
                                      // alvos em ampères, freio = regen em A,
                                      // limitador de velocidade que afunila
                                      // corrente. Não existe mais lei de
                                      // controle por tensão para cair — 0 não
                                      // compila (de propósito: rodar a malha
                                      // sem sensor é o disparo clássico).
                                      // Bancada SEM INA240: OPEN_LOOP_TEST 1.
#if !PHASE_CUR_ENABLED
#error "PHASE_CUR_ENABLED 0 nao e suportado: o controle e todo em corrente medida. Para bancada sem INA240 use OPEN_LOOP_TEST 1 (o modulo nem e inicializado nele)."
#endif

// Pinos de ADC1 (entrada-somente, sem pullup — perfeito p/ ADC). ADC1 porque
// só ele funciona junto com rádio; o BLE não atrapalha, mas o ADC2 sim.
#define PIN_IPH_1               34    // AJUSTE AQUI: canal 1 do INA240
#define PIN_IPH_2               39    // ("VN") canal 2 do INA240

// --- ZERO VIVO (substituiu o canal de referência do GPIO 36) ----------------
// O zero de cada canal é AMOSTRADO no próprio INA240, a 8 kHz, no centro da
// janela high-side (TEZ): com o MOSFET de baixo desligado não passa corrente
// pelo shunt e a saída é exatamente REF — o zero daquele instante, já com a
// deriva do trilho de 3,3 V, a térmica e a oscilação da referência do ADC que
// o rádio BLE induz (é o MESMO ADC lendo). A subtração corrente − zero
// cancela o modo comum por construção; o divisor no GPIO 36 e a chave
// PHASE_REF_* deixaram de existir (o pino ficou livre).
//
// FERRAMENTAS DE DIAGNÓSTICO (estatística de ruído, captura a 4 kHz, teste de
// rádio, degrau da malha). Deixe 1 enquanto estiver mexendo no sensoriamento;
// ponha 0 quando o patinete for para a rua.
#define IPH_DIAG_TOOLS          1

// MAPEAMENTO DAS FASES E SINAL DOS CANAIS: NÃO SE CONFIGURA MAIS AQUI.
// Qual fase está em qual pino, o sinal de cada canal e o evento do MCPWM em que
// os MOSFETs de baixo conduzem são MEDIDOS pela calibração (comando 26 do app,
// roda suspensa) e gravados na NVS. Chutar esses três valores foi exatamente o
// que fez o motor disparar: com o sinal invertido, a malha de corrente vira
// realimentação positiva. Não reintroduza defines para isso.

// CALIBRAÇÃO ADAPTATIVA. A tensão do vetor DC não é fixa: a rotina começa
// baixa e SOBE até a resposta do INA240 ter contagens de ADC suficientes para
// medir com confiança, parando antes no limite de corrente ou de tensão. Isso
// tira do config o palpite sobre o valor do shunt — com shunt pequeno ela sobe
// mais, com shunt grande para antes. Um limiar fixo em volts foi o que
// reprovou uma leitura perfeitamente boa de 13 mV.
#define PHASE_CAL_V_START       1.5f   // [V] primeira tentativa
#define PHASE_CAL_V_MAX         5.0f   // [V] teto do vetor de calibração
#define PHASE_CAL_I_MAX_A       30.0f  // [A] corrente de FASE máxima na cal.
                                       // (a de BARRAMENTO é ~duty x esta, i.e.
                                       // uma fração — por isso a guarda tem de
                                       // ser sobre a de fase, não a do INA226)
#define PHASE_CAL_TGT_LSB       45.0f  // contagens de ADC desejadas na resposta
                                       // da própria fase (~1 % de resolução)
#define PHASE_CAL_MIN_LSB       6.0f   // piso absoluto: abaixo disso é ruído,
                                       // e significa que a amostragem caiu
                                       // fora da condução dos low-side

// Shunt de fase NOMINAL — só reserva. A escala real (A/V) é MEDIDA durante a
// calibração cruzando a resposta do INA240 com a corrente de barramento do
// INA226, então não é preciso saber o valor do shunt. Este número só vale
// enquanto não houver calibração.
#define PHASE_CUR_SHUNT_MOHM    1.0f   // [mΩ] nominal (reserva)
#define PHASE_CUR_AMP_GAIN      20.0f  // INA240A1 = 20 V/V (A2=50, A3=100, A4=200)
#define PHASE_CUR_MV_PER_COUNT  0.806f // [mV/LSB] 3,3 V / 4095
#define PHASE_CUR_VCC_V         3.3f   // alimentação do módulo INA240. Serve
                                       // para classificar o zero medido: perto
                                       // de VCC/2 = REF no meio = bidirecional
                                       // (o único caso que serve); perto de 0
                                       // ou de VCC = REF amarrado = módulo
                                       // unidirecional, que NÃO mede corrente
                                       // de fase (ver phase_current.h).

// O instante de amostragem NÃO é chutado: a latência de disparo é MEDIDA no
// boot (pelo contador do próprio timer) e deriva o duty máximo em que o S&H
// ainda cai dentro da janela low-side. Além disso CADA amostra é validada
// contra o compare real do PWM no momento do disparo — amostra fora da janela
// é descartada, nunca usada. Ver phase_current.h para a conta e o custo.

// Filtro de exibição das correntes de fase (as proteções usam o sinal cru).
#define IPH_DISP_TAU_S          0.30f  // [s]

// PROTEÇÃO POR MAGNITUDE |I| = sqrt(ialpha² + ibeta²). É a grandeza certa para
// proteger: não depende do ângulo elétrico, então o Hall grosso (±30° parado)
// não a contamina. Substitui o antigo teto de tensão sobre a back-EMF.
// 60 A e PICO de fase, nao continuo: com resistencia de ramo medida ~0,12 Ohm,
// 60 A dissipam ~430 W de cobre no motor — muito acima dos 250 W nominais. Quem
// segura o abuso sustentado e o derate termico do motor (MOTOR_TEMP_DERATE_C),
// nao este numero. 90 A era otimista demais para este motor.
#define IPH_TRIP_A              60.0f  // [A] trip duro de pico de fase (suba
                                       // junto com IQ_MAX, sempre acima dele)
#define IPH_TRIP_MS             40     // [ms] acima do trip p/ travar falta
#define IPH_TRIP_HARD_MULT      1.6f   // x IPH_TRIP_A = corte INSTANTÂNEO
                                       // (curto de fase / comutação invertida)

// GUARDA ANTI-DISPARO: tempo que a realimentação de corrente pode ficar
// incoerente (medido com sinal oposto ao pedido, ou tensão no teto sem
// corrente aparecer) antes de CORTAR e travar FAULT_SENSOR. Curto de
// propósito — este é o modo de falha em que o motor dispara e não para, e
// 150 ms já é tempo de sobra para descartar transitório de comutação.
#define IPH_FEEDBACK_TRIP_MS    150

// RASTREIO DO ZERO (anti corrente-fantasma). O offset do INA240 anda com
// temperatura e alimentação; nesta escala uma deriva de poucas dezenas de
// contagens de ADC já vira ampères inventados. Com o motor desabilitado e a
// roda parada — corrente comprovadamente nula — o zero é puxado devagar para a
// leitura, e trava no instante de habilitar.
// SATURAÇÃO DO SENSOR — a verificação que faltava. Um shunt grande demais faz
// a saída do INA240 encostar no trilho, e aí a leitura fica MENOR que a
// corrente real: o PI enxerga folga, integra até o teto e o motor dispara. É o
// mesmo desfecho do sinal invertido, por outro caminho. Por isso a saturação
// não é "leitura ruim", é falta de sensor.
#define IPH_CLIP_LO_CNT         150    // contagens de ADC: abaixo = no trilho
#define IPH_CLIP_HI_CNT         3900   // acima = no trilho
#define IPH_CLIP_TRIP_MS        30     // [ms] saturado com o motor ligado = corta
// Fundo de escala mínimo exigido na calibração, como múltiplo do trip de fase:
// não adianta proteger em 30 A se o sensor cega em 20 A.
#define IPH_FS_MARGIN           1.3f

// NÃO EXISTE MAIS MÉDIA NAS CORRENTES DE FASE. Elas são grandezas AC (até
// ~200 Hz elétricos): a média de 3 ms da versão anterior as atenuava e
// defasava dezenas de graus — o Iq medido "sumia" com o motor andando e a
// malha, vendo menos corrente do que havia, empurrava tensão até disparar.
// Cada canal usa a última amostra VÁLIDA (8 kHz por canal) e o único filtro
// fica em Id/Iq (P_CUR_TF), no referencial rotante, onde o sinal é DC em
// regime e filtrar não distorce nada.

// Tempo entre o disparo da conversão e o fim do S&H do ADC (8 ciclos de
// amostragem do SAR + sobrecusto do disparo). Entra na validação por janela
// e no cálculo do duty máximo.
#define IPH_SAMPLE_US           2.0f
// Margem adicional no cálculo do duty máximo: jitter de latência que a média
// não captura + assentamento do INA240 depois da borda de comutação.
#define PHASE_ISR_MARGIN_US     2.5f
// FOLGA DO TETO DE OPERAÇÃO sobre o teto físico de amostragem. A lição das
// caixas-pretas do topo (faltas 0x01 e depois 0x08): no duty máximo "duro" a
// meia-janela após o TEP é exatamente latência+S&H+margem — a amostra TÍPICA
// vive na fronteira do portão e cada fase fica CEGA durante o arco do seu
// pico de duty (dezenas de graus elétricos POR CICLO). O estrago não é
// amostra suja: é MISTURA — uma fase congelada em ângulo velho combinada com
// a outra fresca vira um vetor dq de lixo (iq medido −17 A com corrente real
// positiva), o PI espeta tensão real em cima disso e nasce o caos do topo.
// A tentativa de guarda extra no portão só ALARGOU os buracos (0x08 = guarda
// de frescor cortando, correta). Conserto estrutural: o teto de OPERAÇÃO
// (vModCeil) fica ESTA folga abaixo do teto físico, de modo que a meia-
// janela no pior duty ainda engula latência média + jitter normal — buraco
// vira exceção de 1 amostra (62 µs), nunca arco cego. Custo: ~8% de tensão
// de topo (≈2 km/h com pack cheio) — o preço de medir de verdade a 100% do
// punho, que é onde o limitador de corrente mais importa.
// ---------------------------------------------------------------------------
// TOPOLOGIA DO SHUNT — 1 = EM SÉRIE COM A FASE (in-line) | 0 = low-side
// ---------------------------------------------------------------------------
// Muda TRÊS coisas, e as três são consequência de um fato só: com o shunt em
// série com a fase a corrente passa por ele o tempo TODO, e não só enquanto o
// MOSFET de baixo conduz.
//
//  1) NÃO EXISTE MAIS TETO DE DUTY POR AMOSTRAGEM. No low-side a amostra tem
//     de caber na janela LOW, que encolhe quando o duty sobe — era isso que
//     limitava a modulação em 0,45 (contra 0,55 do bootstrap) e o que fazia a
//     amostra ser descartada perto do topo. In-line a corrente vale a MÉDIA
//     do período tanto no centro da janela LOW (TEP) quanto no centro da HIGH
//     (TEZ) — a ondulação é um triângulo simétrico e nos dois centros ela
//     cruza a média. Como duty + (1−duty) = 1, uma das duas metades SEMPRE
//     tem pelo menos T/4: a ISR escolhe por canal, por período, a que tiver
//     espaço, e não sobra duty inalcançável.
//
//  2) O ZERO NÃO SAI MAIS DO TEZ. O zero vivo dependia de um instante em que
//     a corrente é ZERO POR CONSTRUÇÃO (low FET aberto). In-line esse
//     instante não existe com o motor andando. O substituto é a média longa:
//     corrente de fase não tem componente contínua, então a média de muitas
//     voltas elétricas converge para o offset (ver IPH_ZERO_SPIN_*).
//
//  3) O MODO COMUM vira o erro dominante. Os dois pinos do INA240 balançam
//     0↔Vbat a cada chaveamento; casamento das entradas e sensoriamento
//     Kelvin passam a decidir a qualidade da medida.
#define IPH_INLINE              1

// Rastreio do zero COM O MOTOR GIRANDO (só existe no in-line). Constante de
// tempo LONGA de propósito: ela precisa cobrir muitas voltas elétricas para
// que a média da corrente seja realmente nula. E só age acima de uma
// frequência elétrica mínima — parado com torque a corrente É contínua, e
// aprender essa média como zero seria aprender a própria corrente.
#define IPH_ZERO_SPIN_TAU_S     3.0f   // [s] constante de tempo da média
#define IPH_ZERO_SPIN_MIN_HZ    8.0f   // [Hz] frequência elétrica mínima

#define IPH_OP_FOLGA_US         3.0f
// AVANÇO DE COMUTAÇÃO default [µs] (P_COMM_LEAD no NVS vence; console
// 'lead <us>'): compensa a latência entre o ângulo medido e a tensão chegar
// nos FETs — meio período do loopFOC, latch do PWM e filtro de velocidade do
// Hall. Sem ele o vetor aplicado inclina ω·τ, ou seja o erro CRESCE com a
// velocidade: liso embaixo, áspero em cima.
//
// MEDIÇÃO (não é chute): o 'varre' imprime, em cada faixa de RODA LIVRE, o
// desalinhamento tirado de Ud/Vq. Com corrente ~zero não há queda resistiva
// nem acoplamento cruzado −ω·Lq·iq, então o Ud que sobra é a componente de
// eixo d da FCEM — desalinhamento puro do referencial de saída. Dividido por
// ω elétrico vira ESTE número. Medir em duas velocidades separa as causas:
// atraso de tempo dá o mesmo τ com ângulos diferentes; erro de ângulo zero
// (P_ZEA) dá o mesmo ângulo com τ diferentes.
//
// 170 µs é o valor medido na bancada (01/set/2026): 167/164/189 µs em três
// velocidades de roda livre entre 10 e 28 km/h. Levou o desalinhamento de
// 11,7° para 0,7° a 27 km/h, o id parasita de 1,17 A para 0,23 A e o Ud sob
// carga de −2,98 V (crescendo com a velocidade) para −0,45 V (plano).
// ATENÇÃO: a varredura 'ang' NÃO enxerga isto — ela roda a ~6 km/h, onde
// 170 µs são 2,2°, abaixo do seu passo de 6°.
#define COMM_LEAD_US_DEFAULT  170.0f
// Idade média da amostra de corrente na projeção dq [µs]. A projeção usa o
// ângulo DO INSTANTE DA AMOSTRA (ângulo atual − ω·idade − ω·lead) — sem isso
// o id real vaza para o iq medido (era o −4 A dos registros do topo).
// Conta, com o intervalo I entre amostras DO MESMO canal:
//     idade = I/2 (a fatia lê num instante qualquer) + I (centro da mediana-3)
// IN-LINE são DOIS centros por período (TEP e TEZ) alternando os canais, ou
// seja 16 kHz por canal -> I = 62,5 µs -> idade ≈ 94 µs. No low-side eram
// 8 kHz por canal (um centro só, e metade dos eventos gastos no zero).
// 460 e não os 95 da conta acima: medido contra corrente de bateria por
// ampère de fase, o 460 repetiu 0,636 em duas corridas na faixa de 22 km/h
// contra 0,668 do 95, e empatou nas faixas de baixo. A conta dá o piso do
// atraso (ADC + mediana), não o total — sobra a estimativa de velocidade do
// Hall e a fatia de 1 kHz, que não têm fórmula fechada aqui.
#if IPH_INLINE
#define IPH_MEAS_AGE_US         460.0f
#else
#define IPH_MEAS_AGE_US         125.0f
#endif
// ---- PRÉ-DISPARO da amostra de corrente (o rework "definitivo") ------------
// Geometria antiga: a conversão disparava NO evento TEP (centro da janela
// low-side) e TODO o orçamento (latência da ISR + setup + S&H + margem,
// ~7,2 µs) era gasto DEPOIS do centro — a metade da janela ANTES do TEP era
// desperdiçada. Rework: um comparador B ocioso do MCPWM (a SimpleFOC só usa
// os A; o gerador B não tem pino nem ação) interrompe (latência média +
// guarda) ANTES do TEP, e o S&H cai CENTRADO na janela. O requisito de
// meia-janela cai p/ max(setup+S&H+margem−ttp, ttp+margem−setup) ≈ 3,8 µs
// ⇒ duty de operação 0,674→~0,78 (+16% de tensão de topo com validação por
// amostra intacta — acima até do 0,738 da era cega). A SUBIDA do compare B
// (pré-TEP) dispara; a DESCIDA (pós-TEP) é ignorada pelo bit de direção do
// timer (uma leitura de timer_status entrega contagem+direção juntas).
// 0 = geometria antiga (fallback também é automático se a calibração
// detectar corrente no TEZ — mapa invertido).
#define IPH_PRETRIG_ENABLE      1
// Guarda somada à latência média medida no boot para posicionar o disparo:
// PRETRIG = lat_avg + guarda ⇒ S&H nominal ~[−0,7..+1,3 µs] do TEP.
#define IPH_PRETRIG_GUARD_US    2.5f
// Entre o start do SAR e o início efetivo do S&H (setup do controlador).
#define IPH_SAR_SETUP_US        0.3f
// CERCO DE SANIDADE do zero vivo [V], em torno do zero da calibração. Não é
// envelope de deriva (o zero vivo já acompanha qualquer deriva legítima por
// medição direta): é um cerco LARGO que só um canal morto, em curto ou
// desligado atravessa. Fora dele o valor prende no limite e o firmware avisa
// (phaseCurrentZeroPinned) em vez de "corrigir" com um número impossível.
#define IPH_ZERO_MAX_DRIFT_V    0.250f
// GUARDAS DO ZERO VIVO — aprendidas COM CARGA na bancada. O portão de janela
// do TEZ era marginal na faixa de duty 15-25% (exatamente onde carga a baixa
// velocidade opera) e o S&H caía no deadtime do FD2103S com o low-side ainda
// conduzindo: corrente REAL de carga entrava no acumulador de zero a 8 kHz,
// o zero envenenava até travar no cerco (±0,25 V x 77 A/V = os "±19 A
// fantasmas congelados" vistos na bancada) e a malha disparava.
// Camada 1: o portão do zero passou a usar a MESMA margem do teto de duty
//           (IPH_SAMPLE_US + PHASE_ISR_MARGIN_US), cobrindo deadtime e
//           propagação do driver — zero só amostra com janela FOLGADA.
// Camada 2: salto máximo por amostra. Offset é grandeza LENTA (mV/s): uma
//           amostra de zero a mais de este tanto do acumulador não é deriva,
//           é contaminação ou spike — descarta e conta.
#define IPH_ZERO_STEP_MAX_CNT   16    // [cnt] ~1 A: acima disso, descarta
// Camada 3: excursão máxima do zero VIVO em torno do zero da calibração. O
//           trilho sob carga anda ~20-30 mV; 40 mV cobrem com folga. (O cerco
//           largo de 0,25 V segue existindo só como detector de canal morto.)
#define IPH_ZERO_LIVE_BAND_CNT  50    // [cnt] ~40 mV / ~3 A de teto de dano
// (Camada 4 fica no motor_control: realimentação CONGELADA >30 ms com o motor
//  habilitado = FAULT_SENSOR — malha cega nunca continua empurrando.)

// Constante de tempo do rastreio de zero em REPOUSO [s] (motor desabilitado +
// roda parada, único estado sem janela high-side — duty 0 — em que o zero
// vivo não atualiza sozinho). Lento de propósito: em repouso não há pressa.
#define IPH_ZERO_REST_TAU_S     0.10f
// RAIO DE CONFIANÇA do rastreio de repouso [V], em torno do zero da
// CALIBRAÇÃO. A bancada provou por que ele tem de ser curto: logo depois de
// uma calibração/teste o rotor BALANÇA dentro de um setor de Hall — nenhuma
// borda é gerada, o portão de "parado" abre, e a corrente de frenagem do
// balanço (±40+ contagens) era aprendida como zero e congelava: ~5 A
// fantasmas com o motor desligado. Deriva LEGÍTIMA de repouso é de poucos
// mV; corrente de balanço é dezenas. Dentro deste raio o rastreio ajusta
// fino; fora dele, não aprende — e o zero vivo (TEZ) segue livre dentro do
// cerco largo IPH_ZERO_MAX_DRIFT_V, porque sob carga o trilho anda mesmo.
#define IPH_ZERO_REST_BAND_V    0.012f

// Corrente de fase máxima que o punho cheio pede (alvo de Iq). É o novo
// "torque de arranque": em vez de uma margem de tensão que VIRAVA corrente por
// uma divisão pela resistência de fase estimada, aqui o número JÁ é a corrente.
// DEFAULT CONSERVADOR de propósito. Este é o número que o punho cheio pede, e
// é o primeiro que se sobe depois de conferir no Monitor que Iq medido segue
// Iq pedido. Comece baixo, ande, confira, suba. 10 A já movem o patinete.
#define IQ_MAX_DEFAULT_A        10.0f  // [A] default (ajustável no app)
// [A] teto ABSOLUTO que o app permite pedir de corrente de fase. Com 6x
// IRFP4110 (100 V, 3,7 mOhm, 180 A de pacote) o estagio de potencia nao e
// mais o limite — quem limita e o MOTOR (250 W nominal) e o pack. Este teto
// existe para o app nao oferecer um numero que o resto do sistema nao
// sustenta; o orcamento real sai de P_IPH_TRIP * I_BUDGET_FRAC.
#define IQ_MAX_ABS_A            70.0f
// Fração do TRIP de fase que serve de orçamento para TODA a corrente pedida —
// o iq do punho e o id do field weakening somados vetorialmente. Não é 1.0
// porque |I| medido tem ondulação: viver no trip é tripar.
#define I_BUDGET_FRAC           0.85f

// PI da malha de corrente (saída em volts, entra em Uq/Ud).
//   P = L * ωc      I = R * ωc      com ωc = 2π * banda desejada
// R = 0,13 Ω MEDIDO na calibração (1,5 V / 7,7 A = 0,195 Ω = 1,5·R, porque o
// vetor DC põe uma fase contra as outras duas em paralelo). L ~ 0,3 mH (típico
// do hub M365 — se medir o seu, refaça a conta).
//
// A BANDA É LIMITADA PELO ATRASO DA MEDIÇÃO. Com a reescrita ele DESPENCOU:
//   amostra mais recente (8 kHz/canal) ... até 125 µs (62 µs médio)
//   filtro de Iq/Id (P_CUR_TF = 2 ms) .... 2000 µs
//   fatia do regulador (1 kHz) ........... 500 µs médio
//   TOTAL ................................ ~2,6 ms, dominado pelo FILTRO
// (antes o anel de 3 ms nas fases AC ainda ATENUAVA e defasava o próprio
// sinal — não era só atraso, era medição errada em movimento.)
// Os ganhos default continuam os validados na bancada (53 Hz): são seguros.
// Para subir a banda agora o caminho é BAIXAR P_CUR_TF (ex.: 1 ms) e recalcular
// P e I pela fórmula acima — o degrau ('step 3') confirma no ato.
// AJUSTADOS NA BANCADA (degrau de 3 A, roda presa), não calculados:
//   P=0,100 I=20 -> assenta em 30 ms, ondulação 0,26 A (abaixo do ruído do
//                   sensor, que é 0,50 A), sobressinal 85 %
//   P=0,100 I=10 -> sobressinal cai para 60 %, mas assenta em 208 ms e a
//                   ondulação sobe para 1,22 A: integrador lento demais rejeita
//                   pior as perturbações. Troca ruim.
//   P=0,050 I=20 -> OSCILA. Baixar o P derruba o cruzamento (P/L) e deixa o
//                   zero do compensador (I/P) ACIMA dele — tira margem de fase
//                   em vez de dar. Quem amortece é baixar o I.
// O sobressinal de 85 % é artefato do degrau instantâneo do teste: o acelerador
// é limitado a TORQUE_SLEW_A_S (120 A/s), então 3 A levam 25 ms para serem
// comandados e a malha assenta em 30 ms — em operação ela nunca vê um degrau.
#define CUR_PID_P               0.10f    // banda ~53 Hz
#define CUR_PID_I               20.0f
#define CUR_LPF_TF              0.002f   // [s] filtro de Iq/Id (2 ms)

// ZONA MORTA DO INTEGRADOR — em ampères, no tamanho do ruído da medição.
// O ruído da corrente tem média zero, então NÃO move o motor sozinho. Mas o
// integrador o ACUMULA: com I=40 e ~0,6 A de ruído, ele faz um passeio
// aleatório de ~0,8 V em 1 s, o que num motor parado (0,195 Ω) são ~4 A de
// corrente REAL. É isso que faz o motor tremer com o punho em zero.
// A correção é não integrar erro menor que a incerteza da medição: abaixo dela
// o "erro" não é distinguível de ruído, e integrá-lo é integrar ruído. Só a
// parte do erro que EXCEDE a zona morta alimenta o integrador (subtração
// contínua, sem degrau). O termo proporcional segue agindo sempre.
// AJUSTE: use o valor que o diagnóstico reportar como ruído em |I|. Se trocar
// para sensor com mais ganho, baixe junto — zona morta grande demais vira erro
// de regime na corrente.
// 0,5 A dimensionado pelo diag com a escala ANCORADA (INA226 aferido a 3 A
// contra amperimetro): |I| cru dp 0,40 A com o radio ATIVO — que é a
// condição real de uso (app conectado). Com os shunts de 1 mΩ (2,7x menos
// ruído nesta escala: 0,107 -> 0,040 A/cnt) desça para ~0,2.
#define CUR_DEADBAND_A          0.5f

// --- MALHA DO EIXO D (id -> 0 via Ud) ---------------------------------------
// O FOC completo que faltava. Com Ud=0 fixo, qualquer erro de ângulo põe a
// BEMF projetada no eixo d empurrando corrente que ninguém regula:
// id ≈ E·sin(erro)/|Z| — na bancada, 8 A circulando a 10 km/h queimando
// ~12 W sem produzir torque (o balanço de potência fecha: 0,5 A de bateria
// = quase todo cobre do id). A nota histórica "não usar foc_current porque o
// Id medido é artefato do Hall" valia quando a MEDIÇÃO era lixo; o balanço
// de potência provou (duas vezes) que o id medido hoje é corrente real — e
// regular corrente real para zero é a função de uma malha de d. Ela usa os
// MESMOS ganhos da malha de q (mesma planta R-L) e a mesma zona morta.
// A malha mata a parte DC/lenta do id (o que o app mostra sustentado); o
// ripple de setor acima da banda (~centenas de Hz) permanece — é inerente.
// Fração do teto de tensão que o eixo d pode usar: o d precisa de pouco
// (E·sin(erro) ~ 1-2 V); 0,35 reserva no pior caso ainda 94% do teto ao q.
#define UD_MAX_FRAC             0.35f

// ZERO TORQUE DE VERDADE (parado). Com alvo de corrente zero, a malha compara
// zero com a corrente MEDIDA — e a medida carrega o fantasma residual do
// offset. Para "zerar" essa leitura o PI injeta corrente REAL no motor; como o
// vetor de erro é fixo no estator e o ângulo do rotor muda, a correção gira e
// o motor treme e dá trancos sem ninguém acelerar.
// Parado, torque zero se realiza aplicando TENSÃO zero: as fases ficam curtas
// ao GND e não há back-EMF para empurrar corrente. Isso não é aproximação, é a
// realização exata de torque nulo com o rotor parado.
// Acima de IQ0_STOP_KMH a back-EMF já importa (tensão zero passaria a FREAR),
// e aí a malha tem de assumir — é ela que faz a costeira.
// A bancada provou o porquê: sem este desvio, com alvo 0 o regulador fica
// perseguindo o ruído da medição (±0,4 A) com o ângulo de Hall saltando 60°
// a cada arrastadinha do rotor — o motor "não quer parar, fica para lá e
// para cá". Abaixo deste limiar (medido pelas bordas de Hall, o sinal
// rápido) e com alvo ~0, o firmware aplica 0 V e zera o integrador; o freio
// residual de 0 V nessa faixa é desprezível (back-EMF < 1 V).
#define IQ0_STOP_KMH            1.5f   // [km/h] só no modo torque com alvo ~0

// LIMITADOR DE VELOCIDADE — regulador, não multiplicador.
// A versão anterior multiplicava o alvo de corrente por um fator que ia a zero
// no teto. Multiplicador não tem ação integral: ele nunca assenta na corrente
// que SEGURA a velocidade, então passa do teto, corta tudo, a roda desacelera
// sem carga, e ele solta tudo de novo — ciclo limite. Com roda solta, onde a
// aceleração é 55x a de rodagem, isso vira "dispara e sai louco" mesmo com
// limite baixo. Antecipar pela derivada (a tentativa anterior) ameniza mas não
// cura, porque o problema é a falta de integrador, não de amortecimento.
// O que as controladoras fazem: um PI de velocidade cuja SAÍDA é um TETO de
// corrente. Com integrador ele converge para a corrente exata que mantém a
// velocidade no limite, e o alvo do punho é simplesmente limitado por ele.
// Ganhos em ampères por km/h de erro. Com roda solta 1 A dá ~5,5 km/h/s, então
// P=3 põe o cruzamento em ~2,6 Hz — vinte vezes mais lento que a malha de
// corrente (53 Hz), que é a separação que uma cascata pede.
#define SPD_LIM_P               3.0f   // [A por km/h de erro]
#define SPD_LIM_I               6.0f   // [A por km/h por segundo]
// Velocidade dos testes de bancada 'gira'/'ab' [km/h]. Baixa de propósito: o
// que se mede é ondulação, e ela aparece igual a 4 km/h — não há motivo para
// pôr a roda a 25 km/h em cima da bancada.
#define SPIN_TEST_KMH           4.0f
// PARADO DE VERDADE: tempo sem NENHUMA borda de Hall que autoriza o rastreio do
// zero da corrente de fase [ms]. Não dá para usar a velocidade estimada: com o
// motor desabilitado as três fases ficam em curto (low-side ligados, o "freio
// suave"), e a roda descendo por esse curto gera corrente REAL de frenagem —
// alguns ampères ainda a 0,5 km/h. Rastrear o zero ali é ensinar ao firmware
// que essa corrente é zero. As bordas de Hall são medida direta de movimento:
// 300 ms de silêncio = abaixo de ~0,05 km/h, aí a corrente é mesmo nula.
#define IPH_ZERO_STILL_MS        300
#define STALL_TRIP_MS           500
#define STALL_SPEED_KMH         1.0f
#define STALL_IQ_MIN_A          12.0f  // [A] |I| de fase que caracteriza tração

// Aborto por sobrecorrente das rotinas de calibração que GIRAM o motor
// (ângulo, Ke, varredura). É a corrente de FASE MEDIDA (|I| dos INA240) —
// a de bateria, com o rotor parado, nem enxerga a corrente que circula.
#define AUTOCAL_I_ABORT_A       15.0f    // [A] |I| de fase que aborta a cal
// Rampa de tensão (soft-start): evita tranco/inrush e dá tempo da malha de
// corrente assumir. Quem limita de fato é o aborto por corrente medida acima.
#define AUTOCAL_RAMP_V_S        6.0f     // [V/s] rampa de subida da tensão na cal
// COSTEIRA (roda-solta ao soltar o acelerador) — liga/desliga por P_COAST.
// Com malha de corrente isto deixou de precisar de regulador e de modelo de
// back-EMF: costeira é literalmente Iq alvo = 0. A malha segura a corrente em
// zero sozinha, então a roda fica solta de verdade em qualquer velocidade.
#define COAST_TARGET_A          0.0f    // [A] Iq alvo ao soltar o acelerador
// SOLTAR O PUNHO NÃO É ALVO ZERO — descoberta da caixa-preta (28/ago):
// "segurar iq=0 em velocidade" é INSTÁVEL neste acionamento. A pulsação de
// setor do eixo d (Hall em degraus, Ud=0) tem assimetria (o atraso L/R
// distorce a onda dentro do setor) que gera ~1-2 A de torque médio REAL
// invisível ao iq medido — o "coast" se auto-acelerava ~10 km/h/s com o punho
// solto, o limitador não podia agir (só reduz alvo POSITIVO) e a pulsação a
// 17+ km/h disparava a sobrecorrente. O firmware antigo nunca teve isso
// porque soltar o punho DERRUBAVA a tensão (freio motor) — comportamento
// estrutural, não estético. Com P_COAST desligado (default), soltar aplica
// este freio-motor leve em corrente — estável por construção: qualquer
// subleitura só o torna um pouco mais fraco, nunca acelerador.
#define RELEASE_BRAKE_A         1.2f    // [A] freio-motor ao soltar (coast OFF)
#define RELEASE_BRAKE_MIN_KMH   3.0f    // [km/h] abaixo disso solta de vez
// GUARDA DA COSTEIRA (P_COAST ligado): costeira de verdade NÃO ACELERA — se
// a velocidade sobe com alvo ~0, a malha está injetando energia que o iq
// medido não enxerga; a derivada da velocidade é o sensor de torque que falta
// (1 A ~ 5,5 km/h/s nesta roda). Acima do limiar, o integrador sangra até a
// subida parar. Rastrear BEMF na DESCIDA continua livre (costeira legítima).
#define COAST_GUARD_ACC_KMH_S   0.8f    // [km/h/s] subida que ENGATA a guarda
#define COAST_GUARD_ACC_OFF     0.25f   // [km/h/s] abaixo disso ela SOLTA
// A guarda é um DECAIMENTO PROPORCIONAL do integrador (τ abaixo), não uma
// sangria fixa: a bancada provou que sangria fixa (8 V/s) EMPATA com o
// rewind do próprio integrador (I=20 sobre ~0,7 A de viés ≈ +6 V/s) e o
// runaway continua. Decaimento proporcional vence qualquer ganho que o app
// permita: a 15 V ele tira 60 V/s, e a autoridade cresce com o erro.
#define COAST_GUARD_TAU_S       0.25f   // [s] tau do decaimento do integrador

// --- Resultado da calibração do initFOC -------------------------------------
// Na primeira execução, o initFOC() gira o motor e imprime no serial o
// "zero electric angle" e a direção do sensor. Anote e cole aqui para os
// próximos boots NÃO precisarem girar o motor para calibrar.
#define MOTOR_ZERO_ELEC_ANGLE   2.0944f      // AJUSTE AQUI: ex. 2.09f (NAN = auto)

// VERIFICAÇÃO DO ÂNGULO ZERO. O initFOC devolve um número mesmo quando o rotor
// não pôde girar (roda presa, fase aberta): ele lê onde o rotor ESTÁ, não onde
// deveria ter alinhado. Por isso a calibração só grava o valor depois de provar
// que ele produz torque no sentido certo — Iq positivo tem de acelerar para
// frente. Sem esta prova, um ângulo 120° errado passa despercebido e faz o
// torque virar negativo, com o motor lutando contra si mesmo.
#define ZEA_VERIFY_A            3.0f    // [A] Iq do teste de torque
#define ZEA_VERIFY_RADS         3.0f    // [rad/s] mínimo p/ considerar que girou
// Direção do sensor: SÓ o default de fábrica do parâmetro P_SENS_DIR. A
// calibração 'zea' MEDE a direção (varredura de uma volta elétrica em cada
// sentido) e persiste — não se chuta mais. Uma direção forçada errada faz o
// ângulo elétrico contra-girar em movimento: parado alinha, girando o eixo d
// dispara sem controle (E·sinΔ/R — foi o aborto de >15 A visto na bancada).
#define MOTOR_SENSOR_DIR        1       // default de P_SENS_DIR: 1 = CW, -1 = CCW

// ============================================================================
// 4) RODA / VELOCIDADE
// ============================================================================
#define WHEEL_DIAMETER_M        0.216f   // [m] roda 8,5" da M365. AJUSTE se outra.
#define MAX_KMH_ABS             50.0f    // [km/h] teto absoluto da dashboard
#define MAX_KMH_DEFAULT         30.0f    // [km/h] limite de velocidade (rua)

// ============================================================================
// 5) CORRENTE — INA226 no shunt único do barramento (I2C)
// ----------------------------------------------------------------------------
// CHAVE GERAL: 0 = leitura de corrente DESATIVADA (foldback + sobrecorrente
// off; dashboard mostra 0,0 A). Subtensão e temperatura seguem ativas. Sem
// isso NÃO há limite de corrente por firmware — use fonte limitada na bancada!
// LIGAÇÃO DO INA226: VCC->3V3, GND->GND, SDA->21, SCL->22, A0/A1->GND (0x40),
// IN+/IN- por cima do shunt ORIGINAL da placa (lado baixo, máx. 36 V de modo
// comum), VBUS->GND. NÃO use o shunt R100 do módulo (some a 20 A).
// ============================================================================
#define CUR_SENSE_ENABLED       1

// Filtros de EXIBIÇÃO (telemetria/app/painel) — as PROTEÇÕES continuam no
// sinal rápido (foldback/OC em ms); só o que você VÊ fica calmo.
#define IBAT_DISP_TAU_S         0.8f   // [s] corrente exibida (medidor de EV)
#define TEMP_DISP_TAU_S         3.0f   // [s] temperatura exibida
#define I2C_SCAN                0      // 1 = só escaneia o barramento no boot e loga
                                       // os endereços no serial (p/ achar o INA226);
                                       // ponha o valor achado no INA226_ADDR e volte a 0
#define INA226_ADDR             0x44   // A0 e A1 no GND
// [mΩ] resistência REAL do shunt do INA226. Não é só o medidor de bateria:
// é a REFERÊNCIA contra a qual a calibração da corrente de FASE deriva a
// escala de cada canal (I_fase = I_bus/duty). Errado aqui, toda a escala de
// corrente do sistema sai errada junto — trip, orçamento do FW e torque por
// ampère. 3.6 mΩ é o valor conferido pela fonte de bancada.
#define INA226_SHUNT_MOHM       3.600f
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22
#define CUR_INVERT              0      // 1 se a corrente positiva aparecer negativa

// Limite de corrente de bateria (foldback suave — reduz potência sem desligar).
// É o ÚNICO teto de tração: default médio (14 A), teto que o app permite pedir
// (20 A). Sem teto de potência oculto — quem manda é a "Corrente máxima" do app.
#define I_LIMIT_DEFAULT_A       14.0f    // [A] limite de corrente de bateria (médio)
#define I_LIMIT_MAX_A           40.0f    // [A] teto que a dashboard permite pedir

#define REGEN_I_LIMIT_A         6.0f     // [A] corrente máx. de REGENERAÇÃO (freio)
                                         //     conservador (regen num pack cheio
                                         //     eleva a tensão do barramento)

// Proteção de SOBRECORRENTE dura (desliga e trava até "limpar faltas")
#define OC_TRIP_A               60.0f    // [A] trip duro de bateria: acima do limite
                                         // (20) com margem. Backstop de BATERIA
                                         // (não vê corrente de fase — ver VOVER)
#define OC_TRIP_MS              100      // [ms] tempo acima do limiar p/ disparar

// Dinâmica do foldback (normalmente não precisa mexer)
#define FOLDBACK_GAIN           2.0f     // [1/(A*s)] velocidade de corte
#define FOLDBACK_RECOVERY       0.5f     // [1/s] velocidade de recuperação

// ============================================================================
// 6) TENSÃO DA BATERIA (divisor resistivo)
// ----------------------------------------------------------------------------
// CHAVE GERAL: 0 = divisor AINDA NÃO LIGADO no pino. Desliga a leitura e a
// proteção de subtensão (um pino de ADC flutuando lê lixo e dispara falta
// à toa!). A dashboard mostra 0,0 V. Volte para 1 quando ligar o divisor.
// ============================================================================
#define VBAT_SENSE_ENABLED      1

// AJUSTE AQUI (quando reativar): valores REAIS do seu divisor. Calibre
// medindo o pack com multímetro e ajustando VBAT_CAL_GAIN até bater.
#define VBAT_R_TOP              100000.0f // [Ω] resistor de cima (pack -> pino)
#define VBAT_R_BOTTOM           4700.0f   // [Ω] resistor de baixo (pino -> GND)

// Filtro da leitura (amostrada a cada 25 ms): peso da amostra nova no EMA.
// 0.015 => constante de tempo ~1,7 s — display firme, sem flutuação.
// A proteção de subtensão não sofre: ela já exige afundamento SUSTENTADO
// (UV_TRIP_MS) + histerese, então filtro lento = menos falso disparo.
#define VBAT_LPF_ALPHA          0.015f
#define VBAT_CAL_GAIN           1.000f    // fator fino de calibração

#define UV_CUTOFF_V             32.0f    // [V] subtensão: corta (10s: 3,2 V/célula)
#define UV_RECOVER_V            34.0f    // [V] histerese p/ permitir limpar a falta
#define UV_TRIP_MS              800      // [ms] filtro (ignora afundamento breve)
#define UV_BOOT_GRACE_MS        3000     // [ms] carência pós-boot: não avalia
                                         // subtensão enquanto filtro/sensores
                                         // ainda estão assentando

// ============================================================================
// 7) TEMPERATURA (NTC junto aos MOSFETs)
// ----------------------------------------------------------------------------
// CHAVE GERAL: 0 = NTC AINDA NÃO LIGADO no pino. Desliga a leitura, o corte
// térmico e o derate (pino flutuando lê lixo e pode "medir" 150 °C!).
// A dashboard mostra 0 °C. Volte para 1 quando ligar o NTC.
// ============================================================================
#define NTC_SENSE_ENABLED       1

// AJUSTE AQUI (quando reativar): parâmetros do SEU NTC e topologia.
// Assumido (padrão seguro): NTC entre o pino e GND, resistor série p/ 3,3 V.
// Se for o contrário (NTC no lado de cima), mude NTC_TO_GND para 0.
#define NTC_BETA                3950.0f  // AJUSTE AQUI: beta do seu NTC
#define NTC_R25                 10000.0f // [Ω] resistência a 25 °C
#define NTC_SERIES_R            10000.0f // [Ω] resistor série do divisor
#define NTC_TO_GND              1        // 1 = NTC p/ GND; 0 = NTC p/ 3,3 V

#define TEMP_DERATE_C           70.0f    // [°C] começa a reduzir potência
#define TEMP_CUTOFF_C           90.0f    // [°C] corta e trava (falta)
#define OT_TRIP_MS              1000     // [ms] filtro do corte térmico

// --- NTC INTERNO do MOTOR (GPIO27) — MODELO PRÓPRIO (diferente do dos MOSFETs) -
// O NTC do motor é OUTRO componente (beta/R25 diferentes), por isso lê errado
// se usar as constantes do NTC dos MOSFETs. AJUSTE AQUI até ler a ambiente com o
// motor FRIO: mexa no R25 pra deslocar o valor; no BETA pra ajustar a inclinação
// (lendo BAIXO demais como 12 °C a 18 °C = R25/beta fora). Derate e corte pela
// temp do MOTOR entram junto com os dos MOSFETs (a proteção usa o pior dos dois).
#define MOTOR_NTC_SENSE_ENABLED 1
#define MOTOR_NTC_BETA          3950.0f  // AJUSTE AQUI: beta do NTC do motor
#define MOTOR_NTC_R25           13000.0f // [Ω] resistência a 25 °C do NTC do motor
#define MOTOR_NTC_SERIES_R      10000.0f // [Ω] resistor série do divisor do motor
#define MOTOR_NTC_TO_GND        1        // 1 = NTC p/ GND; 0 = NTC p/ 3,3 V
#define MOTOR_TEMP_DERATE_C     100.0f   // [°C] motor: começa a reduzir potência
#define MOTOR_TEMP_CUTOFF_C     120.0f   // [°C] motor: corta e trava (falta)

// ============================================================================
// 8) COMPORTAMENTO DE CONTROLE (rampas, freio, PID de velocidade)
// ============================================================================
// Rampas do acelerador — agora em AMPÈRES por segundo (o alvo virou corrente).
// Equivalência aproximada com os valores antigos em V/s: com R_fase ~0,25 Ω,
// 90 V/s de tensão davam ~360 A/s de corrente parado. Os defaults abaixo são
// mais macios de propósito: com malha de corrente o torque chega de verdade.
#define TORQUE_SLEW_A_S         120.0f   // [A/s] rampa do acelerador: RESPOSTA
#define TORQUE_RELEASE_A_S      300.0f   // [A/s] rampa ao SOLTAR/reduzir: rápida

                                         // (soltar o acelerador tem que cortar
                                         // já — 48 V/s dava ~0,4 s de "delay")
#define ACCEL_RADS2             25.0f    // [rad/s²] aceleração (modo velocidade)
#define DECEL_RADS2             8.0f     // [rad/s²] "freio motor" ao soltar acelerador
#define BRAKE_DECEL_RADS2       40.0f    // [rad/s²] desaceleração com freio a 100%
#define BRAKE_MIN_SPEED_RADS    2.0f     // [rad/s] abaixo disso o freio vira alvo 0
                                         //   (evita o motor girar PARA TRÁS no fim
                                         //    da frenagem em modo torque)
// (O limite de velocidade por teto de tensão sobre a back-EMF foi REMOVIDO.
//  Virou afunilamento do Iq permitido perto da vel. máx. — SPD_CAP_BAND_KMH,
//  na seção de corrente de fase. Realimenta velocidade de verdade e não
//  oscila porque atua sobre corrente, que responde em milissegundos.)

// PID de velocidade (modo velocity). A SAÍDA É CORRENTE (Iq em ampères) — a
// malha de corrente fica por dentro (cascata).
// SINTONIA PELA BANCADA (roda livre = o caso de MAIOR ganho da planta:
// 1 A ≈ 14 rad/s² — com piloto a inércia é 20-50x maior e tudo fica mais
// manso). Cruzamento ≈ P·14 rad/s; o ZERO do PI (I/P) tem de ficar pelo
// menos 3x ABAIXO do cruzamento, senão a rampa "surge" a ~2 Hz — eram os
// bumps medidos com P=0,8/I=6 (zero em 7,5 contra cruzamento em 11: em
// cima). Com P=0,5: cruzamento 7 rad/s; I=2,5 põe o zero em 5... ainda
// justo? Não: 2,5/0,5 = 5 contra 7 — use I menor se a rampa ainda pulsar
// ('vi' no console). O filtro do velEst caiu de 80->40 ms junto (o Hall
// interpolado dispensa filtro pesado), devolvendo ~20° de margem.
#define VEL_PID_P               0.5f     // [A/(rad/s)]
#define VEL_PID_I               1.5f     // [A/(rad/s·s)] zero em 3 rad/s (<7/2)
#define VEL_PID_D               0.0f
#define VEL_PID_RAMP            400.0f   // [A/s] rampa interna da saída do PID
#define VEL_LPF_TF              0.10f    // [s] filtro da velocidade medida — Halls
                                         //     são "grossos": suba se der tranco,
                                         //     desça se a resposta ficar mole

// ============================================================================
// 8c) MODOS DE PILOTAGEM / CRUISE / WALK / ENERGIA
// ----------------------------------------------------------------------------
// Cada modo limita corrente, velocidade e suaviza a rampa. O TURBO usa os
// limites configurados no app sem reduções (e é o único que permite field
// weakening). Valores AJUSTÁVEIS aqui; seleção em tempo real pelo app.
// ============================================================================
#define RIDE_ECO_ILIM_A      10.0f
#define RIDE_ECO_VMAX_KMH    22.0f   // solto p/ não segurar abaixo do teto físico (~22)
#define RIDE_ECO_SLEW_MUL    0.55f
#define RIDE_NORMAL_ILIM_A   20.0f
#define RIDE_NORMAL_VMAX_KMH 30.0f   // NORMAL = velocidade cheia (hardware limita ~22)
#define RIDE_NORMAL_SLEW_MUL 0.85f
// TURBO: sem tetos próprios (usa os limites do app) e FW liberado

#define CRUISE_MIN_KMH       5.0f    // só engata acima disso
#define WALK_KMH             4.0f    // alvo do walk mode
#define WALK_ILIM_A          5.0f    // corrente máx. no walk
#define LOCK_ILIM_A          8.0f    // corrente máx. da trava eletrônica

#define UV_WARN_V            33.5f   // [V] aviso de bateria fraca (antes do corte)

// --- Luzes -------------------------------------------------------------------
// LUZ DE FREIO no ESP (GPIO 23 -> gate de MOSFET N -> LED- ; LED+ no 5V/12V).
// Acende no manete OU em regen forte; com o farol ligado vira lanterna fraca
// (PWM). O FAROL fica no NANO (D6) e é comandado pelo app via flag do link.
#define BRAKE_LIGHT_ENABLED     1
#define PIN_BRAKE_LIGHT         23
#define BRAKE_LIGHT_LEDC_CH     7      // canal LEDC livre (MCPWM é separado)
#define BRAKE_LIGHT_DIM_DUTY    28     // [0-255] lanterna (com farol ligado)
#define BATT_VMAX_V          42.0f   // [V] tensão de carga cheia: regen CORTA
                                     // acima disso (camada dura sobre a curva)

// Eco automático (modo ECO): em cruzeiro estável, o firmware assume o comando
// e desce a tensão até o mínimo que segura a velocidade dentro da tolerância.
#define ECO_AUTO_TOL_KMH     1.5f    // [km/h] afundamento tolerado (economia)
#define ECO_AUTO_ENGAGE_MS   3000    // [ms] de cruzeiro estável p/ engatar
#define ECO_AUTO_TRIM_V_S    0.06f   // [V/s] descida lenta da tensão
#define ECO_AUTO_RECOV_V_S   0.50f   // [V/s] recuperação quando afunda demais

// ============================================================================
// 8d) FEATURES OPCIONAIS — estado PADRÃO de cada toggle (0 = off, 1 = on)
// ----------------------------------------------------------------------------
// Estes são os DEFAULTS de fábrica dos parâmetros de feature. Todos podem ser
// ligados/desligados AO VIVO pelo app (Oficina). Config atual = "SÓ O BÁSICO":
// FOC + acelerador->tensão + foldback de corrente + limite de velocidade +
// proteções. Tudo que é "esperto"/adaptativo nasce DESLIGADO p/ um teste limpo.
//
// IMPORTANTE: parâmetro salvo na NVS tem prioridade sobre estes defaults. Para
// que estes valores passem a valer, faça "Restaurar padrões de fábrica" no app
// depois de gravar (ou os parâmetros nunca terem sido salvos).
// ============================================================================
#define FEAT_COAST_DEFAULT         0   // roda-solta regulada ao soltar (adaptativo)
#define FEAT_BATT_LEARN_DEFAULT    0   // aprendizado de capacidade da bateria
#define FEAT_MTPA_DEFAULT          0   // busca de ângulo de mínima corrente
#define FEAT_ECO_AUTO_DEFAULT      0   // cruzeiro econômico automático
#define FEAT_TRAC_DERATE_DEFAULT   0   // derate de tração por tensão (curva TRAC_V)
#define FEAT_THERM_DERATE_DEFAULT  0   // derate térmico gradual (curva THERMAL)
// (Field weakening: injeção de Id NEGATIVO pela malha de d — P_FW_MAX_A,
//  default 0 = desligado; joelho em P_FW_START, % de utilização do teto.)
// Proteções DURAS (corte por sub/sobretensão, sobretemperatura, sobrecorrente)
// NÃO são toggle — seguem sempre ativas. Dependem da calibração dos sensores!

// ============================================================================
// 9) BLUETOOTH (app React Native) — substituiu a dashboard web/WiFi
// ----------------------------------------------------------------------------
// Servidor BLE (NimBLE) no núcleo 0: telemetria por notify a 10 Hz e
// comandos por write (protocolo em ble_dashboard.h — os mesmos UUIDs estão
// no app, pasta "Patinente APP"). Sem WiFi no firmware: menos RAM/flash e
// picos de corrente 3x menores no 3,3 V (os Halls agradecem).
// ============================================================================
#define BLE_DEVICE_NAME         "PatineteESP"
#define BLE_TASK_TICK_MS        20       // [ms] tick da task BLE (base de tudo)
#define BLE_TELEM_MS            100      // [ms] período do notify de telemetria
                                         //      (múltiplo de BLE_TASK_TICK_MS)

// Dead-man: sem comando (painel físico OU app) nesse intervalo, o alvo é
// zerado. Protege contra queda de link com acelerador apertado.
#define CMD_TIMEOUT_MS          600      // [ms]

// ============================================================================
// 9b) PAINEL FÍSICO NO GUIDÃO (Arduino Nano) — acelerador, freio e display
// ----------------------------------------------------------------------------
// Link de UM FIO (half-duplex, 9600 baud): o ESP32 manda telemetria a 10 Hz
// e o Nano responde com acelerador/freio. Cabeçalhos distintos por direção
// (AA55 / BB66) + CRC8 — cada lado ignora o próprio eco naturalmente.
// Com o painel ATIVO (frame fresco), o acelerador/freio da dashboard são
// ignorados (o físico manda); e-stop e limites da dashboard continuam valendo.
// Sem frames por PANEL_TIMEOUT_MS, o dead-man existente zera o alvo.
// Fiação: ver README ("Painel"). Linha em nível 5 V no cabo; canal sobrando
// do módulo BSS138 desce p/ 3,3 V no lado do ESP32.
// ============================================================================
#define PANEL_UART_ENABLED  1
#define PIN_PANEL_RX        16     // RX2 do ESP32 (liga direto na linha 3,3 V)
#define PIN_PANEL_TX        17     // TX2 (liga na linha ATRAVÉS do diodo 1N4148)
#define PANEL_BAUD          9600
#define PANEL_PERIOD_MS     100    // [ms] ciclo telemetria->resposta (10 Hz)
#define PANEL_TIMEOUT_MS    400    // [ms] painel "vivo" = frame mais novo que isso
#define PANEL_CTRL_STALE_MS 250    // [ms] sem resposta do painel por isso => zera
                                   // acelerador/freio na hora (não fica segurando
                                   // o último valor até o dead-man de 600 ms)

// ============================================================================
// 10) FREERTOS
// ============================================================================
#define MOTOR_TASK_PRIO         20       // núcleo 1 — controle. 20 = acima de
                                         // TODAS as tasks de rede (tcpip=18,
                                         // async_tcp=10): nenhuma consegue
                                         // preemptar o loop FOC, mesmo que
                                         // alguma migre de núcleo. Abaixo só
                                         // do ipc1/eventos (24, raros/breves).
#define WEB_TASK_PRIO           1        // núcleo 0 — WiFi/dashboard
#define MOTOR_TASK_STACK        8192
#define WEB_TASK_STACK          8192
