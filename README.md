# Patinete ESP — controlador BLDC M365/Xbot (ESP32 + SimpleFOC)

Firmware para ESP32 substituindo o LKS32MC de uma controladora M365/Xbot,
reaproveitando o estágio de potência original.

> **Hardware e ligações**: a referência completa e atualizada de TODOS os
> componentes e conexões (ESP32, placa M365, halls, INA226, link do painel,
> Nano, OLED, luzes) está em [`HARDWARE.md`](HARDWARE.md).

> **⚠️ CONFIRA O MÓDULO INA240 ANTES.** Módulo pronto não expõe os pinos REF,
> e se a placa amarrou os dois no GND ele é **unidirecional** — não mede
> corrente de fase, que é alternada. O firmware detecta isso sozinho (mede o
> zero e classifica) e mostra no Monitor do app, mas o conserto é físico:
> REF1 no 3,3 V e REF2 no GND. Ver [`HARDWARE.md`](HARDWARE.md) §4b.

> **⚠️ PRIMEIRO BOOT DESTA VERSÃO: o motor NÃO LIGA até calibrar.** É de
> propósito. A malha de corrente só é habilitada com a calibração do sensor de
> fase válida na NVS, porque um sensor com o sinal invertido transforma a malha
> em realimentação positiva — o motor dispara ao acelerar e não para quando
> você solta. Com a **roda suspensa**, rode *Oficina → Parâmetros → Calibrar
> corrente de fase*. Ela mede (não chuta) o mapa de fases, o sinal e a escala
> de cada canal. (O instante de amostragem não se calibra: é derivado da
> configuração do PWM — ver `phase_current.h`.)

> **v3 — MALHA DE CORRENTE DE FASE (ago/2026)**: com os INA240 instalados,
> todos os estimadores de back-EMF que faziam as vezes de limite de corrente
> foram REMOVIDOS: `VOVER_HARD_MAX_V`, `STALL_V_HEADROOM`/`P_HEADROOM`,
> `P_ILIM_FF`, `SPEED_CAP_MARGIN_V` e o stall por tensão. No lugar entraram
> um alvo de corrente (`P_IQ_MAX`), um trip por magnitude `|I|` medida
> (`P_IPH_TRIP`) e um limitador de velocidade que afunila corrente. O Ke
> (`P_KE`) sobrou só como diagnóstico. Detalhes em `include/phase_current.h`.

> **v2 — dashboard por BLUETOOTH**: a dashboard web/WiFi foi substituída pelo
> app React Native (pasta `../Patinente APP`) falando BLE com o firmware
> (NimBLE, núcleo 0). Ganhos: −20% de flash, −3% de RAM, e picos de corrente
> de rádio 3x menores no 3,3 V. O painel físico (Nano) continua igual.
> Referências a "dashboard web" abaixo são históricas — os controles citados
> existem hoje no app.

Estágio de potência:

- **3x FD2103S em modo 3-PWM** — HIN+LIN unidos: 1 sinal por fase, dead-time
  (~100 ns) e anti-shoot-through **dentro do driver**. O ESP32 nunca gera par
  complementar.
- **Halls 5 V → 3,3 V** via conversor de nível (tratados como HallSensor comum).
- **Corrente de FASE medida** por 2x INA240A1 em shunts low-side (GPIO 34 e
  39, ADC1, ISR própria sincronizada aos eventos do MCPWM: corrente no centro
  da janela low-side, **zero vivo** no centro da janela high-side — shunt sem
  corrente = saída do INA240 é o próprio REF). O torque roda em **malha de
  corrente** (PI próprio sobre o Iq medido): o acelerador pede ampères.
- **PWM em 16 kHz** — escolhido pela janela de amostragem, não por conforto
  acústico: o shunt low-side só mede enquanto o MOSFET de baixo conduz, e o
  firmware **limita o teto de modulação** para o S&H do ADC sempre cair dentro
  da janela (cada amostra ainda é validada contra o compare real do PWM).
  Assim toda amostra é medição real.
- **Shunt no barramento DC (INA226)** — limite de corrente de BATERIA
  (foldback) e proteção durante a calibração da corrente de fase.
- **Divisor de tensão do pack** (subtensão) e **NTC nos MOSFETs** (corte térmico).

Toda a configuração (pinos, limiares, calibrações, WiFi) está em
[include/config.h](include/config.h) — procure por **"AJUSTE AQUI"**.

## Pinagem (o que liga onde)

A tabela COMPLETA e comentada de todo o sistema está em
[`HARDWARE.md`](HARDWARE.md) §2. Resumo do ESP32:

| GPIO | Sinal | Liga em |
|---|---|---|
| **19 / 18 / 5** | PWM fases A / B / C | entrada (HIN+LIN unidos) dos FD2103S |
| **33 / 35 / 32** | Halls A / B / C | saídas 3,3 V dos conversores de nível |
| **34 / 39** | corrente de FASE | saídas dos 2x INA240A1 (shunts low-side) |
| **21 / 22** | I²C SDA / SCL | INA226 do barramento (0x44) |
| **25 / 26 / 27** | ADC2 lentos | tensão do pack, NTC dos MOSFETs, NTC do motor |
| **16 / 17** | link do painel | RX direto, TX através de 1N4148 |
| **23** | luz de freio | gate de MOSFET N |
| **GND** | referência | ⚠️ **GND da controladora** — obrigatório |

Notas de ligação:

- **Todos os sensores estão ativos** (`*_SENSE_ENABLED 1`): corrente de fase,
  corrente/tensão de barramento, NTC dos MOSFETs e NTC do motor. As proteções
  por firmware dependem deles estarem calibrados — confira a escala de cada um
  antes de subir os limites.
- Os INA240 são alimentados em **3,3 V** com REF em 1,65 V. Alimentar em 5 V
  colocaria até ~4,8 V nos GPIO 34/39 (máx. 3,3 V) e **queima a entrada**.
- **Não ligue nada do ESP32 no 5 V dos Halls** — só na saída 3,3 V dos
  conversores de nível.
- A corrente de fase fica no **ADC1** (34/39): é o único sincronizável com o
  MCPWM. Os sensores lentos foram para o ADC2 (25/26/27), que só seria
  problema com WiFi — e o firmware é BLE.
- GPIO 34/35/39 são **entrada-somente** e **sem pullup interno**.
- Os pinos de PWM foram escolhidos por não serem de *strapping* (nada de
  0, 2, 12, 15) — não mexem no boot nem soltam pulsos ao resetar.
- Para **inverter o sentido de giro** sem ressoldar: use `P_DIR_INVERT` no
  app (aplica no próximo boot) e recalibre o ângulo zero.

## Arquitetura

| Núcleo | Task | Função |
|--------|------|--------|
| 1 | `motorControlTask` (prio 4) | `loopFOC()`/`move()` sem bloqueios + proteções a 1 kHz |
| 0 | `webDashboardTask` (prio 1) | WiFi, ESPAsyncWebServer, WebSocket a 10 Hz |

Os núcleos se comunicam apenas por `Command`/`Telemetry` copiadas sob spinlock
(`shared_state.*`). A dashboard envia keepalive a 5 Hz; se o link cair por
mais de `CMD_TIMEOUT_MS` (600 ms), o alvo é zerado (**dead-man**).

## Sequência de primeiro teste

> ⚠️ **GND comum**: o GND do ESP32 deve estar ligado ao GND da controladora
> (referência dos FD2103S, do shunt e dos conversores de nível). Sem isso nada
> funciona — e pode queimar coisas.

1. **Fonte de bancada com limite de corrente** (~2 A) no lugar da bateria.
   Roda **suspensa**. Nunca faça o primeiro teste na bateria.
2. **Malha aberta primeiro**: em `config.h`, `OPEN_LOOP_TEST 1` → gravar.
   Abra o app (BLE, dispositivo **PatineteESP**), **LIGAR MOTOR** e suba o
   acelerador devagar. (A dashboard WiFi não existe mais — o AP
   `Patinete-OTA` só sobe no modo de manutenção, cmd 40.)
   - Girou liso e corrente baixa → potência e ordem de fases OK.
   - Vibra/range e não gira, ou corrente alta → **troque dois `PIN_PWM_*`**
     no `config.h` e teste de novo.
3. **Fechar a malha**: volte `OPEN_LOOP_TEST 0` → gravar. No boot o
   `initFOC()` **gira o motor** para calibrar os Halls e imprime no serial:
   ```
   [motor] initFOC OK. ANOTE NO config.h -> MOTOR_ZERO_ELEC_ANGLE 2.0944f | MOTOR_SENSOR_DIR 1
   ```
   Cole esses valores no `config.h` — os próximos boots não precisarão girar
   o motor para calibrar.
> ⚠️ **Mexeu na fiação de Hall ou de fase? RECALIBRE antes de rodar**: coloque
> `MOTOR_ZERO_ELEC_ANGLE NAN` e `MOTOR_SENSOR_DIR 0` por um boot e anote os
> valores novos. A calibração fixada é um retrato da fiação do momento — com
> ela desatualizada o motor treme e não parte (o boot mostra "Skip dir calib.",
> ou seja, ele confia cegamente no que está no config.h). O autoteste
> (`SELF_TEST 1`) ajuda a separar fase × Hall × calibração quando houver dúvida.

4. **Sentido de giro**: o SimpleFOC compensa a direção do sensor por software.
   Se quiser inverter o que é "para frente", troque dois `PIN_PWM_*` **e** os
   dois `PIN_HALL_*` correspondentes (ex.: A↔B nos dois), ou inverta
   `MOTOR_SENSOR_DIR`.
5. **Calibrações** (tudo em `config.h`):
   - `VBAT_R_TOP/VBAT_R_BOTTOM/VBAT_CAL_GAIN` — compare com multímetro;
   - `CUR_MV_PER_A` — compare a corrente da dashboard com a da fonte
     (o offset de 0 A é medido sozinho no boot, com o motor parado);
   - `NTC_BETA/NTC_R25/NTC_SERIES_R/NTC_TO_GND` — conforme seu NTC/divisor;
   - `VOLTAGE_POWER_SUPPLY` — tensão típica do pack.
6. **Calibre a corrente de fase** — sem isto o motor não liga. Roda suspensa,
   fonte limitada: *Calibrar corrente de fase*. O serial mostra a matriz de
   resposta e a conclusão (evento do MCPWM, fase de cada pino, sinal de cada
   canal). Se falhar, ele diz em qual etapa.
7. **Confira antes de acelerar**: no Monitor, com a roda parada, |I| ≈ 0 A.
   Gire a roda na mão: as três fases oscilam com amplitudes parecidas.
8. **Suba o torque aos poucos**: `P_IQ_MAX` nasce em 10 A. Acelere devagar e
   verifique no Monitor que **Iq medido segue Iq pedido** (o card ERRO Iq fica
   perto de zero). Se Iq medido tiver sinal contrário ao pedido, PARE — está
   invertido, e o firmware vai cortar sozinho em 150 ms com falta de sensor.
9. **Ajuste o PI de corrente**: aba Tuning, séries *Iq med* e *Iq alvo*. Suba
   `P_CUR_P` até aparecer oscilação e recue ~30 %; depois `P_CUR_I` até o erro
   em regime sumir sem sobressinal.

## Dashboard

- Mostra: km/h, rpm, corrente, tensão, temperatura, modo, estado, faltas,
  frequência real do loop FOC e % de potência disponível (foldback).
- Controla ao vivo: enable, acelerador, freio (manete: volta a 0 ao soltar),
  modo torque/velocidade, limite de corrente, velocidade máx. e voltage_limit.
- **⛔ PARADA DE EMERGÊNCIA**: zera alvo + desabilita + trava. Destrava no
  botão **Limpar faltas**.
- Use **uma aba só** — várias abas enviando acelerador brigam entre si.
- Faltas travadas (sobrecorrente/subtensão/sobretemperatura) só limpam se a
  condição física tiver normalizado.

## Painel do guidão (Arduino Nano) — projeto irmão `../Patinete Arduino/`

Acelerador + freio físicos e display OLED, falando com o ESP32 por **um fio**
(UART half-duplex 9600, telemetria a 10 Hz, CRC8 nos dois sentidos). Com o
painel vivo, os sliders da dashboard viram só-leitura (badge "PAINEL").

**Nano** (projeto PlatformIO `../Patinete Arduino/` — libs baixam sozinhas; gravar com `pio run -t upload`):

| Pino Nano | Liga em |
|---|---|
| A0 | sinal do acelerador (hall, 0,8–4,2 V) |
| A1 | sinal do freio (hall) |
| D2 | botão → GND (curto = troca tela, 1,5 s = zera trip) |
| A4 / A5 | SDA / SCL do OLED 0,96" (VCC 5 V, GND) |
| D8 | linha de dados (RX, direto) |
| D9 | linha de dados através de **diodo 1N4148** (catodo no D9) |
| 5V / GND | do cabo vindo do fundo do patinete |

**ESP32** (lado do fundo): GPIO **16** (RX2, direto na linha 3,3 V) e GPIO
**17** (TX2, através de **diodo 1N4148**, catodo no 17).

**A linha única**: roda em nível **5 V** ao longo do cabo (pullup 4,7k→5 V em
qualquer ponto do lado 5 V); no lado do ESP32, passa por **um canal sobrando
do módulo BSS138** (HV = linha 5 V, LV = linha 3,3 V com o pullup do módulo).
Cada lado só *puxa a linha para baixo* pelo diodo (estilo open-drain) e escuta
direto — half-duplex sem colisão porque o protocolo é pergunta-resposta.

Calibração do acelerador/freio: o Nano imprime os mV crus na USB (115200,
`pio device monitor`) — ajuste `TH_MIN/MAX_MV` e `BR_MIN/MAX_MV` no topo de
`src/main.cpp` do projeto do painel. Failsafe: sensor fora de 0,3–4,8 V =
considerado solto = comando zero.

## Avisos do hardware (3-PWM com HIN+LIN unidos)

- **Não existe estado de alta impedância**: motor "desligado" = MOSFETs de
  baixo conduzindo = fases em curto ao GND. Girar a roda na mão com o sistema
  alimentado tem arrasto (freio dinâmico). É do hardware, não é bug.
- Freio elétrico devolve corrente ao barramento (**regen**). Fonte de bancada
  não absorve regen — freadas fortes com fonte podem elevar a tensão do
  barramento. Na bancada, freie suave; `REGEN_I_LIMIT_A` limita isso via
  firmware (conservador se o amplificador do shunt for unidirecional).
- **Ruído do PWM nos cabos de Hall** (confirmado e RESOLVIDO nesta placa): o
  chicote original traz fases e Halls juntos e o chaveamento induz ruído nas
  linhas. **Solução validada (jul/2026): pullup de 1 kΩ para 5 V + capacitor
  de 1 nF em CADA linha de Hall, no lado 5 V do conversor de nível.** A chave
  é impedância: ruído capacitivo induzido é proporcional à impedância da
  linha — 1 k deixa a linha ~10× mais "dura" que os 10 k usuais. Com isso os
  contadores da dashboard (lento/skip/rev/inv/fix) zeraram. O firmware ainda
  mantém as defesas (filtro de glitch, espera-de-assentamento, reconciliação)
  como rede de segurança.
