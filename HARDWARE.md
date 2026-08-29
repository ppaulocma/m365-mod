# Patinete ESP — Hardware e Ligações

Documento de referência de TODO o hardware do sistema e de cada ligação
elétrica. Fiel ao firmware em 28/jul/2026 (pinos e valores vêm do
`include/config.h` — se mudar lá, atualize aqui).

O sistema tem três módulos:

```
┌─────────────┐  BLE (NimBLE)   ┌──────────────────┐  UART 1 fio 9600  ┌─────────────┐
│  APP (RN)   │ ◄─────────────► │  ESP32 DevKit    │ ◄───────────────► │ Nano (painel)│
│ PatineteApp │                 │ + placa M365     │    AA55 / BB66    │ OLED+manetes │
└─────────────┘                 │   (potência)     │                   │ FAROL+botão  │
                                └───────┬──────────┘                   └─────────────┘
                                        │ 3×PWM / 3×Hall / 2×INA240 / INA226 / luz
                                ┌───────┴──────────┐
                                │ Motor hub M365   │  15 pares de polos, halls internos
                                └──────────────────┘
```

---

## 1. Lista de hardware (BOM)

| item | detalhe | função |
|---|---|---|
| ESP32 DevKit V1 | 38 pinos | cérebro: FOC, proteções, BLE, luzes |
| Controladora M365 | reaproveitada; LKS32MC **removido** | estágio de potência |
| — 3× FD2103S | gate drivers half-bridge, bootstrap | acionam os MOSFETs (HIN+LIN unidos = 3-PWM) |
| — 6× MOSFET N | ponte trifásica original da placa | chaveiam as fases |
| Motor hub M365 | BLDC, 15 pares de polos, 3 halls internos | tração |
| INA226 (módulo) | I²C **0x44**, shunt externo **4,085 mΩ** (calibrado) | corrente/tensão de BARRAMENTO |
| 2x INA240A1 | ganho 20 V/V, V+ 3,3 V, REF 1,65 V | corrente de FASE (shunts low-side) |
| — 2x shunt low-side | source do MOSFET de baixo, valor no `P_PH_SHUNT` | sensor da corrente de fase |
| Conversor de nível | canal por hall, com RC no lado alto | halls 5 V → 3,3 V |
| — 3× pullup 10 kΩ | lado 5 V (alto) do conversor | polarização dos halls |
| — 3× capacitor 1 nF | hall → GND, lado alto | filtro de ruído |
| Arduino Nano | ATmega328 (clone) | painel do guidão |
| OLED SSD1306 0,96" | I²C, **bicolor** (16 linhas amarelas + azul) | display |
| Manete acelerador | sensor hall analógico (~0,8–4,2 V) | comando |
| Manete freio | sensor hall analógico | comando |
| 1× diodo 1N4148 | no TX do ESP (lado ESP do link) | barramento 1 fio |
| 1× diodo 1N4148 | no D9 do Nano (lado Nano) | idem |
| 1× diodo 1N4148 "A" | linha POWER-ON → nó do botão (catodo no nó) | botão liga sem vazar pullup (§8b) |
| 1× diodo 1N4148 "B" | D2 → nó do botão (catodo/faixa no nó) | impede ligar sozinho c/ Nano off (§8b) |
| NPN comum (BC547/2N2222) + 10 kΩ na base | base no D5 do Nano | **trava de energia** (§8b; MOSFET 2N7000 também serve) |
| Push button | no guidão | liga / modo / telas / desliga (5 s) |
| MOSFET N + LED traseiro | gate no GPIO 23 do ESP | **luz de freio / lanterna** |
| MOSFET N + farol LED | gate no D6 do Nano | **farol** |
| NTC 10 kΩ (β 3950) | junto aos MOSFETs, divisor c/ 10 kΩ | temperatura da ponte |
| Divisor resistivo | 100 kΩ / 4,7 kΩ | tensão do pack no ADC |
| Bateria 10s li-ion | 36 V nominal / 42 V cheia | energia |

---

## 2. ESP32 — mapa de pinos

| GPIO | direção | liga em | observações |
|---|---|---|---|
| 19 | saída PWM | entrada (HIN+LIN) do FD2103S da **fase A** | 3-PWM, 20 kHz (parâmetro) |
| 18 | saída PWM | FD2103S **fase B** | |
| 5  | saída PWM | FD2103S **fase C** | |
| 33 | entrada | **Hall A** (saída 3,3 V do conversor) | interrupção + filtro de glitch |
| 35 | entrada | **Hall B** | entrada-somente, sem pullup interno |
| 32 | entrada | **Hall C** | |
| 34 | ADC1 | **INA240A1 canal 1** — corrente de fase | entrada-somente; ver §4b |
| 39 (VN) | ADC1 | **INA240A1 canal 2** — corrente de fase | idem |
| 21 | I²C SDA | INA226 SDA | |
| 22 | I²C SCL | INA226 SCL | |
| 25 | ADC2 | divisor de tensão do pack | 100 kΩ (pack→pino) / 4,7 kΩ (pino→GND) |
| 26 | ADC2 | divisor do NTC da CONTROLADORA | NTC 10 kΩ → GND, série 10 kΩ → 3,3 V |
| 27 | ADC2 | divisor do NTC interno do MOTOR | idem, com β/R25 próprios |
| 16 (RX2) | entrada | linha do link do painel (direto) | barramento 1 fio, 3,3 V |
| 17 (TX2) | saída | linha do link **através de 1N4148** | catodo no GPIO 17 (open-drain) |
| 23 | saída PWM | gate do MOSFET da **luz de freio** | LEDC canal 7, 2 kHz |
| 3V3 / GND | — | INA226, INA240, conversor lado baixo, NTC | |

**Por que os lentos foram para o ADC2**: a corrente de fase precisa de ADC1
(é o único que o MCPWM consegue disparar em sincronismo e o único imune a
rádio). Vbat e os dois NTCs mudam devagar e foram para o ADC2, que só seria
problema com WiFi ligado — e o firmware é BLE.

**Nunca ligue WiFi em operação** — só o modo OTA usa (cmd 40 pelo app).

---

## 3. Estágio de potência (placa M365)

- O microcontrolador original (LKS32MC) foi removido; os 3 sinais de PWM do
  ESP entram direto nos FD2103S.
- Cada FD2103S tem HIN e LIN **unidos** → modo 3-PWM com dead-time interno.
  Consequências que o firmware respeita:
  - não existe alta impedância: fases sempre "seguradas" (desligado = fases
    ao GND — arrasto suave ao girar na mão é normal);
  - bootstrap não permite 100% de duty → `MODULATION_CEIL_FACTOR 0.55`;
  - modulação **não-centrada** (`modulation_centered=0`): em repouso os
    duties ficam em 0 (sem chaveamento parado, sem "dentes" na roda).
- Fases A/B/C da placa → fios de fase do motor (bullets). **Conector de fase
  oxidado/folgado = estralos intermitentes sob carga** — suspeito nº 1.

## 4. Halls (motor → conversor → ESP)

```
Motor (5V)          Lado ALTO (5 V)                 Lado BAIXO   ESP32
HALL x ────┬── pullup 10 kΩ → 5 V
           ├── capacitor 1 nF → GND      → conversor de nível →  GPIO 5/18/19
           └── (fio o mais longe possível dos fios de fase!)
```

- Alimentação dos halls: 5 V e GND do chicote original do motor.
- Firmware: filtro de glitch 100 µs + confirmação 3 µs + verificador de
  sequência (`HALL_DIAG 1` liga o diagnóstico completo pelo serial).

## 4b. Corrente de FASE — 2x INA240A1 (shunts low-side)

```
     fase X do motor
          |
     [MOSFET de baixo]
          |  source
          +----------------+---- IN+  \
       [SHUNT low-side]    |           > INA240A1  OUT -> GPIO 34 (ou 39)
          |                +---- IN-  /   V+ = 3,3 V   REF = 1,65 V
         GND
```

| INA240 (módulo) | liga em |
|---|---|
| VIN / V+ | **3,3 V** do ESP (nunca 5 V: a saída passaria de 3,3 V no ADC e mataria o pino) |
| GND | GND comum |
| IN+ / IN− | por cima do shunt low-side, respeitando a polaridade |
| OUT | GPIO **34** (canal 1) e GPIO **39/VN** (canal 2) |

### O pino REF — o que decide se o módulo serve

Módulos prontos não expõem os pinos REF do INA240: eles já vêm amarrados na
placa, e é isso que define onde fica o zero da saída (`Vout = Vref + 20×Vshunt`).

| REF na placa | Zero da saída (V+ = 3,3 V) | Serve? |
|---|---|---|
| REF1 = V+, REF2 = GND | ~1,65 V | **sim** — mede os dois sentidos |
| REF1 = REF2 = GND | ~0 V | **não** — só corrente positiva |
| REF1 = REF2 = V+ | ~3,3 V | **não** — só corrente negativa |

A corrente de fase é **alternada**: troca de sinal a cada ciclo elétrico. Um
módulo unidirecional ceifa metade da onda e a transformada de Park sai sem
sentido — não há correção por software.

O firmware **mede** o zero de cada canal (corrente comprovadamente nula, motor
desabilitado) e classifica. O resultado aparece no boot, no Monitor do app
(cards **ZERO CH1/CH2**) e no aviso vermelho. Se não der bidirecional, a
calibração é recusada e o motor fica bloqueado.

**Conserto quando o módulo é unidirecional**: no INA240 os dois pinos REF
entram num divisor interno de 100 kΩ + 100 kΩ. Ligando **REF1 no 3,3 V e REF2
no GND** o zero vai para o meio da alimentação sem nenhum componente externo.
Se a placa tem os dois no GND, é levantar o REF1 e levá-lo ao 3,3 V.
Alternativa sem mexer no CI: alimentar o REF por um divisor externo de 3,3 V
(2× 10 kΩ) com 100 nF para o GND.

### Dimensionamento do shunt de fase (INA240A1, ganho 20, V+ 3,3 V)

Com o zero em ~1,52 V a excursão útil até os trilhos é ±1,32 V. Daí sai tudo:

| shunt | escala | fundo de escala | resolução | dissipação a 20 A / 45 A |
|---|---|---|---|---|
| **R100 = 0,1 Ω** | 2000 mV/A | **0,7 A** | — | **40 W / 202 W** |
| R010 = 10 mΩ | 200 mV/A | 6,6 A | 0,004 A/cnt | 4 W / 20 W |
| **R001 = 1 mΩ** | 20 mV/A | **66 A** | **0,040 A/cnt** | 0,4 W / 2 W |
| 0,1 mΩ | 2 mV/A | 660 A | 0,42 A/cnt | ~0 |

**R001 (1 mΩ) é o valor certo** para este ganho e esta faixa de corrente: cobre
os 48 A do corte instantâneo com folga, dá 0,04 A por contagem de ADC e dissipa
2 W no pior caso. Use elemento metálico de potência (2512 ou maior, ≥3 W).

Shunt **grande demais é perigoso**, não só impreciso: acima da saturação a
leitura fica MENOR que a corrente real, a malha enxerga folga que não existe e
integra a tensão até o teto — o mesmo desfecho do sinal de sensor invertido. O
firmware recusa a calibração quando o fundo de escala não cobre o trip de fase
com 30 % de margem, e corta em 30 ms se a saída encostar num trilho rodando.

### A resolução depende do par shunt × ganho

A escala medida diz quantos ampères cabem numa contagem do ADC, e isso limita a
qualidade da malha de corrente:

| shunt equivalente | ganho | escala | A por contagem |
|---|---|---|---|
| 0,1 mΩ | 20 (A1) | 2 mV/A | **0,40 A** — grosso |
| 0,1 mΩ | 200 (A4) | 20 mV/A | 0,04 A |
| 1 mΩ | 20 (A1) | 20 mV/A | 0,04 A |

Com ~0,4 A por contagem a malha funciona, mas o Iq carrega ~1 A de ruído de
quantização, o que vira um leve chiado de torque. Melhorar é **hardware**:
INA240**A4** (ganho 200) no lugar do A1, ou shunt maior. O firmware imprime a
resolução medida no fim da calibração e avisa quando passa de 0,2 A/contagem.

### O valor do shunt não precisa ser conhecido

A escala (A/V) é **medida** na calibração: com o rotor parado e um vetor DC numa
fase, a corrente do barramento se relaciona com a de fase pelo duty daquela
fase (`I_bus = duty × I_fase`). O INA226 já mede `I_bus` e o firmware sabe o
duty que aplicou, então `I_fase` sai da conta — e dividindo pela resposta em
volts do INA240 sai a escala real. `P_PH_SHUNT` fica só como reserva. A
precisão herda a do INA226, então calibre-o primeiro.

- Saída = `1,65 V + 20 × R_shunt × I`. O zero real é medido no boot com o motor
  desabilitado e conferido: fora de 1,2–2,1 V o firmware recusa e limita o
  torque a 5 A (aparece no app).
- A terceira corrente sai de `ia + ib + ic = 0`. Qual fase está em qual pino e
  o sinal de cada canal **não se configuram**: são medidos pela calibração
  (comando 26, roda suspensa) e gravados na NVS.
- **Instante de amostragem**: o PWM é center-aligned (contador up-down do
  MCPWM); a janela em que os MOSFETs de baixo conduzem é `(1 − duty) × T_pwm`
  **centrada no evento TEP** do timer. A corrente é amostrada nesse centro —
  o ponto mais longe das bordas de chaveamento, onde a corrente vale a própria
  média do período (o ripple se cancela sem filtro).
- **Zero vivo**: no evento oposto (TEZ, centro da janela do MOSFET de cima) o
  shunt está sem corrente e a saída do INA240 é exatamente o REF — o zero
  daquele canal é amostrado ali a 8 kHz, já incluindo deriva do trilho de
  3,3 V, deriva térmica e a oscilação da referência do ADC induzida pelo rádio
  BLE. A subtração corrente − zero cancela o modo comum por construção. (O
  divisor de referência no GPIO 36 deixou de existir; o pino está livre.)
- **Amostra validada**: cada conversão registra o contador do timer no disparo
  e é descartada se o S&H não coube na janela de condução (comparado com o
  compare real do PWM). O boot mede a latência da ISR e limita o teto de
  modulação por `duty_max = 1 − 2·(latência + S&H + margem)/T_pwm` — o valor
  real aparece no serial e no card **DUTY MÁX**.
- O PWM (16 kHz por padrão) é escolhido por esta janela: período maior = janela
  maior = menos teto de modulação sacrificado (ver tabela no `config.h`).

## 5. INA226 (corrente/tensão de barramento)

| INA226 | liga em |
|---|---|
| VCC | 3V3 do ESP |
| GND | GND comum |
| SDA / SCL | GPIO 21 / 22 |
| IN+ / IN− | nos dois lados do shunt de **4,085 mΩ** (calibrado) no negativo do barramento |
| A0, A1 | endereço **0x44** (ver `INA226_ADDR`; `I2C_SCAN 1` acha o real) |

Escala configurada: ±19 A. O valor do shunt é o parâmetro interno
`INA226_SHUNT_MOHM` (calibração fina pela corrente lida).

## 6. Link do painel (1 fio, half-duplex, 9600 baud)

Uma única linha de dados em 3,3 V entre ESP e Nano + GND comum:

```
        ESP32                       LINHA                       Nano
  GPIO 16 (RX2) ───────────────────────┬─────────────────────── D8 (RX)
  GPIO 17 (TX2) ──►|── 1N4148 ─────────┤
                (catodo no 17)         ├───────── 1N4148 ──|◄── D9 (TX)
                                       │        (catodo no D9)
                                  pullup p/ 3,3 V
```

- Os dois TX "puxam a linha para baixo" através dos diodos (open-drain de
  pobre); quem fala vê o próprio eco e o outro escuta.
- Protocolo: ESP manda `AA 55 len=16 [payload] crc8` a 10 Hz
  (payload: kmh, vbat, ibat, potência, temp, SoC, flags, faltas,
  **autonomia**, **Wh/km**, **flags2** modo/estados);
  Nano responde `BB 66 len=3 [th br flags] crc8` — flags: b0 acelerador
  válido, b1 freio válido, **b4-5 contador de cliques** (cada incremento =
  um clique do botão = ESP gira o modo eco→normal→turbo).
- Failsafe: 250 ms sem resposta → acelerador/freio zerados no ESP.
- O Nano aceita payload de 12/15/16 bytes (compatibilidade entre versões).

---

## 7. Arduino Nano (painel) — mapa de pinos

| pino | direção | liga em | observações |
|---|---|---|---|
| A0 | ADC | sinal do **acelerador** (~0,8–4,2 V) | fora da faixa = failsafe zero |
| A1 | ADC | sinal do **freio** | |
| D2 | entrada | **botão** (via diodo, ver §8b) | 1 clique = **modo**; 2 cliques = tela; 1,5 s = zera viagem; **5 s = DESLIGA** |
| D8 | entrada | linha do link (direto) | |
| D9 | saída | linha do link **através de 1N4148** | catodo no D9 |
| D5 | saída | base do NPN da **trava de energia** (via 10 kΩ) | HIGH = mantém ligado (§8b) |
| D6 | saída | gate do MOSFET do **FAROL** | comandado pelo app (flag 0x10) |
| A4 / A5 | I²C | OLED SSD1306 (SDA/SCL) | módulo 5 V |
| D13 | — | LED onboard | diagnóstico |
| 5V / GND | — | OLED, manetes | GND comum com o ESP! |

**OLED bicolor**: linhas 0–15 = faixa AMARELA, 16–63 = azul. As telas foram
desenhadas para nada cruzar a divisa (status/alertas no amarelo, dados no
azul; tela BLOQUEADO usa o amarelo para o letreiro).

---

## 8. Luzes

### Luz de freio (ESP, GPIO 23)

```
GPIO 23 ──[100 Ω]── gate ┐
                         │ MOSFET N (fonte → GND)
LED traseiro (+) ── V+ da luz
LED traseiro (−) ── dreno ┘
```

- Acende **cheia**: manete de freio OU regen > 1,5 A (freio-motor conta!).
- Farol ligado + sem freio: **lanterna** (PWM ~11%).
- Tudo apagado: 0. Atualização a 50 Hz.

### Farol (Nano, D6)

Mesmo circuito de MOSFET N no D6. Estado vem do app (BLE cmd 46 → ESP →
flag 0x10 do frame). RAM apenas: reset = apagado. Link caído mantém o
último estado. Ícone de farol aparece no topo do OLED quando ligado.

### 8b. Power on/off (soft-latch estilo M365)

Um único botão liga o patinete (hardware) e desliga (firmware, segurando
5 s). A linha **POWER-ON** da placa M365 é aquela que, aterrada, liga os
conversores (a mesma que o botão original usava).

```
  LINHA POWER-ON ────┬────────────────┐
                     │             coletor
                  anodo           BC337/BC547/2N2222
                 DIODO A          (emissor → GND)
                  catodo             base ──[10 kΩ]── D5 do Nano
                     │
      NÓ DO BOTÃO ───┼─── catodo ──DIODO B── anodo ─── D2 do Nano
                     │    (faixa no nó)
                  BOTÃO (pernas em DIAGONAL!)
                     │
                    GND
```

  São DOIS 1N4148, cada um resolvendo um vazamento:

  - **DIODO A** (linha→nó): o botão puxa a linha para LIGAR, mas o pullup
    do D2 não vaza para a linha. A trava (coletor) fica NA LINHA, separada
    do nó — se trava e D2 dividirem o mesmo nó, o D2 lê LOW eterno e o
    firmware "vê" botão preso (zera trip e desliga sozinho).
  - **DIODO B** (D2→nó, faixa no nó): sem ele, com o Nano DESLIGADO a linha
    vaza pelos diodos de proteção internos do D2 (pino → VCC morto) e o
    patinete LIGA SOZINHO. O diodo B bloqueia esse caminho reverso.

  Alternativa com MOSFET (2N7000/BS170): dreno na linha, fonte no GND,
  gate ←[10 kΩ]← D5 e OBRIGATÓRIO 100 kΩ gate→GND (pull-down de boot).

Funcionamento:

1. **Ligar**: apertar o botão aterra POWER-ON → conversores sobem → ESP e
   Nano bootam. O Nano, na PRIMEIRA linha do setup, põe D5 em HIGH → o
   MOSFET passa a aterrar a linha em paralelo com o botão. **Segure ~2 s**
   (tempo do bootloader do Nano assumir a trava) e solte.
2. **Ligado**: D5 HIGH mantém tudo alimentado. Com transistor NPN comum
   (BC547/2N2222: base ←[10 kΩ]← D5, emissor→GND, coletor→linha), o
   pull-down é dispensável — sem corrente de base, ele desliga sozinho.
   Com MOSFET, mantenha 100 kΩ gate→GND (pino em alta impedância ≠ off).
   Botão de 4 pernas: use pernas em DIAGONAL (pares laterais são ligados
   internamente!).
3. **Desligar**: segurar o botão **5 s** → OLED mostra "DESLIGANDO", o Nano
   derruba D5 e trava num laço; ao soltar o botão, nem o MOSFET nem o
   botão seguram a linha → tudo morre de verdade (consumo zero).
4. O diodo entre a linha e o nó do botão deixa o botão LIGAR o patinete
   (puxa a linha através dele) sem que o pullup do D2 vaze 5 V para a
   linha — e mantém a leitura do botão independente da trava.

Atenção: o ESP **desliga junto sem aviso** — por isso tudo que precisa
persistir (odômetro, parâmetros...) já grava com a roda parada, muito antes
do desligamento.

---

## 9. Alimentação e massas

- **Bateria 10s li-ion** (42,0 V cheia / corte 32,0 V — parâmetros).
- A placa M365 fornece a alimentação abaixada para a lógica (como no
  original). ESP e Nano precisam de **GND comum** (via link e alimentação).
- O shunt do INA226 fica no caminho do barramento — observar polaridade
  IN+/IN− para o sinal de corrente (positivo = tração, negativo = regen).
- Luz de freio e farol podem usar a tensão que o LED pedir (5 V ou pack via
  regulador) — o MOSFET N com gate em 3,3/5 V isola a lógica.

## 10. Resumo dos protocolos (para depurar com lógica/scope)

| canal | físico | taxa | formato |
|---|---|---|---|
| App ↔ ESP | BLE (NimBLE), serviço `7a0b1000-…0001` | telemetria 10 Hz (151 B), stream 25/50 Hz (20 B) | ver `ble_dashboard.cpp` |
| ESP ↔ Nano | 1 fio 3,3 V, 9600 8N1 | 10 Hz ida+volta | `AA55…crc8` / `BB66…crc8` |
| ESP ↔ INA226 | I²C 100 kHz+ | ~1 kHz leitura | registradores INA226 |
| OTA | WiFi AP `Patinete-OTA` / `patinete123` | só em manutenção | `http://192.168.4.1` |

---

*Gerado a partir do firmware; qualquer divergência, o `config.h` é a
fonte da verdade. Última sincronização: 25/ago/2026 (entrada dos INA240 e
da malha de corrente de fase).*
