/* ---------------------------------------------------------------------------
   CAMINHO QUENTE DO CONTROLE EM IRAM
   ---------------------------------------------------------------------------
   PROBLEMA QUE ISTO RESOLVE (medido, nao suposto): o ESP32 executa codigo de
   flash atraves de um cache de instrucao de 32 KB, e o .flash.text deste
   projeto tem ~878 KB — 27x o cache. Quem colide com quem no cache depende do
   ENDERECO. Comparando dois builds que diferiam por 64 bytes numa unica
   funcao, 7626 de 8922 simbolos MUDARAM DE ENDERECO: 85% do binario se
   reposiciona a cada alteracao de tamanho, e o mapa de colisoes e redesenhado
   do zero.

   O sintoma disso e brutal e sem nexo aparente: acrescentar QUALQUER coisa em
   QUALQUER lugar — inclusive uma variavel morta que ninguem le — pode fazer o
   laco de controle passar a bater em cache miss (leitura de flash SPI, ordem
   de microssegundos) e o motor fica aspero com corrente alta. Reverter
   conserta, nao porque a logica era errada, mas porque volta ao layout que
   estava dando sorte. Trocar uma constante por outra nao quebra: nao muda
   tamanho, nao realoca nada.

   IRAM nao passa por cache. Movendo a cadeia que roda a cada iteracao do FOC
   para ca, ela sai da loteria: o tempo dela deixa de depender de onde o
   linker resolveu por o resto do firmware.

   Se estourar a IRAM, o link FALHA — nao gera binario ruim em silencio.
   --------------------------------------------------------------------------- */
SECTIONS
{
  .iram0.hotfoc :
  {
    . = ALIGN(4);
    *BLDCMotor.cpp.o(.literal .literal.* .text .text.*)
    *BLDCDriver3PWM.cpp.o(.literal .literal.* .text .text.*)
    *HallSensor.cpp.o(.literal .literal.* .text .text.*)
    *foc_utils.cpp.o(.literal .literal.* .text .text.*)
    *lowpass_filter.cpp.o(.literal .literal.* .text .text.*)
    *pid.cpp.o(.literal .literal.* .text .text.*)
    . = ALIGN(4);
  } > iram0_0_seg
} INSERT BEFORE .iram0.text
