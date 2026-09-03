# Injeta o fragmento de linker que poe o caminho quente do FOC em IRAM.
# Feito por script (e nao por build_flags) porque o caminho do projeto tem
# espaco, e -Wl,-T nao sobrevive a isso sem citacao.
Import("env")
import os
ld = os.path.join(env["PROJECT_DIR"], "iram_hot.ld")
env.Append(LINKFLAGS=["-T", ld])
env.Depends("$BUILD_DIR/${PROGNAME}.elf", ld)
