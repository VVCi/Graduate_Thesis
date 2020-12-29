-include sources/platform/stm32f4/Libraries/STM32F4xx_StdPeriph_Driver/Makefile.mk
-include sources/platform/stm32f4/Libraries/NNN_library/Makefile.mk
-include sources/platform/stm32f4/arduino/Makefile.mk
-include sources/platform/stm32f4/fatfs/Makefile.mk

LDFILE = sources/platform/stm32f4/ak.ld

CFLAGS += -I./sources/platform/stm32f4
CFLAGS += -I./sources/platform/stm32f4/Libraries/CMSIS/Device/ST/STM32F4xx/Include
CFLAGS += -I./sources/platform/stm32f4/Libraries/CMSIS/Include
CFLAGS += -I./sources/platform/stm32f4/Libraries/STM32F4xx_StdPeriph_Driver/inc

CPPFLAGS += -I./sources/platform/stm32f4

VPATH += sources/platform/stm32f4
VPATH += sources/platform/stm32f4/Libraries/STM32F4xx_StdPeriph_Driver/src

# C source files
SOURCES += sources/platform/stm32f4/stm32f4.c
SOURCES += sources/platform/stm32f4/system.c
SOURCES += sources/platform/stm32f4/sys_cfg.c
SOURCES += sources/platform/stm32f4/io_cfg.c
SOURCES += sources/platform/stm32f4/system_stm32f4xx.c

# C++ source files
SOURCES_CPP += sources/platform/stm32f4/mini_cpp.cpp
