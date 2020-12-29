# header path
CPPFLAGS	+= -I./sources/platform/stm32f4/arduino
CPPFLAGS	+= -I./sources/platform/stm32f4/arduino/SPI
CPPFLAGS	+= -I./sources/platform/stm32f4/Libraries/STM32F4xx_StdPeriph_Driver/inc
CPPFLAGS	+= -I./sources/platform/stm32f4/Libraries/CMSIS/Device/ST/STM32F4xx/Include
CPPFLAGS	+= -I./sources/platform/stm32f4/Libraries/CMSIS/Include

# source path
VPATH += sources/platform/stm32f4/arduino
VPATH += sources/platform/stm32f4/arduino/SPI

# CPP source files
SOURCES_CPP += sources/platform/stm32f4/arduino/SPI/SPI.cpp
SOURCES_CPP += sources/platform/stm32f4/arduino/wiring_digital.cpp
SOURCES_CPP += sources/platform/stm32f4/arduino/wiring_shift.cpp
SOURCES_CPP += sources/platform/stm32f4/arduino/Print.cpp
