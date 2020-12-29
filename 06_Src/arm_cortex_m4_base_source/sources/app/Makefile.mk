CFLAGS		+= -I./sources/app
CPPFLAGS	+= -I./sources/app

VPATH += sources/app

CFLAGS += -I./sources/platform/stm32f4
CFLAGS += -I./sources/platform/stm32f4/Libraries/CMSIS/Device/ST/STM32F4xx/Include
CFLAGS += -I./sources/platform/stm32f4/Libraries/CMSIS/Include
CFLAGS += -I./sources/platform/stm32f4/Libraries/STM32F4xx_StdPeriph_Driver/inc

CPPFLAGS += -I./sources/platform/stm32f4

VPATH += sources/platform/stm32f4
VPATH += sources/platform/stm32f4/Libraries/STM32F4xx_StdPeriph_Driver/src

# CPP source files
SOURCES_CPP += sources/app/app.cpp
SOURCES_CPP += sources/app/app_data.cpp
SOURCES_CPP += sources/app/app_non_clear_ram.cpp
SOURCES_CPP += sources/app/app_bsp.cpp
SOURCES_CPP += sources/app/rf_remote_ctrl.cpp
SOURCES_CPP += sources/app/shell.cpp

SOURCES_CPP += sources/app/task_shell.cpp
SOURCES_CPP += sources/app/task_life.cpp
SOURCES_CPP += sources/app/task_fw.cpp
SOURCES_CPP += sources/app/task_list.cpp
SOURCES_CPP += sources/app/task_display.cpp

SOURCES_CPP += sources/app/task_if.cpp
SOURCES_CPP += sources/app/task_rf24_if.cpp
SOURCES_CPP += sources/app/task_uart_if.cpp
SOURCES_CPP += sources/app/task_dbg.cpp
SOURCES_CPP += sources/app/task_encoder_pid.cpp
