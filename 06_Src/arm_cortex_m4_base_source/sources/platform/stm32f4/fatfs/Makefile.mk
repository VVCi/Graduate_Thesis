CFLAGS += -I./sources/platform/stm32f4/fatfs
CPPFLAGS += -I./sources/platform/stm32f4/fatfs

CFLAGS += -I./sources/platform/stm32f4/fatfs/drivers
CPPFLAGS += -I./sources/platform/stm32f4/fatfs/drivers

CFLAGS += -I./sources/platform/stm32f4/fatfs/option
CPPFLAGS += -I./sources/platform/stm32f4/fatfs/option

VPATH += sources/platform/stm32f4/fatfs
VPATH += sources/platform/stm32f4/fatfs/drivers
VPATH += sources/platform/stm32f4/fatfs/option

# C source files
SOURCES += sources/platform/stm32f4/fatfs/diskio.c
SOURCES += sources/platform/stm32f4/fatfs/ff.c

SOURCES += sources/platform/stm32f4/fatfs/drivers/fatfs_sd_sdio.c
SOURCES += sources/platform/stm32f4/fatfs/option/syscall.c
SOURCES += sources/platform/stm32f4/fatfs/option/unicode.c
