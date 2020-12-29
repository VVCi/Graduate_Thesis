Print		= @echo "~"
NAME_MODULE	= arm_cortex_m4_base_application
PROJECT		= $(NAME_MODULE)

APP_ADDR		= -DAPP_START_ADDR=0x08000000
SYS_CONSOLE_BAUDRATE	= 115200
CONSOLE_BAUDRATE_DEF	= -DSYS_CONSOLE_BAUDRATE=$(SYS_CONSOLE_BAUDRATE)

OPTIMIZE_OPTION		= -g -O3
RELEASE_OPTION		= -DRELEASE
IRQ_DEBUG_OPTION	= -UAK_IO_IRQ_ANALYZER
RF24_OPTION		= -URF24_USED_POLLING
LOG_AK_KERNEL_ENABLE	= -UAK_TASK_LOG_ENABLE
WARNING_OPTION		= -Wpointer-sign

GCC_PATH	= /home/covv/workspace/tools/gcc-arm-none-eabi-5_4-2016q3
OPENOCD_PATH	= /home/covv/workspace/tools/openocd-0.11.0-rc1

OBJECTS_DIR = build_$(NAME_MODULE)_stm32f4
TARGET = $(OBJECTS_DIR)/$(NAME_MODULE).axf

#Include sources file
-include sources/ak/Makefile.mk
-include sources/app/Makefile.mk
-include sources/common/Makefile.mk
-include sources/driver/Makefile.mk
-include sources/sys/Makefile.mk
-include sources/rf_protocols/Makefile.mk
#-include sources/networks/Makefile.mk
-include sources/platform/stm32f4/Makefile.mk

SOURCES		+=
SOURCES_	 = $(shell find $(SOURCES) -type f -printf "%f\n")
OBJECTS		+= $(patsubst %.c, $(OBJECTS_DIR)/%.o, $(SOURCES_))

SOURCES_CPP	+=
SOURCES_CPP_	 = $(shell find $(SOURCES_CPP) -type f -printf "%f\n")
OBJECTS		+= $(patsubst %.cpp, $(OBJECTS_DIR)/%.o, $(SOURCES_CPP_))

LIBC		= $(GCC_PATH)/arm-none-eabi/lib/armv7-m/libc.a
LIBM		= $(GCC_PATH)/arm-none-eabi/lib/armv7-m/libm.a
LIBFPU		= $(GCC_PATH)/arm-none-eabi/lib/armv7-m/libg.a
LIBGCC		= $(GCC_PATH)/lib/gcc/arm-none-eabi/5.4.1/armv7-m/libgcc.a

LIB_PATH += -L$(GCC_PATH)/arm-none-eabi/lib/armv7-m
LIB_PATH += -L$(GCC_PATH)/lib/gcc/arm-none-eabi/5.4.1/armv7-m

# The command for calling the compiler.
CC		=	$(GCC_PATH)/bin/arm-none-eabi-gcc
CPP		=	$(GCC_PATH)/bin/arm-none-eabi-g++
AR		=	$(GCC_PATH)/bin/arm-none-eabi-ar
LD		=	$(GCC_PATH)/bin/arm-none-eabi-ld
OBJCOPY		=	$(GCC_PATH)/bin/arm-none-eabi-objcopy
ARM_GDB		=	$(GCC_PATH)/bin/arm-none-eabi-gdb
ARM_SIZE	=	$(GCC_PATH)/bin/arm-none-eabi-size

# Set the compiler CPU/FPU options.
CPU = -mcpu=cortex-m4
FPU = -mfloat-abi=soft

# ASM compiler flags
AFLAGS +=	-mthumb	\
		$(CPU) 	\
		$(FPU)	\
		-MD

# C compiler flags
CFLAGS +=	-mthumb			\
		$(CPU)			\
		$(FPU)			\
		-ffunction-sections	\
		-fdata-sections		\
		-MD			\
		-Wall			\
		-std=c99		\
		-c			\
		$(CONSOLE_BAUDRATE_DEF)	\
		$(IRQ_DEBUG_OPTION)	\
		$(WARNING_OPTION)	\
		$(RF24_OPTION)		\
		$(OPTIMIZE_OPTION)	\
		$(RELEASE_OPTION)	\
		$(LOG_AK_KERNEL_ENABLE)	\
		$(APP_ADDR)		\
		-DUSE_STDPERIPH_DRIVER	\
		-DSTM32F40XX		\
		-DLOGIN_PRINT_EN	\
		-DSYS_PRINT_EN		\
		-DSYS_DBG_EN		\
		-DAPP_DBG_EN		\
		-DAPP_PRINT_EN		\
		-DAPP_DBG_SIG_EN	\

# C++ compiler flags
CPPFLAGS += 	-mthumb			\
		$(CPU)			\
		$(FPU)			\
		-ffunction-sections	\
		-fdata-sections		\
		-fno-rtti		\
		-fno-exceptions		\
		-fno-use-cxa-atexit	\
		-MD			\
		-Wall			\
		-std=c++11		\
		-c			\
		$(CONSOLE_BAUDRATE_DEF)	\
		$(IRQ_DEBUG_OPTION)	\
		$(RF24_OPTION)		\
		$(OPTIMIZE_OPTION)	\
		$(RELEASE_OPTION)	\
		$(LOG_AK_KERNEL_ENABLE)	\
		$(APP_ADDR)		\
		-DUSE_STDPERIPH_DRIVER	\
		-DSTM32F40XX		\
		-DLOGIN_PRINT_EN	\
		-DSYS_PRINT_EN		\
		-USYS_DBG_EN		\
		-UAPP_DBG_EN		\
		-UAPP_PRINT_EN		\
		-UAPP_DBG_SIG_EN	\

# linker flags
LDFLAGS	=	-Map=$(OBJECTS_DIR)/$(PROJECT).map	\
		--gc-sections	\
		$(LIB_PATH)	\
		$(LIBC) $(LIBM) $(LIBGCC) $(LIBFPU)

all: create $(TARGET)

create:
	$(Print) CREATE $(OBJECTS_DIR) folder
	@mkdir -p $(OBJECTS_DIR)

$(TARGET): $(OBJECTS) $(LIBC) $(LIBM) $(LIBGCC) $(LIBFPU)
	$(Print) LD $@
	@$(LD) --entry reset_handler -T $(LDFILE) $(LDFLAGS) -o $(@) $(^)
	$(Print) OBJCOPY $(@:.axf=.bin)
	@$(OBJCOPY) -O binary $(@) $(@:.axf=.bin)
	@$(OBJCOPY) -O binary $(@) $(@:.axf=.out)
	@$(OBJCOPY) -O binary $(@) $(@:.axf=.elf)
	@$(ARM_SIZE) $(TARGET)

$(OBJECTS_DIR)/%.o: %.c
	$(Print) CC $@
	@$(CC) $(CFLAGS) -o $@ $<

$(OBJECTS_DIR)/%.o: %.cpp
	$(Print) CXX $@
	@$(CPP) $(CPPFLAGS) -o $@ $<

$(OBJECTS_DIR)/%.o: %.s
	$(Print) CC $@
	@$(CC) $(AFLAGS) -o $@ -c $<

flash: 	all
	$(Print) BURNING $(TARGET:.axf=.bin) to target
ifeq ($(ak_flash),yes)
	@sudo ak_flash /dev/ac_mod_com $(TARGET:.axf=.bin) 0x08000000
else
	@st-flash write $(TARGET:.axf=.bin) 0x08000000
endif

debug:	all
	xterm -e openocd -f $(OPENOCD_PATH)/tcl/board/stm32ldiscovery.cfg &
ifeq ($(gdb),ddd)
	$(Print) DBG using DDD
	ddd --debugger $(ARM_GDB) --command=stm32f4_init.gdb $(TARGET)
else
	$(Print) DBG GDB
	$(ARM_GDB) --command=stm32f4_init.gdb $(TARGET)
endif

com:
	sudo minicom -D /dev/ac_mod_com -b $(SYS_CONSOLE_BAUDRATE)

clean:
	$(Print) CLEAN $(OBJECTS_DIR) folder
	@rm -rf $(OBJECTS_DIR)
