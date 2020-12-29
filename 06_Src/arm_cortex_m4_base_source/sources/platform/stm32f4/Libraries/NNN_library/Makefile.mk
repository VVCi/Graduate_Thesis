
#CFLAGS += -I./sources/platform/stm32f4/Libraries/NNN_library/spi
CFLAGS += -I./sources/platform/stm32f4/Libraries/NNN_library/gpio
#CFLAGS += -I./sources/platform/stm32f4/Libraries/NNN_library/usbhid


#CPPFLAGS += -I./sources/platform/stm32f4/Libraries/NNN_library/spi
CPPFLAGS += -I./sources/platform/stm32f4/Libraries/NNN_library/gpio
#CPPFLAGS += -I./sources/platform/stm32f4/Libraries/NNN_library/usbhid

#VPATH += sources/platform/stm32f4/Libraries/NNN_library/spi
VPATH += sources/platform/stm32f4/Libraries/NNN_library/gpio
#VPATH += sources/platform/stm32f4/Libraries/NNN_library/usbhid

# C source files
#SOURCES += sources/platform/stm32f4/Libraries/NNN_library/spi/tm_stm32f4_spi.c
SOURCES += sources/platform/stm32f4/Libraries/NNN_library/gpio/tm_stm32f4_gpio.c
#SOURCES += sources/platform/stm32f4/Libraries/NNN_library/usbhid/tm_stm32f4_usb_hid_device.c
SOURCES += sources/platform/stm32f4/Libraries/NNN_library/gpio/tm_stm32f4_pwm.c
SOURCES += sources/platform/stm32f4/Libraries/NNN_library/gpio/tm_stm32f4_timer_properties.c

