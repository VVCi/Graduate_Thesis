CFLAGS += -I./sources/platform/stm32f4/Libraries/STM32_USB_Device_Library/inc
CPPFLAGS += -I./sources/platform/stm32f4/Libraries/STM32_USB_Device_Library/inc

VPATH += sources/platform/stm32f4/Libraries/STM32_USB_Device_Library/src

# C source files

SOURCES += sources/platform/stm32f4/Libraries/STM32_USB_Device_Library/src/usb_bsp.c
SOURCES += sources/platform/stm32f4/Libraries/STM32_USB_Device_Library/src/usb_core.c
SOURCES += sources/platform/stm32f4/Libraries/STM32_USB_Device_Library/src/usb_dcd.c
SOURCES += sources/platform/stm32f4/Libraries/STM32_USB_Device_Library/src/usb_dcd_int.c
SOURCES += sources/platform/stm32f4/Libraries/STM32_USB_Device_Library/src/usbd_core.c
SOURCES += sources/platform/stm32f4/Libraries/STM32_USB_Device_Library/src/usbd_desc.c
SOURCES += sources/platform/stm32f4/Libraries/STM32_USB_Device_Library/src/usbd_hid_core.c
SOURCES += sources/platform/stm32f4/Libraries/STM32_USB_Device_Library/src/usbd_ioreq.c
SOURCES += sources/platform/stm32f4/Libraries/STM32_USB_Device_Library/src/usbd_req.c
SOURCES += sources/platform/stm32f4/Libraries/STM32_USB_Device_Library/src/usbd_usr.c
