CFLAGS		+= -I./sources/common
CPPFLAGS	+= -I./sources/common

VPATH += sources/common

SOURCES_CPP += sources/common/view_item.cpp
SOURCES_CPP += sources/common/view_render.cpp
SOURCES_CPP += sources/common/screen_manager.cpp

SOURCES += sources/common/utils.c
SOURCES += sources/common/xprintf.c
SOURCES += sources/common/cmd_line.c
SOURCES += sources/common/log_queue.c
SOURCES += sources/common/fifo.c
SOURCES += sources/common/ring_buffer.c
