#ifndef __APP_DBG_H__
#define __APP_DBG_H__

#if defined(STM32L_PLATFORM)
#include "../common/xprintf.h"

#if defined(APP_DBG_EN)
#define APP_DBG(fmt, ...)       xprintf("[DBG] " fmt, ##__VA_ARGS__)
#else
#define APP_DBG(fmt, ...)
#endif

#if defined(APP_PRINT_EN)
#define APP_PRINT(fmt, ...)       xprintf("[PRINT] " fmt, ##__VA_ARGS__)
#else
#define APP_PRINT(fmt, ...)
#endif

#if defined(LOGIN_PRINT_EN)
#define LOGIN_PRINT(fmt, ...)       xdymcprintf(fmt, ##__VA_ARGS__)
#else
#define LOGIN_PRINT(fmt, ...)
#endif

#if defined(APP_DBG_SIG_EN)
#define APP_DBG_SIG(fmt, ...)       xprintf("-SIG-> " fmt, ##__VA_ARGS__)
#else
#define APP_DBG_SIG(fmt, ...)
#endif
#endif

#if defined(STM32F40XX)
#include "../common/xprintf.h"

#if defined(APP_DBG_EN)
#define APP_DBG(fmt, ...)       xprintf((const char*)fmt, ##__VA_ARGS__)
#else
#define APP_DBG(fmt, ...)
#endif

#if defined(APP_PRINT_EN)
#define APP_PRINT(fmt, ...)       xprintf((const char*)fmt, ##__VA_ARGS__)
#else
#define APP_PRINT(fmt, ...)
#endif

#if defined(LOGIN_PRINT_EN)
#define LOGIN_PRINT(fmt, ...)       xdymcprintf((const char*)fmt, ##__VA_ARGS__)
#else
#define LOGIN_PRINT(fmt, ...)
#endif
#endif

#endif //__APP_DBG_H__
