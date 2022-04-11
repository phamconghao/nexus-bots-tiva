#ifndef DEBUG_PRINTF_H
#define DEBUG_PRINTF_H

#include <Arduino.h>
#include <stdio.h>
#include <stdarg.h>

#define DBG_MSG_SIZE    64

void debug_init();
void debug_printf(const char *format, ...);

#ifdef DEBUG_BUILD
#  define DEBUG(x) debug_printf(x)
#else
#  define DEBUG(x) do {} while (0)
#endif

#endif