#ifndef DEBUG_PRINTF_H
#define DEBUG_PRINTF_H

#include <Arduino.h>
#include <stdio.h>
#include <stdarg.h>

#define DEBUG

#define DEBUG_SERIAL    Serial
#define DBG_MSG_SIZE    64

/**	
 *  @brief Initialize debug printf function.
 *
 *  @param None
 *
 *  @returns None
 */
void debug_init();

/**	
 *  @brief Debug printf function.
 *
 *  @param format Standard printf string format, maximum length is DBG_MSG_SIZE
 *
 *  @returns None
 */
void debug_printf(const char *format, ...);


#ifdef DEBUG
    #define DEBUG_PRINTF(...) debug_printf(__VA_ARGS__)
    // #define DEBUG_PRINTF(...) Serial.print(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...) do {} while (0)
#endif

#endif