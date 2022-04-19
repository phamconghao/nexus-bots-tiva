#ifndef HW_HANDLER_H
#define HW_HANDLER_H

#include <MotorWheel.h>
#include "debug_printf.h"
#include "main.h"

/**	
 *  @brief Wheel 1 encoder handler function.
 *
 *  @param None
 *
 *  @returns None
 */
void encoderWheel_1_Handler(void);

/**	
 *  @brief Wheel 2 encoder handler function.
 *
 *  @param None
 *
 *  @returns None
 */
void encoderWheel_2_Handler(void);

/**	
 *  @brief Wheel 3 encoder handler function.
 *
 *  @param None
 *
 *  @returns None
 */
void encoderWheel_3_Handler(void);

/**	
 *  @brief External interrupt Demo handler function.
 *
 *  @param None
 *
 *  @returns None
 */
extern void extInterruptDemoHandler();

#endif /* HW_HANDLER_H */
