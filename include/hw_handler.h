#ifndef HW_HANDLER_H
#define HW_HANDLER_H

#include <MotorWheel.h>
#include "debug_printf.h"
#include "main.h"

/**	
 *  @brief Wheel Back encoder handler function.
 *
 *  @param None
 *
 *  @returns None
 */
void encoderWheel_Back_Handler(void);

/**	
 *  @brief Wheel 2 encoder handler function.
 *
 *  @param None
 *
 *  @returns None
 */
void encoderWheel_Right_Handler(void);

/**	
 *  @brief Wheel 3 encoder handler function.
 *
 *  @param None
 *
 *  @returns None
 */
void encoderWheel_Left_Handler(void);

/**	
 *  @brief External interrupt Demo handler function.
 *
 *  @param None
 *
 *  @returns None
 */
extern void extInterruptDemoHandler();

#endif /* HW_HANDLER_H */
