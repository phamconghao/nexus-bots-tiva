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

/**	
 *  @brief PID Timer interrupt handler function.
 *
 *  @param None
 *
 *  @returns None
 */
extern void PID_TimerInterrupt_Handler();

/**	
 *  @brief IMU Timer interrupt handler function.
 *
 *  @param None
 *
 *  @returns None
 */
extern void IMU_TimerInterrupt_Handler();

#ifdef DEBUG_PID
/**	
 *  @brief Debugger Timer interrupt handler function.
 *
 *  @param None
 *
 *  @returns None
 */
extern void DEBUGGER_TimerInterrupt_Handler();
#endif

#endif /* HW_HANDLER_H */
