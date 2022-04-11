#ifndef HW_HANDLER_H
#define HW_HANDLER_H

#include <MotorWheel.h>

/**	
 *  @brief Wheel 1 encoder handler function.
 *
 *  @param None
 *
 *  @returns 	None
 */
void encoderWheel_1_Handler();

/**	
 *  @brief Wheel 2 encoder handler function.
 *
 *  @param None
 *
 *  @returns 	None
 */
void encoderWheel_2_Handler();

/**	
 *  @brief Wheel 3 encoder handler function.
 *
 *  @param None
 *
 *  @returns 	None
 */
void encoderWheel_3_Handler();

extern Encoder_Params_t encoderWheel_1_Params;

#endif /* HW_HANDLER_H */
