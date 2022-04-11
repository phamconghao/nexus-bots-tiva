/**
 *   @file  main.cpp
 *
 *   @brief
 *     MSS main implementation of the Vital Signs Demo
 */

#define DEBUG_BUILD
/**************************************************************************
 *                            Include Files
 **************************************************************************/

/* Standard Include Files. */
#include <Arduino.h>

/* Control Include Files: */
#include <hw_handler.h>
#include <MotorWheel.h>
/* Demo Include Files */
#include "main.h"
#include "debug_printf.h"

/**************************************************************************
 *                           Local Definitions
 **************************************************************************/

/**************************************************************************
 *                           Global Definitions
 **************************************************************************/

/**
 *  Global Variable for Motor 1 Encoder parameters required by the Demo
 */
Encoder_Params_t encoderWheel_1_Params;
MotorWheel wheel1(M1_PWM, M1_DIR, M1_ENCA, M1_ENCB, &encoderWheel_1_Params, 
                    REDUCTION_RATIO_NAMIKI_MOTOR, WHEEL_CIRC);

/**************************************************************************
 *                         Nexus Bot Demo Functions
 **************************************************************************/
void main_program()
{
#ifdef  DEBUG_BUILD
    debug_init();
#endif

    pinMode(PF_3, OUTPUT);

    for (;;) 
    {
        DEBUG("LED Blink");
        digitalWrite(PF_3, LOW);
        delay(1000);
        digitalWrite(PF_3, HIGH);
        delay(1000);
    }
}