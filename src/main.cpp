/**
 *   @file  main.cpp
 *
 *   @brief
 *     MSS main implementation of the Vital Signs Demo
 */

#define DEBUG
/**************************************************************************
 *                            Include Files
 **************************************************************************/

/* Standard Include Files. */
#include <Arduino.h>
/* Demo Include Files */
#include "main.h"
#include "debug_printf.h"
/* Control Include Files: */
#include <hw_handler.h>
#include <MotorWheel.h>

/**************************************************************************
 *                           Local Declarations
 **************************************************************************/

/**************************************************************************
 *                           Global Declarations
 **************************************************************************/
/**
 *  Global Variable for Motor 1 Encoder parameters required by the Demo
 */
Encoder_Params_t encoderWheel_1_Params = {.Encoder_Handler = 
                                            encoderWheel_1_Handler};
MotorWheel wheel1(M1_PWM, M1_DIR, M1_ENCA, M1_ENCB, 
                &encoderWheel_1_Params, REDUCTION_RATIO_NAMIKI_MOTOR, WHEEL_CIRC);

/**************************************************************************
 *                         Nexus Bot Demo Functions
 **************************************************************************/
void main_program()
{
    // ledBlink_demo();
    // ledBlinkPWM_demo();
    // externalInterrupt_demo();
    dbgPrintf_demo();
}

void dbgPrintf_demo()
{
    /* Init */
#ifdef DEBUG
    debug_init();
#endif
    DEBUG_PRINTF("Start Debug Printf Demo");

    float flValue = 1.5;
    long longValue = 146;

    DEBUG_PRINTF("Float value %f", flValue);
    DEBUG_PRINTF("Long value %ld", longValue);

    /* Main loop */
    for (;;) 
    {
        ;
    }
}

void externalInterrupt_demo()
{
    /* Init */
#ifdef DEBUG
    debug_init();
#endif
    DEBUG_PRINTF("Start external Interrupt Demo");

    pinMode(TIVA_SW1, INPUT_PULLUP);
    attachInterrupt(TIVA_SW1, extInterruptDemoHandler, RISING);

    /* Main loop */
    for (;;) 
    {
        ;
    }
}

void ledBlink_demo()
{
#ifdef DEBUG
    debug_init();
#endif
    DEBUG_PRINTF("Start LED Blink counter Demo");

    uint16_t blinkCounter = 0;

    pinMode(M1_DIR, OUTPUT);

    for (;;) 
    {
        DEBUG_PRINTF("LED Blink %d", blinkCounter ++);
        digitalWrite(M1_DIR, LOW);
        delay(500);
        digitalWrite(M1_DIR, HIGH);
        delay(500);
    }
}

void ledBlinkPWM_demo()
{
    /* Init */
#ifdef DEBUG
    debug_init();
#endif
    DEBUG_PRINTF("Start LED Blink with PWM Demo");

    uint16_t pwmIntensity = 0;
    uint8_t incrFlag = 0;

    pinMode(M1_PWM, OUTPUT);

    /* Main loop */
    for (;;) 
    {
        analogWrite(M1_PWM, pwmIntensity);
        delay(20);

        if (pwmIntensity >= MAX_PWM)
        {
            incrFlag = 0;
            DEBUG_PRINTF("Decreasing PWM");
        }
        if (pwmIntensity == 0)
        {
            incrFlag = 1;
            DEBUG_PRINTF("Increasing PWM");
        }
         
        if (incrFlag)
        {
            pwmIntensity ++;
        }
        else
        {
            pwmIntensity --;
        }

    }
}
