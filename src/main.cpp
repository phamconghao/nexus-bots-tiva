/**
 *   @file  main.cpp
 *
 *   @brief
 *     MSS main implementation of the Vital Signs Demo
 */

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
// ---------------------------
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

/**************************************************************************
 *                           Local Declarations
 **************************************************************************/

/**************************************************************************
 *                           Global Declarations
 **************************************************************************/

/**
 *  Global Variable for Motor 1 Encoder parameters required by the Demo
 */
Encoder_Params_t encoderWheel_1_Params = {encoderWheel_1_Handler};
MotorWheel wheel1(M1_PWM, M1_DIR_A, M1_DIR_B, M1_ENCA, M1_ENCB, 
                &encoderWheel_1_Params,
                REDUCTION_RATIO_NAMIKI_MOTOR, WHEEL_CIRC);

/**************************************************************************
 *                      Nexus Bot Demo MAIN Functions
 **************************************************************************/
void main_program()
{
#ifdef DEBUG
    debug_init();
#endif

    // ledBlink_demo();
    // ledBlinkPWM_demo();
    // externalInterrupt_demo();
    // dbgPrintf_demo();
    pidMotorControl_demo();
    // tmrInterrupt_demo();
}

void attachTimerInterrupt(uint32_t ui32Base, uint32_t ui32Peripheral, void (*p_TmrHandler)(), unsigned int tmrFreq)
{
    // Timer Period calculation
    unsigned long ulPeriod = SysCtlClockGet() / tmrFreq;
    // Timer interrupt Configuration
    MAP_SysCtlPeripheralEnable(ui32Peripheral);
    MAP_TimerConfigure(ui32Base, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(ui32Base, TIMER_A, ulPeriod - 1);
    TimerIntRegister(ui32Base, TIMER_A, p_TmrHandler);
    MAP_TimerIntEnable(ui32Base, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(ui32Base, TIMER_A);
}

/**************************************************************************
 *                     PID Motor Control Demo
 **************************************************************************/
void PID_TimerInterrupt_Handler()
{
    MAP_TimerIntClear(PID_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    wheel1.PIDRegulate();
}

// void DEBUGGER_TimerInterrupt_Handler()
// {
//     MAP_TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
//     DEBUG_PRINTF("EncoderWheel_1_");
//     DEBUG_PRINTF("\tcurrDirection -> %d", encoderWheel_1_Params.currDirection);
//     DEBUG_PRINTF("\tspeedPPS -> %d", encoderWheel_1_Params.speedPPS);
//     DEBUG_PRINTF("\t\tspeedMMPS -> %d\n", wheel1.getSpeedMMPS());
// }

void pidMotorControl_demo()
{
    DEBUG_PRINTF("Start PID Motor Control Demo\n");
    
    /* PID Regulate periodic with SAMPLETIME = 5ms or 200Hz freq */
    attachTimerInterrupt(PID_TIMER_BASE, PID_TIMER_SYSCTL_PERIPH, &PID_TimerInterrupt_Handler, 200);

    // attachTimerInterrupt(TIMER4_BASE, SYSCTL_PERIPH_TIMER4, &DEBUGGER_TimerInterrupt_Handler, 5);

    wheel1.setupInterrupt();
    wheel1.PIDEnable(KC, TAUI, TAUD, SAMPLETIME);

    /* Main loop */
    for (;;)
    {
        DEBUG_PRINTF("Motor run ADVANCE MMPS: 100 in 3sec\n");
        wheel1.setSpeedMMPS(100, DIR_ADVANCE);
        delay(3000);
        DEBUG_PRINTF("Motor run ADVANCE MMPS: 200 in 3sec\n");
        wheel1.setSpeedMMPS(200, DIR_ADVANCE);
        delay(3000);
        DEBUG_PRINTF("Motor run BACKOFF MMPS: 200 in 3sec\n");
        wheel1.setSpeedMMPS(200, DIR_BACKOFF);
        delay(3000);
        DEBUG_PRINTF("Motor run BACKOFF MMPS: 50 in 3sec\n");
        wheel1.setSpeedMMPS(50, DIR_BACKOFF);
        delay(2000); 
        DEBUG_PRINTF("Motor Stop in 3sec\n");
        wheel1.setSpeedMMPS(0, DIR_ADVANCE);
        delay(3000);
    }
}

/**************************************************************************
 *                Timer Interrupt Demo (Timer 5)
 **************************************************************************/
void TimerInterruptDemo_Handler()
{
    MAP_TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    digitalWrite(TIVA_BLUE_LED, !(digitalRead(TIVA_BLUE_LED)));
    DEBUG_PRINTF("Timer Interrupt Occured!");
    DEBUG_PRINTF("\tLED State %d\n", digitalRead(TIVA_BLUE_LED));
}

void tmrInterrupt_demo(void)
{
    DEBUG_PRINTF("Start Timer Interrupt Demo\n");

    pinMode(TIVA_BLUE_LED, OUTPUT);

    attachTimerInterrupt(TIMER5_BASE, SYSCTL_PERIPH_TIMER5, &TimerInterruptDemo_Handler, 1);

    /* Main loop */
    for (;;)
    {
        ;
    }
}

/**************************************************************************
 *                     Serial Debug Printf Demo
 **************************************************************************/
void dbgPrintf_demo()
{
    /* Init */
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

/**************************************************************************
 *                   External IO Interrupt Demo
 **************************************************************************/
void externalInterrupt_demo()
{
    /* Init */
    DEBUG_PRINTF("Start external Interrupt Demo");

    pinMode(TIVA_SW1, INPUT_PULLUP);
    attachInterrupt(TIVA_SW1, extInterruptDemoHandler, RISING);

    /* Main loop */
    for (;;) 
    {
        ;
    }
}

/**************************************************************************
 *                     LED Blink Demo
 **************************************************************************/
void ledBlink_demo()
{
    DEBUG_PRINTF("Start LED Blink counter Demo");

    uint16_t blinkCounter = 0;

    pinMode(M1_DIR_A, OUTPUT);

    for (;;) 
    {
        DEBUG_PRINTF("LED Blink %d", blinkCounter ++);
        digitalWrite(M1_DIR_A, LOW);
        delay(500);
        digitalWrite(M1_DIR_A, HIGH);
        delay(500);
    }
}

/**************************************************************************
 *                    LED Blink with PWM Demo
 **************************************************************************/
void ledBlinkPWM_demo()
{
    /* Init */
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
