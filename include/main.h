#ifndef MAIN_H
#define MAIN_H

#ifndef MICRO_PER_SEC
    #define MICRO_PER_SEC 1000000
#endif

#define BODY_RADIUS                     110 //mm
#define REDUCTION_RATIO_NAMIKI_MOTOR    80
#define WHEEL_RADIUS                    24
#define WHEEL_CIRC                      (WHEEL_RADIUS * 2 * PI)

/* TIVA Board */
#define TIVA_RED_LED    PF_1
#define TIVA_BLUE_LED   PF_2  
#define TIVA_GREEN_LED  PF_3
#define TIVA_SW1        PF_4
#define TIVA_SW2        PF_0

// Motor 1
#define M1_PWM          PF_2
#define M1_DIR_A        PA_4
#define M1_DIR_B        PA_3
#define M1_ENCA         PC_5
#define M1_ENCB         PC_6

/**************************************************************************
 *                         Functions Prototype
 **************************************************************************/
/**	
 *  @brief Attach Function to Timer interrupt
 *
 *  @details ...details
 *
 *  @param ui32Base        Hardware base address
 *  @param ui32Peripheral  Hardware Peripheral ID
 *  @param p_TmrHander     Interrupt Hander function
 *  @param tmrFreq         Interrupt Frequency in Hz
 *
 *  @returns None
 */
void attachTimerInterrupt(uint32_t ui32Base, uint32_t ui32Peripheral, void (*p_TmrHandler)(void), unsigned int tmrFreq);

/* LED Blink Demo Functions */
void ledBlink_demo();
/* LED Blink with PWM Demo Functions */
void ledBlinkPWM_demo();
/* External IO Interrupt Demo Functions */
void externalInterrupt_demo();
/* Serial Debug Printf Demo Functions */
void dbgPrintf_demo();

/* PID Motor Control Demo Functions */
void pidMotorControl_demo();
void PIDTimerInterrupt_Handler();

/* Timer Interrupt Demo Functions (Timer 5) */
void tmrInterrupt_demo(void);
void TimerInterruptDemo_Handler(uint32_t ui32Base);

#endif