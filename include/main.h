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
void ledBlink_demo();
void ledBlinkPWM_demo();
void externalInterrupt_demo();
void dbgPrintf_demo();
void pidMotorControl_demo();

void Timer5InterruptDemo_Handler(void);
void Timer5Interrupt_init(void (*p_TmrHandler)(void));
void tmrInterrupt_demo(void);

#endif