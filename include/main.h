#ifndef MAIN_H
#define MAIN_H

#ifndef MICRO_PER_SEC
    #define MICRO_PER_SEC 1000000
#endif

#define BODY_RADIUS                     110 //mm
#define REDUCTION_RATIO_NAMIKI_MOTOR    80
#define WHEEL_RADIUS                    24
#define WHEEL_CIRC                      (WHEEL_RADIUS * 2 * PI)

#define TIVA_SW1    PF_4
#define TIVA_SW2    PF_0

// Motor 1
#define M1_PWM      PF_2
#define M1_DIR      PA_4
#define M1_ENCA     PC_5
#define M1_ENCB     PC_6

/**************************************************************************
 *                         Functions Prototype
 **************************************************************************/
void ledBlink_demo();
void ledBlinkPWM_demo();
void externalInterrupt_demo();

#endif