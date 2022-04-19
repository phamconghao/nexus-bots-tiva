#ifndef MAIN_H
#define MAIN_H

/**************************************************************************
 *                            Include Files
 **************************************************************************/
/* Energia framework Include Files. */
#include "Energia.h"
/* Debug print Include Files */
#include "debug_printf.h"
/* Control Include Files: */
#include <hw_handler.h>
#include <MotorWheel.h>
#include <Omni3WD.h>
// Tiva C driver Include Files
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
/* ROS Include Files */
#include <ros.h>
#include <std_msgs/String.h>
/* Demo Include Files */
#include "demo.h"

/**************************************************************************
 *                           Macros definition
 **************************************************************************/
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

// Motor 2
#define M2_PWM          PF_2
#define M2_DIR_A        PA_4
#define M2_DIR_B        PA_3
#define M2_ENCA         PC_5
#define M2_ENCB         PC_6

// Motor 3
#define M3_PWM          PF_2
#define M3_DIR_A        PA_4
#define M3_DIR_B        PA_3
#define M3_ENCA         PC_5
#define M3_ENCB         PC_6

/**************************************************************************
 *                           Global Declarations
 **************************************************************************/
extern Encoder_Params_t encoderWheel_Back_Params;
extern Encoder_Params_t encoderWheel_Right_Params;
extern Encoder_Params_t encoderWheel_Left_Params;

extern MotorWheel motorWheel_Back;
extern MotorWheel motorWheel_Right;
extern MotorWheel motorWheel_Left;

extern ros::NodeHandle h_Node;

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

#endif