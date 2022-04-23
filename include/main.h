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
#include <std_msgs/Empty.h>
/* BMX160 Include Files */
#include <DFRobot_BMX160.h>
// #include "MadgwickAHRS.h"
/* LCD I2C PCF8574 Include Files */
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
/* Demo Include Files */
#include "demo.h"

/**************************************************************************
 *                           Macros definition
 **************************************************************************/
/* Timer for scheduling periodic PID calculation */
#define IMU_TIMER_BASE              TIMER2_BASE
#define IMU_TIMER_SYSCTL_PERIPH     SYSCTL_PERIPH_TIMER2
/* Timer for scheduling periodic DEBUG print calculation */
#define DEBUG_TIMER_BASE            TIMER4_BASE
#define DEBUG_TIMER_SYSCTL_PERIPH   SYSCTL_PERIPH_TIMER4
/* Timer for scheduling periodic PID calculation */
#define PID_TIMER_BASE              TIMER5_BASE
#define PID_TIMER_SYSCTL_PERIPH     SYSCTL_PERIPH_TIMER5

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
#define M1_DIR_A        PE_2
#define M1_DIR_B        PE_3
#define M1_ENCA         PC_5
#define M1_ENCB         PC_6

// Motor 2
#define M2_PWM          PF_3
#define M2_DIR_A        PD_3
#define M2_DIR_B        PE_1
#define M2_ENCA         PC_7
#define M2_ENCB         PD_6

// Motor 3
#define M3_PWM          PB_3
#define M3_DIR_A        PB_7
#define M3_DIR_B        PD_2
#define M3_ENCA         PD_7
#define M3_ENCB         PA_2

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
 *  @brief User defined function to Attach handler Timer interrupt
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

#endif /* MAIN_H */