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
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
/* BMX160 Include Files */
#include <DFRobot_BMX160.h>
#include "MadgwickAHRS.h"
/* LCD I2C PCF8574 Include Files */
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
/* Math Include Files */
#include <math.h>
#include <MatrixMath.h>
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
#define TEST_

#ifndef MICRO_PER_SEC
    #define MICRO_PER_SEC 1000000
#endif

#define BODY_RADIUS                     110 //mm
#define REDUCTION_RATIO_NAMIKI_MOTOR    80
#define WHEEL_RADIUS                    24
#define WHEEL_CIRC                      (WHEEL_RADIUS * 2 * PI)

#define LCD_OPTION                      (0)
#define IMU_OPTION                      (1)


/* TIVA Board */
#define TIVA_RED_LED    PF_1
#define TIVA_BLUE_LED   PF_2  
#define TIVA_GREEN_LED  PF_3
#define TIVA_SW1        PF_4
#define TIVA_SW2        PF_0

/* Nexus Control Breakout BoosterPack Board */
// #define NEXUS_STT_LED   PB_5
#define NEXUS_STT_LED   PF_4
#define NEXUS_SWA       PD_2

/* Motor 1 Pin Defs */
#define M1_PWM          PB_3
#define M1_DIR_A        PD_3
#define M1_DIR_B        PE_1
#define M1_ENCA         PA_2
#define M1_ENCB         PD_7

/* Motor 2 Pin Defs */
#define M2_PWM          PF_3
#define M2_DIR_A        PE_2
#define M2_DIR_B        PE_3
#define M2_ENCA         PD_6
#define M2_ENCB         PC_7

/* Motor 3 Pin Defs */
#define M3_PWM          PF_2
#define M3_DIR_A        PA_3
#define M3_DIR_B        PA_4
#define M3_ENCA         PC_5
#define M3_ENCB         PC_6

/* LCD I2C Defs */
#define LCD_I2C_ADDR    (0x27)
#define LCD_COLS_NUM    (16)
#define LCD_ROWS_NUM    (2)

/**************************************************************************
 *                           Global Declarations
 **************************************************************************/
extern Encoder_Params_t encoderWheel_Back_Params;
extern Encoder_Params_t encoderWheel_Right_Params;
extern Encoder_Params_t encoderWheel_Left_Params;

extern MotorWheel motorWheel_Back;
extern MotorWheel motorWheel_Right;
extern MotorWheel motorWheel_Left;

extern Omni3WD omniNexusBot;

extern ros::NodeHandle h_Node;
extern ros::Subscriber<geometry_msgs::Twist> sub_velTwist;
extern ros::Publisher pub_GyroAccel;
extern ros::Publisher pub_Mag;
extern sensor_msgs::Imu imu_GyroAccel_msg;
extern sensor_msgs::MagneticField imu_Mag_msg;

extern TwoWire Wire2;
extern DFRobot_BMX160 bmx160;
extern sBmx160SensorData_t Omagn, Ogyro, Oaccel;

extern LiquidCrystal_PCF8574 lcd; // set the LCD address to 0x27 for a 16 chars and 2 line display

/**************************************************************************
 *                      User-defined Functions Prototype
 **************************************************************************/
/**	
 *  @brief User defined function to Init Hardware for main program
 */
void hardware_Init();

/**	
 *  @brief User defined function to flash STT LED on main program starting
 */
void sttLED_Flash(void);

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

/**
 * @brief User defined function to map ROS Geometry messages 
 *        linear value to Motor control value in MMPS
 * @details formular: mapped = (linearValue - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
 *          in_min = 0.00
 *          in_max = 0.1
 *          out_min = 0
 *          out_max = 150
 *          (out_max - out_min) / (in_max - in_min) + out_min = 2000
 * 
 * @param linearValue ROS Geometry messages linear value
 * @return Motor control value in MMPS
 */
uint8_t geoLinear2mmps(float linearValue);

/**
 * @brief User defined function to map ROS Geometry messages 
 *        angular value to Motor control value in MMPS
 * 
 * @param angularValue ROS Geometry messages angular value
 * @return Motor control value in MMPS
 */
uint8_t geoAngular2mmps(float angularValue);

/**
 * @brief The user can search for a value in a certain range 
 *        to compare whether the value belongs to the range 
 *        under consideration.
 * 
 * @param min_value     The value is minimum in a range
 * @param max_value     The value is maximum in a range
 * @param comp_value    The value is considered in a range
 * @return true 
 * @return false 
 */
bool positive_inRange(float min_value, float max_value, float comp_value);

/**
 * @brief The user can search for a value in a certain range 
 *        to compare whether the value belongs to the range 
 *        under consideration.
 * 
 * @param min_value     The value is minimum in a range
 * @param max_value     The value is maximum in a range
 * @param comp_value    The value is considered in a range
 * @return true 
 * @return false 
 */
bool negative_inRange(float min_value, float max_value, float comp_value);

/**
 * @brief Implement speed control advance or back-off 
 *        from Jetson Nano's command
 * 
 * @param cmd_twistSpeed Twist value from keyboard.
 */
void cmdTwistSpeedCallback(const geometry_msgs::Twist& cmd_twistSpeed);

#endif /* MAIN_H */