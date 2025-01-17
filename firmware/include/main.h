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
#include "hw_handler.h"
#include "MotorWheel.h"
#include "Omni3WD.h"
#include "holonomic.h"
// #include "holonomic_3wheel.h"
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
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
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

#define BODY_RADIUS                     150 // mm
#define REDUCTION_RATIO_NAMIKI_MOTOR    80
#define WHEEL_RADIUS                    50  // mm
#define WHEEL_CIRC                      (WHEEL_RADIUS * 2 * PI)

#define LCD_OPTION                      (0)
#define IMU_OPTION                      (0)
#define CAL_ODOM                        (1)


/* TIVA Board */
#define TIVA_RED_LED    PF_1
#define TIVA_BLUE_LED   PF_2
#define TIVA_GREEN_LED  PF_3
#define TIVA_SW1        PF_4
#define TIVA_SW2        PF_0

/* Nexus Control Breakout BoosterPack Board */
#define NEXUS_STT_LED   PB_5
#define NEXUS_SWA       PD_2

/* Motor 1 Pin Defs */
#define M1_PWM          PB_3
#define M1_DIR_A        PD_3
#define M1_DIR_B        PE_1
#define M1_ENCA         PD_7
#define M1_ENCB         PA_2

/* Motor 2 Pin Defs */
#define M2_PWM          PF_3
#define M2_DIR_A        PE_2
#define M2_DIR_B        PE_3
#define M2_ENCA         PC_7
#define M2_ENCB         PD_6

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
extern ros::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_initPose;
// extern ros::Publisher pub_GyroAccel;
// extern ros::Publisher pub_Mag;
extern ros::Publisher pub_Odom;
// extern sensor_msgs::Imu imu_GyroAccel_msg;
// extern sensor_msgs::MagneticField imu_Mag_msg;
extern nav_msgs::Odometry odom_msg;
extern nav_msgs::Odometry odom_initpose;

extern TwoWire Wire2;
extern DFRobot_BMX160 bmx160;
extern sBmx160SensorData_t Omagn, Ogyro, Oaccel;

extern LiquidCrystal_PCF8574 lcd; // set the LCD address to 0x27 for a 16 chars and 2 line display

extern float matrix_speed_wheels_encoder[];
extern int i, j;
extern float matrix_w_b[3][3];
extern float matrix_b_w[3][3];
extern float robot_heading_inertial;
extern float robot_speed_inertial_frame[]; 
extern float robot_speed_body_frame[];
extern float omega;
extern float robot_position_inertial[];
extern bool initialPoseReceived;

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
 * @brief Implement speed control advance or back-off 
 *        from Jetson Nano's command
 * 
 * @param cmd_twistSpeed Twist value from keyboard.
 */
void cmdTwistSpeedCallback(const geometry_msgs::Twist& cmd_twistSpeed);

/**
 * @brief 
 * 
 * @param init_pose 
 */
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& init_pose);

#endif /* MAIN_H */