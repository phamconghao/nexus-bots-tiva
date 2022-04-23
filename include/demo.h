#ifndef DEMO_H
#define DEMO_H

#include "main.h"

extern std_msgs::String str_msg;
extern ros::Publisher chatter;
extern ros::Subscriber<std_msgs::Empty> suber;
extern char hello_msg[];

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
void PID_TimerInterrupt_Handler();
/* Timer Interrupt Demo Functions (Timer 5) */
void tmrInterrupt_demo(void);
void TimerInterruptDemo_Handler();
/* ROS communication Demo Functions */
void ros_PubSub_demo();
void message_Callback(const std_msgs::Empty& toggle_msg);
/* BMX160 Demo Functions */
void bmx160_demo(void);
void IMU_TimerInterrupt_Handler();
/* LCD 16x2 I2C PCF8574 interface Demo Functions */
void lcd16x2_I2CLCD_demo();

#endif /* DEMO_H */