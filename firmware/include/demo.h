#ifndef DEMO_H
#define DEMO_H

#include "main.h"

// #define DEMO

#ifdef DEMO
extern std_msgs::String str_msg;
extern ros::Publisher chatter;
extern ros::Subscriber<std_msgs::Empty> suber;
extern char hello_msg[];

/* LED Blink Demo Functions */
void ledBlink_demo(void);
/* LED Blink with PWM Demo Functions */
void ledBlinkPWM_demo(void);
/* External IO Interrupt Demo Functions */
void externalInterrupt_demo(void);
/* Serial Debug Printf Demo Functions */
void dbgPrintf_demo(void);
/* PID Motor Control Demo Functions */
#define DEMO_MOTOR_WHEEL      motorWheel_Back
void pidMotorControl_demo(void);
/* Timer Interrupt Demo Functions (Timer 5) */
void tmrInterrupt_demo(void);
void TimerInterruptDemo_Handler(void);
/* ROS communication Demo Functions */
void ros_PubSub_demo(void);
void message_Callback(const std_msgs::Empty& toggle_msg);
/* BMX160 Demo Functions */
void bmx160_demo(void);
/* LCD 16x2 I2C PCF8574 interface Demo Functions */
void lcd16x2_I2CLCD_demo(void);
/* Nexus Control Demo Functions */
void nexusControl_demo(void);

#endif

#endif /* DEMO_H */