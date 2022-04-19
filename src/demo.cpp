#include "demo.h"

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello_msg[] = "hello world!";

/**************************************************************************
 *                     ROS chatter communication Demo
 **************************************************************************/
void ros_chatter_demo()
{
    DEBUG_PRINTF("Start ROS chatter communication Demo\n");

    h_Node.initNode();
    h_Node.advertise(chatter);

    /* Main loop */
    for (;;)
    {
        str_msg.data = hello_msg;
        chatter.publish(&str_msg);

        h_Node.spinOnce();
        delay(1000);
    }
}

/**************************************************************************
 *                     PID Motor Control Demo
 **************************************************************************/
void PID_TimerInterrupt_Handler()
{
    MAP_TimerIntClear(PID_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    motorWheel_Back.PIDRegulate();
}

void DEBUGGER_TimerInterrupt_Handler()
{
    MAP_TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    DEBUG_PRINTF("EncoderWheel_1_");
    DEBUG_PRINTF("\tcurDir -> %d", encoderWheel_1_Params.currDirection);
    DEBUG_PRINTF("\tPID Out -> %d", motorWheel_Back.getSpeedRPMOutput());
    DEBUG_PRINTF("\tSpeedMMPS -> %d\n", motorWheel_Back.getSpeedMMPS());
}

void pidMotorControl_demo()
{
    DEBUG_PRINTF("Start PID Motor Control Demo\n");
    
    /* PID Regulate periodic with SAMPLETIME = 2ms or 500Hz freq */
    attachTimerInterrupt(PID_TIMER_BASE, PID_TIMER_SYSCTL_PERIPH, &PID_TimerInterrupt_Handler, 500);

    attachTimerInterrupt(TIMER4_BASE, SYSCTL_PERIPH_TIMER4, &DEBUGGER_TimerInterrupt_Handler, 1);

    motorWheel_Back.setupInterrupt();
    motorWheel_Back.PIDEnable(KC, TAUI, TAUD, SAMPLETIME);

    /* Main loop */
    for (;;)
    {
        DEBUG_PRINTF("Motor run ADVANCE MMPS: 100 in 3sec\n");
        motorWheel_Back.setSpeedMMPS(100, DIR_ADVANCE);
        delay(3000);
        DEBUG_PRINTF("Motor run ADVANCE MMPS: 200 in 3sec\n");
        motorWheel_Back.setSpeedMMPS(200, DIR_ADVANCE);
        delay(3000);
        DEBUG_PRINTF("Motor run BACKOFF MMPS: 200 in 3sec\n");
        motorWheel_Back.setSpeedMMPS(200, DIR_BACKOFF);
        delay(3000);
        DEBUG_PRINTF("Motor run BACKOFF MMPS: 50 in 3sec\n");
        motorWheel_Back.setSpeedMMPS(50, DIR_BACKOFF);
        delay(2000); 
        DEBUG_PRINTF("Motor Stop in 3sec\n");
        motorWheel_Back.setSpeedMMPS(0, DIR_ADVANCE);
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
