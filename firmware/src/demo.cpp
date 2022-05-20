#include "demo.h"

#ifdef DEMO

/**************************************************************************
 *                  LCD 16x2 I2C PCF8574 interface Demo
 **************************************************************************/
// LiquidCrystal_PCF8574 lcd(LCD_I2C_ADDR); // set the LCD address to 0x27 for a 16 chars and 2 line display
uint8_t show = -1;

void lcd16x2_I2CLCD_demo()
{
    DEBUG_PRINTF("LCD 16x2 I2C PCF8574 interface Demo\n");

    Wire.begin();
    Wire.beginTransmission(LCD_I2C_ADDR);

    if (Wire.endTransmission())
    {
        DEBUG_PRINTF("  LCD not found.\n");
    }
    else
    {
        DEBUG_PRINTF("  LCD found.\n");
        show = 0;
        lcd.begin(16, 2); /* Initialize the LCD */
    }

    /* Main loop */
    for (;;)
    {
        if (show == 0)
        {
            lcd.setBacklight(255);
            lcd.home();
            lcd.clear();
            DEBUG_PRINTF("Hello LCD\n");
            lcd.print("Hello LCD");
            delay(1000);

            lcd.setBacklight(0);
            delay(400);
            lcd.setBacklight(255);
        }
        else if (show == 1)
        {
            lcd.clear();
            lcd.print("Cursor On");
            lcd.cursor();
        }
        else if (show == 2)
        {
            lcd.clear();
            lcd.print("Cursor Blink");
            lcd.blink();
        }
        else if (show == 3)
        {
            lcd.clear();
            lcd.print("Cursor OFF");
            lcd.noBlink();
            lcd.noCursor();
        }
        else if (show == 4)
        {
            lcd.clear();
            lcd.print("Display Off");
            lcd.noDisplay();
        }
        else if (show == 5)
        {
            lcd.clear();
            lcd.print("Display On");
            lcd.display();
        }
        else if (show == 7)
        {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("*** first line.");
            lcd.setCursor(0, 1);
            lcd.print("*** second line.");
        }
        else if (show == 8)
        {
            lcd.scrollDisplayLeft();
        }
        else if (show == 9)
        {
            lcd.scrollDisplayLeft();
        }
        else if (show == 10)
        {
            lcd.scrollDisplayLeft();
        }
        else if (show == 11)
        {
            lcd.scrollDisplayRight();
        }
        else if (show == 12)
        {
            lcd.clear();
            lcd.print("write-");
        }
        else if (show > 12)
        {
            lcd.print(show - 13);
        } // if

        delay(1400);
        show = (show + 1) % 16;
    }
}
/**************************************************************************
 *                    BMX160 IMU Demo
 **************************************************************************/
// TwoWire Wire2;
// DFRobot_BMX160 bmx160(&Wire2);
// sBmx160SensorData_t Omagn, Ogyro, Oaccel;

void bmx160_demo()
{
    DEBUG_PRINTF("Start BMX160 IMU Demo\n");
    Serial.begin(115200);
    Serial.print("Start BMX160 IMU Demo\n");
    /* Init the hardware bmx160 */
    if (bmx160.begin() != true)
    {
        DEBUG_PRINTF("Initialization faild, please check the I2C connect!\n");
        while(1);
    }

    attachTimerInterrupt(IMU_TIMER_BASE, IMU_TIMER_SYSCTL_PERIPH, &IMU_TimerInterrupt_Handler, 100);

    /* Main loop */
    for (;;)
    {
        /* Display the magnetometer results (magn is magnetometer in uTesla) */
        Serial.print("M ");
        Serial.print("X: "); Serial.print(Omagn.x); Serial.print("  ");
        Serial.print("Y: "); Serial.print(Omagn.y); Serial.print("  ");
        Serial.print("Z: "); Serial.print(Omagn.z); Serial.print("  ");
        Serial.println("uT");
        /* Display the gyroscope results (gyroscope data is in g) */
        Serial.print("G ");
        Serial.print("X: "); Serial.print(Ogyro.x); Serial.print("  ");
        Serial.print("Y: "); Serial.print(Ogyro.y); Serial.print("  ");
        Serial.print("Z: "); Serial.print(Ogyro.z); Serial.print("  ");
        Serial.println("g");
        /* Display the accelerometer results (accelerometer data is in m/s^2) */
        Serial.print("A ");
        Serial.print("X: "); Serial.print(Oaccel.x); Serial.print("  ");
        Serial.print("Y: "); Serial.print(Oaccel.y); Serial.print("  ");
        Serial.print("Z: "); Serial.print(Oaccel.z); Serial.print("  ");
        Serial.println("m/s^2");
        /* Display the accelerometer results (accelerometer data is in m/s^2) */
        Serial.print("Q ");
        Serial.print("X: "); Serial.print(q0); Serial.print("  ");
        Serial.print("Y: "); Serial.print(q1); Serial.print("  ");
        Serial.print("Z: "); Serial.print(q2); Serial.print("  ");
        Serial.println("");

        delay(1000);
    }
}

/**************************************************************************
 *                     ROS communication Demo
 **************************************************************************/
std_msgs::String str_msg;
ros::Publisher chatter("tiva_chatter", &str_msg);
char hello_msg[] = "Hello world from Tiva C!";
ros::Subscriber<std_msgs::Empty> suber("toggle_led", &message_Callback);

void message_Callback(const std_msgs::Empty& toggle_msg)
{
    digitalWrite(TIVA_BLUE_LED, HIGH - digitalRead(TIVA_BLUE_LED));
}

void ros_PubSub_demo()
{
    DEBUG_PRINTF("Start ROS chatter communication Demo\n");

    pinMode(TIVA_BLUE_LED, OUTPUT);

    h_Node.initNode();
    h_Node.advertise(chatter);
    h_Node.subscribe(suber);

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
void pidMotorControl_demo()
{
    DEBUG_PRINTF("Start PID Motor Control Demo\n");
    
    /* PID Regulate periodic with SAMPLETIME = 2ms or 500Hz freq */
    attachTimerInterrupt(PID_TIMER_BASE, PID_TIMER_SYSCTL_PERIPH, &PID_TimerInterrupt_Handler, 500);

#ifdef DEBUG_PID
    attachTimerInterrupt(DEBUG_TIMER_BASE, DEBUG_TIMER_SYSCTL_PERIPH, &DEBUGGER_TimerInterrupt_Handler, 1);
#endif 

    DEMO_MOTOR_WHEEL.setupInterrupt();
    DEMO_MOTOR_WHEEL.PIDEnable(KC, TAUI, TAUD, SAMPLETIME);

    /* Main loop */
    for (;;)
    {
        DEBUG_PRINTF("Motor run ADVANCE MMPS: 100 in 3sec\n");
        DEMO_MOTOR_WHEEL.setSpeedMMPS(50, DIR_ADVANCE);
        delay(3000);
        DEBUG_PRINTF("Motor run ADVANCE MMPS: 200 in 3sec\n");
        DEMO_MOTOR_WHEEL.setSpeedMMPS(200, DIR_ADVANCE);
        delay(3000);
        DEBUG_PRINTF("Motor run BACKOFF MMPS: 200 in 3sec\n");
        DEMO_MOTOR_WHEEL.setSpeedMMPS(200, DIR_BACKOFF);
        delay(3000);
        DEBUG_PRINTF("Motor run BACKOFF MMPS: 50 in 3sec\n");
        DEMO_MOTOR_WHEEL.setSpeedMMPS(50, DIR_BACKOFF);
        delay(2000); 
        DEBUG_PRINTF("Motor Stop in 3sec\n");
        DEMO_MOTOR_WHEEL.setSpeedMMPS(0, DIR_ADVANCE);
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

    pinMode(TIVA_RED_LED, OUTPUT);

    for (;;) 
    {
        DEBUG_PRINTF("LED Blink %d", blinkCounter ++);
        digitalWrite(TIVA_RED_LED, LOW);
        delay(500);
        digitalWrite(TIVA_RED_LED, HIGH);
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

#endif