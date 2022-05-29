/**
 *   @file  main.cpp
 *
 *   @brief
 *     Main implementation of the Nexus Demo
 */

/**************************************************************************
 *                            Include Files
 **************************************************************************/
/* Main Include Files */
#include "main.h"

/**************************************************************************
 *                           Global Declarations
 **************************************************************************/

/**
 *  Global Variable for Motor Wheel Back Encoder parameters
 */
Encoder_Params_t encoderWheel_Back_Params = {.Encoder_Handler = 
                                                encoderWheel_Back_Handler};
/**
 *  Global Variable for Motor Wheel Right Encoder parameters
 */
Encoder_Params_t encoderWheel_Right_Params = {.Encoder_Handler = 
                                                encoderWheel_Right_Handler};
/**
 *  Global Variable for Motor Wheel Left Encoder parameters
 */
Encoder_Params_t encoderWheel_Left_Params = {.Encoder_Handler = 
                                                encoderWheel_Left_Handler};

/**************************************************************************
 *                           Local Declarations
 **************************************************************************/

/**
 *  Wheel _Back_ object for control task
 */
MotorWheel motorWheel_Back(M1_PWM, M1_DIR_A, M1_DIR_B, M1_ENCA, M1_ENCB, 
                &encoderWheel_Back_Params,
                REDUCTION_RATIO_NAMIKI_MOTOR, WHEEL_CIRC);
/**
 *  Wheel _Right_ object for control task
 */
MotorWheel motorWheel_Right(M2_PWM, M2_DIR_A, M2_DIR_B, M2_ENCA, M2_ENCB, 
                &encoderWheel_Right_Params,
                REDUCTION_RATIO_NAMIKI_MOTOR, WHEEL_CIRC);
/**
 *  Wheel _Left_ object for control task
 */
MotorWheel motorWheel_Left(M3_PWM, M3_DIR_A, M3_DIR_B, M3_ENCA, M3_ENCB, 
                &encoderWheel_Left_Params,
                REDUCTION_RATIO_NAMIKI_MOTOR, WHEEL_CIRC);

/**
 *  Omni Nexus object for control task
 */
Omni3WD omniNexusBot(&motorWheel_Back, &motorWheel_Right, &motorWheel_Left);

/**
 *  Node handle object for communication task
 */
ros::NodeHandle h_Node;
ros::Subscriber<geometry_msgs::Twist> sub_velTwist("/cmd_vel", &cmdTwistSpeedCallback);

/**
 *  IMU BMX160 handle object
 */
DFRobot_BMX160 bmx160;
sBmx160SensorData_t Omagn, Ogyro, Oaccel;

/**
 *  PCF8574 LCD handle object
 */
LiquidCrystal_PCF8574 lcd(LCD_I2C_ADDR);

/**************************************************************************
 *                      Nexus Bot Demo MAIN Program
 **************************************************************************/
void hardware_Init()
{
    /* 
     * Init the STT LED on Nexus controller Board
     */
    pinMode(NEXUS_STT_LED, OUTPUT);

#if LCD_OPTION
    /* 
     * Init the hardware I2C LCD 
     */
    Wire.begin();
    Wire.beginTransmission(LCD_I2C_ADDR);

    if (Wire.endTransmission())
    {
        DEBUG_PRINTF("  LCD not found.\n");
    }
    else
    {
        DEBUG_PRINTF("  LCD found.\n");
        /* Initialize the LCD */
        lcd.begin(LCD_COLS_NUM, LCD_ROWS_NUM);
    }
#endif

#if IMU_OPTION
    /**
     * Init the hardware bmx160 IMU
     */
    while (bmx160.begin() != true)
    {
        DEBUG_PRINTF("Initialization faild, please check the I2C connect!\n");
        sttLED_Flash();
        delay(1000);
    }
#endif 

    /* 
     * Init the hardware PID
     */
    motorWheel_Back.setupInterrupt();
    motorWheel_Back.PIDEnable(KC, TAUI, TAUD, SAMPLETIME);
    motorWheel_Right.setupInterrupt();
    motorWheel_Right.PIDEnable(KC, TAUI, TAUD, SAMPLETIME);
    motorWheel_Left.setupInterrupt();
    motorWheel_Left.PIDEnable(KC, TAUI, TAUD, SAMPLETIME);
}

void main_program()
{
#ifdef DEBUG
    debug_init();
#endif

#ifdef DEMO      // --> For DEMO
    pinMode(NEXUS_STT_LED, OUTPUT);
    sttLED_Flash();

    // ledBlink_demo();
    // ledBlinkPWM_demo();
    // externalInterrupt_demo();
    // dbgPrintf_demo();
    // pidMotorControl_demo();
    // tmrInterrupt_demo();
    ros_PubSub_demo();
    // bmx160_demo();
    // lcd16x2_I2CLCD_demo();
    // nexusControl_demo();

#else           // --> For MAIN Program

    DEBUG_PRINTF("Start MAIN Program\n");

    hardware_Init();
    sttLED_Flash();

    /* Start PID Regulate Task, periodic with SAMPLETIME = 2ms or 500Hz freq */
    attachTimerInterrupt(PID_TIMER_BASE, PID_TIMER_SYSCTL_PERIPH, &PID_TimerInterrupt_Handler, 500);
    /* Start Reading IMU Task, periodic with 100Hz frequency */
    // attachTimerInterrupt(IMU_TIMER_BASE, IMU_TIMER_SYSCTL_PERIPH, &IMU_TimerInterrupt_Handler, 100);

    /** Enable PID for object Omni Nexus */
    omniNexusBot.PIDEnable(KC, TAUI, TAUD, SAMPLETIME);
    
    h_Node.initNode();
    h_Node.subscribe(sub_velTwist);

    /* Main loop */
    for (;;)
    {
        h_Node.spinOnce();
        delay(2);
    }
#endif         // --> Endif DEMO
}

void attachTimerInterrupt(uint32_t ui32Base, uint32_t ui32Peripheral, void (*p_TmrHandler)(), unsigned int tmrFreq)
{
    // Timer Period calculation
    unsigned long ulPeriod = SysCtlClockGet() / tmrFreq;
    // Timer interrupt Configuration
    MAP_SysCtlPeripheralEnable(ui32Peripheral);
    MAP_TimerConfigure(ui32Base, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(ui32Base, TIMER_A, ulPeriod - 1);
    TimerIntRegister(ui32Base, TIMER_A, p_TmrHandler);
    MAP_TimerIntEnable(ui32Base, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(ui32Base, TIMER_A);
}

void sttLED_Flash(void)
{
    digitalWrite(NEXUS_STT_LED, HIGH);
    delay(50);
    digitalWrite(NEXUS_STT_LED, LOW);
    delay(50);
    digitalWrite(NEXUS_STT_LED, HIGH);
    delay(50);
    digitalWrite(NEXUS_STT_LED, LOW);
    digitalWrite(NEXUS_STT_LED, HIGH);
    delay(50);
    digitalWrite(NEXUS_STT_LED, LOW);
}

uint8_t geoLinear2mmps(float linearValue)
{
    return linearValue * 1000;
}

uint8_t geoAngular2mmps(float angularValue)
{
    return angularValue * 300;
}

bool positive_inRange(float min_value, float max_value, float comp_value)
{
    return ((min_value < comp_value) && (comp_value <= max_value));
}

bool negative_inRange(float min_value, float max_value, float comp_value)
{
    return ((min_value <= comp_value) && (comp_value < max_value));
}

void cmdTwistSpeedCallback(const geometry_msgs::Twist& cmd_twistSpeed)
{
    uint8_t mappedSpeed = 0;

    /* Robot Stop Control */
    if (cmd_twistSpeed.linear.x == 0 && cmd_twistSpeed.angular.z == 0)
    {
        omniNexusBot.setCarStop();
    }

    /* Robot Advance Control */
    if (positive_inRange(0, 0.02, cmd_twistSpeed.linear.x) == true)
    {
        mappedSpeed = geoLinear2mmps(0.02);
        omniNexusBot.setCarAdvance(mappedSpeed);
    }
    if (positive_inRange(0.02, 0.04, cmd_twistSpeed.linear.x) == true)
    {
        mappedSpeed = geoLinear2mmps(0.04);
        omniNexusBot.setCarAdvance(mappedSpeed);
    }
    if (positive_inRange(0.04, 0.06, cmd_twistSpeed.linear.x) == true)
    {
        mappedSpeed = geoLinear2mmps(0.06);
        omniNexusBot.setCarAdvance(mappedSpeed);
    }
    if (positive_inRange(0.06, 0.08, cmd_twistSpeed.linear.x) == true)
    {
        mappedSpeed = geoLinear2mmps(0.08);
        omniNexusBot.setCarAdvance(mappedSpeed);
    }
    if (positive_inRange(0.08, 0.1, cmd_twistSpeed.linear.x) == true)
    {
        mappedSpeed = geoLinear2mmps(0.1);
        omniNexusBot.setCarAdvance(mappedSpeed);
    }

    /* Robot Backoff Control */
    if (negative_inRange(-0.02, 0, cmd_twistSpeed.linear.x) == true)
    {
        mappedSpeed = geoLinear2mmps(0.02);
        omniNexusBot.setCarBackoff(mappedSpeed);
    }
    if (negative_inRange(-0.04, -0.02, cmd_twistSpeed.linear.x) == true)
    {
        mappedSpeed = geoLinear2mmps(0.04);
        omniNexusBot.setCarBackoff(mappedSpeed);
    }
    if (negative_inRange(-0.06, -0.04, cmd_twistSpeed.linear.x) == true)
    {
        mappedSpeed = geoLinear2mmps(0.06);
        omniNexusBot.setCarBackoff(mappedSpeed);
    }
    if (negative_inRange(-0.08, -0.06, cmd_twistSpeed.linear.x) == true)
    {
        mappedSpeed = geoLinear2mmps(0.08);
        omniNexusBot.setCarBackoff(mappedSpeed);
    }
    if (negative_inRange(-0.1, -0.08, cmd_twistSpeed.linear.x) == true)
    {
        mappedSpeed = geoLinear2mmps(0.1);
        omniNexusBot.setCarBackoff(mappedSpeed);
    }

    /* Robot RotateLeft Control */
    if (positive_inRange(0, 0.1, cmd_twistSpeed.angular.z) == true)
    {
        mappedSpeed = geoAngular2mmps(0.1);
        omniNexusBot.setCarRotateLeft(mappedSpeed);
    }
    if (positive_inRange(0.1, 0.2, cmd_twistSpeed.angular.z) == true)
    {
        mappedSpeed = geoAngular2mmps(0.2);
        omniNexusBot.setCarRotateLeft(mappedSpeed);
    }
    if (positive_inRange(0.2, 0.3, cmd_twistSpeed.angular.z) == true)
    {
        mappedSpeed = geoAngular2mmps(0.3);
        omniNexusBot.setCarRotateLeft(mappedSpeed);
    }
    if (positive_inRange(0.3, 0.4, cmd_twistSpeed.angular.z) == true)
    {
        mappedSpeed = geoAngular2mmps(0.4);
        omniNexusBot.setCarRotateLeft(mappedSpeed);
    }
    if (positive_inRange(0.4, 0.5, cmd_twistSpeed.angular.z) == true)
    {
        mappedSpeed = geoAngular2mmps(0.5);
        omniNexusBot.setCarRotateLeft(mappedSpeed);
    }
    
    /* Robot RotateRight Control */
    if (negative_inRange(-0.1, 0, cmd_twistSpeed.angular.z) == true)
    {
        mappedSpeed = geoAngular2mmps(0.1);
        omniNexusBot.setCarRotateRight(mappedSpeed);
    }
    if (negative_inRange(-0.2, -0.1, cmd_twistSpeed.angular.z) == true)
    {
        mappedSpeed = geoAngular2mmps(0.2);
        omniNexusBot.setCarRotateRight(mappedSpeed);
    }
    if (negative_inRange(-0.3, -0.2, cmd_twistSpeed.angular.z) == true)
    {
        mappedSpeed = geoAngular2mmps(0.3);
        omniNexusBot.setCarRotateRight(mappedSpeed);
    }
    if (negative_inRange(-0.4, -0.3, cmd_twistSpeed.angular.z) == true)
    {
        mappedSpeed = geoAngular2mmps(0.4);
        omniNexusBot.setCarRotateRight(mappedSpeed);
    }
    if (negative_inRange(-0.5, -0.4, cmd_twistSpeed.angular.z) == true)
    {
        mappedSpeed = geoAngular2mmps(0.5);
        omniNexusBot.setCarRotateRight(mappedSpeed);
    }
    ////////////////////////////////////////////////////////////////

}
