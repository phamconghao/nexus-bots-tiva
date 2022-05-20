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
    /**
     * Init the hardware bmx160 IMU
     */
    Wire2.setModule(2);
    if (bmx160.begin() != true)
    {
        DEBUG_PRINTF("Initialization faild, please check the I2C connect!\n");
        while(1);
    }

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

    /* 
     * Init the hardware PID
     */
    motorWheel_Back.setupInterrupt();
    motorWheel_Back.PIDEnable(KC, TAUI, TAUD, SAMPLETIME);
    motorWheel_Right.setupInterrupt();
    motorWheel_Right.PIDEnable(KC, TAUI, TAUD, SAMPLETIME);
    motorWheel_Left.setupInterrupt();
    motorWheel_Left.PIDEnable(KC, TAUI, TAUD, SAMPLETIME);

    /* 
     * Init the STT LED on Nexus controller Board
     */
    pinMode(NEXUS_STT_LED, OUTPUT);
}

void main_program()
{
#ifdef DEBUG
    debug_init();
#endif

#ifdef DEMO     // --> For DEMO
    pinMode(NEXUS_STT_LED, OUTPUT);
    sttLED_Flash();

    // ledBlink_demo();
    // ledBlinkPWM_demo();
    // externalInterrupt_demo();
    // dbgPrintf_demo();
    // pidMotorControl_demo();
    // tmrInterrupt_demo();
    // ros_PubSub_demo();
    // bmx160_demo();
    // lcd16x2_I2CLCD_demo();
    nexusControl_demo();

#else           // --> For MAIN Program

    DEBUG_PRINTF("Start MAIN Program\n");

    hardware_Init();

    /* Start PID Regulate Task, periodic with SAMPLETIME = 2ms or 500Hz freq */
    attachTimerInterrupt(PID_TIMER_BASE, PID_TIMER_SYSCTL_PERIPH, &PID_TimerInterrupt_Handler, 500);
    /* Start Reading IMU Task, periodic with 100Hz frequency */
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

        // lcd.print("Hello LCD");

        delay(1000);
    }
#endif          // --> Endif DEMO
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
