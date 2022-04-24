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


TwoWire Wire2;
/**
 *  IMU BMX160 handle object
 */
DFRobot_BMX160 bmx160(&Wire2);
sBmx160SensorData_t Omagn, Ogyro, Oaccel;

/**************************************************************************
 *                      Nexus Bot Demo MAIN Functions
 **************************************************************************/
void main_program()
{
#ifdef DEBUG
    debug_init();
#endif

    // ledBlink_demo();
    // ledBlinkPWM_demo();
    // externalInterrupt_demo();
    // dbgPrintf_demo();
    // pidMotorControl_demo();
    // tmrInterrupt_demo();
    // ros_PubSub_demo();
    // bmx160_demo();
    // lcd16x2_I2CLCD_demo();

    DEBUG_PRINTF("Start MAIN Demo\n");

    Wire2.setModule(2);

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

        delay(1000);
    }
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
