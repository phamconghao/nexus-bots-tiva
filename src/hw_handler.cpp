#include "hw_handler.h"

void encoderWheel_Back_Handler(void)
{
    motorWheel_Back.encoderHandler();
}

void encoderWheel_Right_Handler(void)
{
    motorWheel_Right.encoderHandler();
}

void encoderWheel_Left_Handler(void)
{
    motorWheel_Left.encoderHandler();
}

void extInterruptDemoHandler()
{
    DEBUG_PRINTF("External Interrupt Occured!");
}

void PID_TimerInterrupt_Handler()
{
    MAP_TimerIntClear(PID_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    motorWheel_Back.PIDRegulate();
}


void IMU_TimerInterrupt_Handler()
{
    MAP_TimerIntClear(IMU_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    /* Get a new sensor event */
    bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);
    // Ogyro.x = 33;
    // Ogyro.x = 33;
    // Ogyro.x = 33;
    
    // Oaccel.x = 11;
    // Oaccel.y = 11;
    // Oaccel.z = 11;

    // Omagn.x = 22;
    // Omagn.y = 22;
    // Omagn.y = 22;
    MadgwickAHRSupdate(Ogyro.x, Ogyro.y, Ogyro.z,
                       Oaccel.x, Oaccel.y, Oaccel.z,
                       Omagn.x, Omagn.y, Omagn.x);

    // // Accelerometer
    // Serial.print(Oaccel.x, DEC);
    // Serial.print("\t");
    // Serial.print(Oaccel.y, DEC);
    // Serial.print("\t");
    // Serial.print(Oaccel.z, DEC);
    // Serial.print("\t");

    // // Gyroscope
    // Serial.print(Ogyro.x, DEC);
    // Serial.print("\t");
    // Serial.print(Ogyro.y, DEC);
    // Serial.print("\t");
    // Serial.print(Ogyro.z, DEC);
    // Serial.print("\t");

    // // Magnetometer
    // Serial.print(Omagn.x + 200, DEC);
    // Serial.print("\t");
    // Serial.print(Omagn.y - 70, DEC);
    // Serial.print("\t");
    // Serial.print(Omagn.z - 700, DEC);
    // Serial.print("\t");
}

#ifdef DEBUG_PID
void DEBUGGER_TimerInterrupt_Handler()
{
    MAP_TimerIntClear(DEBUG_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    DEBUG_PRINTF("EncoderWheel_1_");
    DEBUG_PRINTF("\tcurDir -> %d", encoderWheel_Back_Params.currDirection);
    DEBUG_PRINTF("\tPID Out -> %d", motorWheel_Back.getSpeedRPMOutput());
    DEBUG_PRINTF("\tSpeedMMPS -> %d\n", motorWheel_Back.getSpeedMMPS());
}
#endif
