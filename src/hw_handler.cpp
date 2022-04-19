#include "MotorWheel.h"
#include "hw_handler.h"

void encoderWheel_1_Handler(void)
{
    wheel1.encoderHandler();
}

void encoderWheel_2_Handler(void)
{
    wheel2.encoderHandler();
}

void encoderWheel_3_Handler(void)
{
    wheel3.encoderHandler();
}

void extInterruptDemoHandler()
{
    DEBUG_PRINTF("External Interrupt Occured!");
}
