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
