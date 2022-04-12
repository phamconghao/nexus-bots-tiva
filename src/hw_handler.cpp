#include "MotorWheel.h"
#include "hw_handler.h"

//
void encoderWheel_1_Handler()
{
    DEBUG_PRINTF("Encoder interrupt Occured!");

    static bool first_pulse = true;
    encoderWheel_1_Params.pulseEndMicros = micros();
    if ((first_pulse == false) && (encoderWheel_1_Params.pulseEndMicros > encoderWheel_1_Params.pulseStartMicros))
    {
        encoderWheel_1_Params.speedPPS = MICROS_PER_SEC / (encoderWheel_1_Params.pulseEndMicros - encoderWheel_1_Params.pulseStartMicros);
        /* encoderWheel_1_Params.accPPSS=(encoderWheel_1_Params.speedPPS-encoderWheel_1_Params.lastSpeedPPS)*encoderWheel_1_Params.speedPPS; */
    }
    else
    {
        first_pulse = false;
    }
    encoderWheel_1_Params.pulseStartMicros = encoderWheel_1_Params.pulseEndMicros;
    /* encoderWheel_1_Params.lastSpeedPPS=encoderWheel_1_Params.speedPPS; */
    if (encoderWheel_1_Params.pinIRQB != PIN_UNDEFINED)
    {
        encoderWheel_1_Params.currDirection = DIR_INVERSE(digitalRead(encoderWheel_1_Params.pinIRQ) ^ digitalRead(encoderWheel_1_Params.pinIRQB));
    }
    encoderWheel_1_Params.currDirection == DIR_ADVANCE ? ++encoderWheel_1_Params.pulses : --encoderWheel_1_Params.pulses;
}

// //
// void encoderWheel_2_Handler()
// {
//     static bool first_pulse = true;
//     encoderWheel_2_Params.pulseEndMicros = micros();
//     if (first_pulse == false && encoderWheel_2_Params.pulseEndMicros > encoderWheel_2_Params.pulseStartMicros)
//     {
//         encoderWheel_2_Params.speedPPS = MICROS_PER_SEC / (encoderWheel_2_Params.pulseEndMicros - encoderWheel_2_Params.pulseStartMicros);
//         /* encoderWheel_2_Params.accPPSS=(encoderWheel_2_Params.speedPPS-encoderWheel_2_Params.lastSpeedPPS)*encoderWheel_2_Params.speedPPS; */
//     }
//     else
//     {
//         first_pulse = false;
//     }
//     encoderWheel_2_Params.pulseStartMicros = encoderWheel_2_Params.pulseEndMicros;
//     /* encoderWheel_2_Params.lastSpeedPPS=encoderWheel_2_Params.speedPPS; */
//     if (encoderWheel_2_Params.pinIRQB != PIN_UNDEFINED)
//     {
//         encoderWheel_2_Params.currDirection = DIR_INVERSE(digitalRead(encoderWheel_2_Params.pinIRQ) ^ digitalRead(encoderWheel_2_Params.pinIRQB));
//     }
//     encoderWheel_2_Params.currDirection == DIR_ADVANCE ? ++encoderWheel_2_Params.pulses : --encoderWheel_2_Params.pulses;
// }

// //
// void encoderWheel_3_Handler()
// {
//     static bool first_pulse = true;                                                   
//     encoderWheel_3_Params.pulseEndMicros = micros();
//     if (first_pulse == false && encoderWheel_3_Params.pulseEndMicros > encoderWheel_3_Params.pulseStartMicros)
//     {
//         encoderWheel_3_Params.speedPPS = MICROS_PER_SEC / (encoderWheel_3_Params.pulseEndMicros - encoderWheel_3_Params.pulseStartMicros);
//         /* encoderWheel_3_Params.accPPSS=(encoderWheel_3_Params.speedPPS-encoderWheel_3_Params.lastSpeedPPS)*encoderWheel_3_Params.speedPPS; */
//     }
//     else
//     {
//         first_pulse = false;
//     }
//     encoderWheel_3_Params.pulseStartMicros = encoderWheel_3_Params.pulseEndMicros;
//     /* encoderWheel_3_Params.lastSpeedPPS=encoderWheel_3_Params.speedPPS; */
//     if (encoderWheel_3_Params.pinIRQB != PIN_UNDEFINED)
//     {
//         encoderWheel_3_Params.currDirection = DIR_INVERSE(digitalRead(encoderWheel_3_Params.pinIRQ) ^ digitalRead(encoderWheel_3_Params.pinIRQB));
//     }
//     encoderWheel_3_Params.currDirection == DIR_ADVANCE ? ++encoderWheel_3_Params.pulses : --encoderWheel_3_Params.pulses;
// }

void extInterruptDemoHandler()
{
    DEBUG_PRINTF("External Interrupt Occured!");
}
