#ifndef MAIN_H
#define MAIN_H

#ifndef MICRO_PER_SEC
    #define MICRO_PER_SEC 1000000
#endif

#define BODY_RADIUS                     110 //mm
#define REDUCTION_RATIO_NAMIKI_MOTOR    80
#define WHEEL_RADIUS                    24
#define WHEEL_CIRC                      (WHEEL_RADIUS * 2 * PI)

// Motor 1
#define M1_PWM    5
#define M1_DIR    4
#define M1_ENCA   2
#define M1_ENCB   3

#endif