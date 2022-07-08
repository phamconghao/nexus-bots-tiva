#include "holonomic.h"

void generate_matrix_wheel_body(float matrix_w_b[3][3], float matrix_b_w[3][3])
{
    float angle_between_wheels[3];
    angle_between_wheels[0] = 0 * PI / 180; //rad (wheel back)
    angle_between_wheels[1] = 120 * PI / 180; //rad (wheel left)
    angle_between_wheels[2] = -120 * PI / 180; //rad (wheel right)
    // see reference: https://github.com/cvra/CVRA-doc/blob/master/holonomic.pdf

    for (i = 0; i < 3; i++)
    {
        matrix_b_w[i][0] = -(double)BODY_RADIUS / 1000;
        matrix_b_w[i][1] = sin(angle_between_wheels[i]);
        matrix_b_w[i][2] = -cos(angle_between_wheels[i]);
    }
    

    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            matrix_w_b[i][j] = matrix_b_w[i][j]; //create a copy
        }
    }

    Matrix.Invert((float*)matrix_w_b, 3); // this thing stores the inverted matrix in itself
}

void generate_rot_matrix_body_to_inertial(float rotation_z_axis[][2], float theta)
{
    rotation_z_axis[0][0] = cos(theta);
    rotation_z_axis[0][1] = -sin(theta);
    rotation_z_axis[1][0] = sin(theta);
    rotation_z_axis[1][1] = cos(theta);
}

// Formula 10th
void get_speed_body_frame_from_encoderMMPS(float robot_speed_body_frame[], float *omega)
{
    /* Update current wheels speed in MMPS */
    matrix_speed_wheels_encoder[0] = (double)omniNexusBot.wheelBackGetSpeedMMPS() / 1000;   // r1 * w1
    matrix_speed_wheels_encoder[1] = (double)omniNexusBot.wheelLeftGetSpeedMMPS() / 1000;   // r2 * w2
    matrix_speed_wheels_encoder[2] = (double)omniNexusBot.wheelRightGetSpeedMMPS() / 1000;  // r3 * w3
    if (abs(motorWheel_Back.getSpeedRPM()) <= 300 && abs(motorWheel_Right.getSpeedRPM()) <= 300 && abs(motorWheel_Left.getSpeedRPM()) <= 300)
    {
        matrix_speed_wheels_encoder[0] = 0;
        matrix_speed_wheels_encoder[1] = 0;
        matrix_speed_wheels_encoder[2] = 0;
    }
    DEBUG_PRINTF("SpeedMMPS: %f %f %f\n", matrix_speed_wheels_encoder[0], matrix_speed_wheels_encoder[1], matrix_speed_wheels_encoder[2]);
    DEBUG_PRINTF("SpeedMMPS: %d %d %d\n", motorWheel_Back.getSpeedRPM(), motorWheel_Left.getSpeedRPM(), motorWheel_Right.getSpeedRPM());
    robot_speed_body_frame[0] = 0;  // vx: Velocity x
    robot_speed_body_frame[1] = 0;  // vy: Velocity y
    *omega = 0;                     // omega.

    /* Calculate linear and angular velocity */
    for (i = 0; i <= 2; i++)
    {
        *omega = *omega + matrix_w_b[0][i] * matrix_speed_wheels_encoder[i];
        robot_speed_body_frame[0] = robot_speed_body_frame[0] + matrix_w_b[1][i] * matrix_speed_wheels_encoder[i];
        robot_speed_body_frame[1] = robot_speed_body_frame[1] + matrix_w_b[2][i] * matrix_speed_wheels_encoder[i];
    }
}

void robot_transform_body_to_inertial(float heading_angle, float vector[], float transformed_vector[])
{
    float rotation_z_axis[2][2] = {0};
    generate_rot_matrix_body_to_inertial(rotation_z_axis, heading_angle);
    transformed_vector[0] = (rotation_z_axis[0][0] * vector[0] + rotation_z_axis[0][1] * vector[1]);
    transformed_vector[1] = (rotation_z_axis[1][0] * vector[0] + rotation_z_axis[1][1] * vector[1]);
}

void robot_integrate_speed(float position[], float *heading, float velocity[], float omega, float delta_t)
{
    // *heading = *heading + omega * delta_t * 1.4425;
    *heading = *heading + omega * delta_t * 1.7432;
    // position[0] = position[0] + velocity[0] * delta_t * 1.1444; // x
    position[0] = position[0] + velocity[0] * delta_t * 1.265;
    // position[1] = position[1] + velocity[1] * delta_t * 1.2141; // y
    position[1] = position[1] + velocity[1] * delta_t * 1.2036;
    if (*heading >= 2 * PI || *heading <= - 2 * PI)
    {
        *heading = 0;
    }
}

bool setDirection(float value)
{
    return value > 0;
}

void robot_control_wheel(float vel_x, float angu_z, float vel_y)
{
    float speed_wheel_back = 0;
    float speed_wheel_left = 0;
    float speed_wheel_right = 0;
    float matrix_wheel_body[3];
    matrix_wheel_body[0] = angu_z;
    matrix_wheel_body[1] = vel_x;
    matrix_wheel_body[2] = vel_y;
    for (i = 0; i < 3; i++)
    {
        speed_wheel_back = speed_wheel_back + matrix_b_w[0][i] * matrix_wheel_body[i];
        speed_wheel_left = speed_wheel_left + matrix_b_w[1][i] * matrix_wheel_body[i];
        speed_wheel_right = speed_wheel_right + matrix_b_w[2][i] * matrix_wheel_body[i];
    }
    omniNexusBot.wheelBackSetSpeedMMPS(abs(speed_wheel_back * 300), setDirection(speed_wheel_back));
    omniNexusBot.wheelLeftSetSpeedMMPS(abs(speed_wheel_left * 300), setDirection(speed_wheel_left));
    omniNexusBot.wheelRightSetSpeedMMPS(abs(speed_wheel_right * 300), setDirection(speed_wheel_right));
    // DEBUG_PRINTF("\nVel back: %f, Vel left: %f, Vel right: %f", speed_wheel_back, speed_wheel_left, speed_wheel_right);
}