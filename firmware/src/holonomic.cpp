#include "holonomic.h"

void generate_matrix_wheel_body(float matrix_w_b[][3], float matrix_b_w[][3])
{
    float angle_between_wheels[3];
    angle_between_wheels[0] = 180 * PI / 180; //rad
    angle_between_wheels[1] = (-60) * PI / 180; //rad
    angle_between_wheels[2] = 60 * PI / 180; //rad
    // see reference: https://github.com/cvra/CVRA-doc/blob/master/holonomic.pdf
    for (i = 0; i < 3; i++)
    {
        matrix_b_w[i][0] = -BODY_RADIUS;
        matrix_b_w[i][1] = sin(angle_between_wheels[i]);
        matrix_b_w[i][2] = -cos(angle_between_wheels[i]);
    }
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            matrix_w_b[i][j] = matrix_b_w[i][j]; //create a copy
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
    matrix_speed_wheels_encoder[0] = omniNexusBot.wheelBackGetSpeedMMPS();
    matrix_speed_wheels_encoder[1] = omniNexusBot.wheelRightGetSpeedMMPS();
    matrix_speed_wheels_encoder[2] = omniNexusBot.wheelLeftGetSpeedMMPS();

    robot_speed_body_frame[0] = robot_speed_body_frame[1] = *omega = 0;

    for (i = 0; i <= 2; i++)
    {
      *omega = *omega + matrix_w_b[0][i] * matrix_speed_wheels_encoder[i];
      robot_speed_body_frame[0] = robot_speed_body_frame[0] + matrix_w_b[1][i] * matrix_speed_wheels_encoder[i]; // vx
      robot_speed_body_frame[1] = robot_speed_body_frame[1] + matrix_w_b[2][i] * matrix_speed_wheels_encoder[i]; // vy
    }
}

void robot_transform_body_to_inertial(float heading_angle, float vector[], float transformed_vector[])
{
    float rotation_z_axis[2][2] = {0};
    generate_rot_matrix_body_to_inertial(rotation_z_axis, heading_angle);
    transformed_vector[0] = rotation_z_axis[0][0] * vector[0] + rotation_z_axis[0][1] * vector[1];
    transformed_vector[1] = rotation_z_axis[1][0] * vector[0] + rotation_z_axis[1][1] * vector[1];
}

void robot_integrate_speed(holoOdom_t hOdom, float velocity[], float omega, float delta_t)
{
    hOdom.heading = hOdom.heading + omega*delta_t;
    hOdom.position_x = hOdom.position_x + velocity[0]*delta_t; // x
    hOdom.position_y = hOdom.position_y + velocity[1]*delta_t; // y
}