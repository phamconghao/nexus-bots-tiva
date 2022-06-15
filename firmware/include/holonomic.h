#ifndef  HOLONOMIC_H
#define  HOLONOMIC_H

#include "main.h"

typedef struct
{
    /* data */
    float position_x;
    float position_y;
    float heading;
} holoOdom_t;

/**
 * @brief 
 * 
 * @param matrix_w_b 
 * @param matrix_b_w 
 */
void generate_matrix_wheel_body(float matrix_w_b[3][3], float matrix_b_w[3][3]);

/**
 * @brief 
 * 
 * @param rotation_z_axis 
 * @param theta 
 */
void generate_rot_matrix_body_to_inertial(float rotation_z_axis[][2], float theta);

// Formula 10th
/**
 * @brief Get the speed body frame from encoderMMPS object
 * 
 * @param robot_speed_body_frame 
 * @param omega 
 */
void get_speed_body_frame_from_encoderMMPS(float robot_speed_body_frame[], float *omega);

/**
 * @brief 
 * 
 * @param position 
 * @param heading 
 * @param velocity 
 * @param omega 
 * @param delta_t 
 */
void robot_integrate_speed(float position[], float *heading, float velocity[], float omega, float delta_t);

/**
 * @brief 
 * 
 * @param heading_angle 
 * @param vector 
 * @param transformed_vector 
 */
void robot_transform_body_to_inertial(holoOdom_t hOdom, float vector[], float transformed_vector[]);

/**
 * @brief 
 * 
 * @param hOdom 
 * @param velocity 
 * @param omega 
 * @param delta_t 
 */
void robot_integrate_speed(holoOdom_t hOdom, float velocity[], float omega, float delta_t);

#endif  // HOLONOMIC_H