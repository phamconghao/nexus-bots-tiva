#ifndef  HOLONOMIC_H
#define  HOLONOMIC_H

#include "main.h"

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
 * @param heading_angle 
 * @param vector 
 * @param transformed_vector 
 */
void robot_transform_body_to_inertial(float heading_angle, float vector[], float transformed_vector[]);

/**
 * @brief Calculate position and rotation of robot.
 * 
 * @param hOdom 
 * @param velocity 
 * @param omega 
 * @param delta_t 
 */
void robot_integrate_speed(float position[], float *heading, float velocity[], float omega, float delta_t);

/**
 * @brief Set the Direction object
 * 
 * @param value     Direction velocity of each wheel robot.
 * @return          Advance direction
 * @return          Backoff direction 
 */
bool setDirection(float value);

/**
 * @brief Calculate speed each wheel rotation based on command velocity that is received the ROS. 
 * 
 * @param vel_x     Velocity of x axis
 * @param vel_y     Velocity of y axis
 * @param angu_z    Velocity rotation
 */
void robot_control_wheel(float vel_x, float angu_z, float vel_y = 0);

#endif  // HOLONOMIC_H