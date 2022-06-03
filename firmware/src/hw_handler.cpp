#include "hw_handler.h"

#define DEBUG_PID

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
    omniNexusBot.PIDRegulate();
}


void IMU_TimerInterrupt_Handler()
{
    MAP_TimerIntClear(IMU_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    /* Get a new sensor event */
    bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);
    
    /* Passing current time stamp to ROS sensor message */
    imu_GyroAccel_msg.header.stamp = h_Node.now();
    imu_Mag_msg.header.stamp = h_Node.now();
    // imu_data.header.stamp = h_Node.now();
    // imu_data.header.frame_id = "imu_madgwick";

    /* Frame_id to ROS sensor message */
    imu_GyroAccel_msg.header.frame_id = "imu_gyro_accel";
    imu_Mag_msg.header.frame_id = "imu_mag";
    
    /* Passing IMU raw data to ROS sensor message */
    imu_GyroAccel_msg.linear_acceleration.x = Oaccel.x; // * 9.8;
    imu_GyroAccel_msg.linear_acceleration.y = Oaccel.y; // * 9.8;
    imu_GyroAccel_msg.linear_acceleration.z = Oaccel.z; // * 9.8;
    imu_GyroAccel_msg.angular_velocity.x = Ogyro.x * DEG_TO_RAD;
    imu_GyroAccel_msg.angular_velocity.y = Ogyro.y * DEG_TO_RAD;
    imu_GyroAccel_msg.angular_velocity.z = Ogyro.z * DEG_TO_RAD;
    imu_Mag_msg.magnetic_field.x = Omagn.x;
    imu_Mag_msg.magnetic_field.y = Omagn.y;
    imu_Mag_msg.magnetic_field.z = Omagn.z;

    // madgwickFilter.update(Ogyro.x, Ogyro.y, Ogyro.z,
    //                       Oaccel.x, Oaccel.y, Oaccel.z,
    //                       Omagn.x, Omagn.y, Omagn.z);
    
    // imu_data.x = madgwickFilter.getRoll();
    // imu_data.y = madgwickFilter.getPitch();
    // imu_data.z = madgwickFilter.getYaw();

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
