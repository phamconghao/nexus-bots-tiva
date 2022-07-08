/**
 *   @file  main.cpp
 *
 *   @brief
 *     Main implementation of the Nexus Demo
 */

/**************************************************************************
 *                            Include Files
 **************************************************************************/
/* Main Include Files */
#include "main.h"

/**************************************************************************
 *                           Global Declarations
 **************************************************************************/

/**
 *  Global Variable for Motor Wheel Back Encoder parameters
 */
Encoder_Params_t encoderWheel_Back_Params = {.Encoder_Handler = 
                                                encoderWheel_Back_Handler};
/**
 *  Global Variable for Motor Wheel Right Encoder parameters
 */
Encoder_Params_t encoderWheel_Right_Params = {.Encoder_Handler = 
                                                encoderWheel_Right_Handler};
/**
 *  Global Variable for Motor Wheel Left Encoder parameters
 */
Encoder_Params_t encoderWheel_Left_Params = {.Encoder_Handler = 
                                                encoderWheel_Left_Handler};

/**************************************************************************
 *                           Local Declarations
 **************************************************************************/

/**
 *  Wheel _Back_ object for control task
 */
MotorWheel motorWheel_Back(M1_PWM, M1_DIR_A, M1_DIR_B, M1_ENCA, M1_ENCB, 
                &encoderWheel_Back_Params,
                REDUCTION_RATIO_NAMIKI_MOTOR, WHEEL_CIRC);
/**
 *  Wheel _Right_ object for control task
 */
MotorWheel motorWheel_Right(M2_PWM, M2_DIR_A, M2_DIR_B, M2_ENCA, M2_ENCB, 
                &encoderWheel_Right_Params,
                REDUCTION_RATIO_NAMIKI_MOTOR, WHEEL_CIRC);
/**
 *  Wheel _Left_ object for control task
 */
MotorWheel motorWheel_Left(M3_PWM, M3_DIR_A, M3_DIR_B, M3_ENCA, M3_ENCB, 
                &encoderWheel_Left_Params,
                REDUCTION_RATIO_NAMIKI_MOTOR, WHEEL_CIRC);

/**
 *  Omni Nexus object for control task
 */
Omni3WD omniNexusBot(&motorWheel_Back, &motorWheel_Right, &motorWheel_Left);

/**
 *  Node handle object for communication task
 */
ros::NodeHandle h_Node;
ros::Subscriber<geometry_msgs::Twist> sub_velTwist("/cmd_vel", &cmdTwistSpeedCallback);
ros::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_initPose("/initialpose", &initialPoseCallback);
// ros::Publisher pub_GyroAccel("imu/data_raw", &imu_GyroAccel_msg);
// ros::Publisher pub_Mag("imu/mag", &imu_Mag_msg);
ros::Publisher pub_Odom("/odom", &odom_msg);
// sensor_msgs::Imu imu_GyroAccel_msg;
// sensor_msgs::MagneticField imu_Mag_msg;
nav_msgs::Odometry odom_msg;
nav_msgs::Odometry odom_initpose;

/**
 *  IMU BMX160 handle object
 */
DFRobot_BMX160 bmx160;
sBmx160SensorData_t Omagn, Ogyro, Oaccel;

/**
 *  PCF8574 LCD handle object
 */
LiquidCrystal_PCF8574 lcd(LCD_I2C_ADDR);

/**
 *  Holonomic handle
 */
// holoOdom_t nexusHoloOdom;
int i = 0, j = 0;
float matrix_speed_wheels_encoder[3];
float matrix_w_b[3][3];
float matrix_b_w[3][3];
float robot_heading_inertial;
float robot_speed_inertial_frame[2]; 
float robot_speed_body_frame[2];
float omega;
float robot_position_inertial[2];
bool initialPoseReceived = false;


/**************************************************************************
 *                      Nexus Bot Demo MAIN Program
 **************************************************************************/
void hardware_Init()
{
    /* 
     * Init the STT LED on Nexus controller Board
     */
    pinMode(NEXUS_STT_LED, OUTPUT);

#if LCD_OPTION
    /* 
     * Init the hardware I2C LCD 
     */
    Wire.begin();
    Wire.beginTransmission(LCD_I2C_ADDR);

    if (Wire.endTransmission())
    {
        DEBUG_PRINTF("  LCD not found.\n");
    }
    else
    {
        DEBUG_PRINTF("  LCD found.\n");
        /* Initialize the LCD */
        lcd.begin(LCD_COLS_NUM, LCD_ROWS_NUM);
    }
#endif

#if IMU_OPTION
    /**
     * Init the hardware bmx160 IMU
     */
    while (bmx160.begin() != true)
    {
        DEBUG_PRINTF("Initialization faild, please check the I2C connect!\n");
        sttLED_Flash();
        delay(1000);
    }
#endif 

    /** Enable PID for object Omni Nexus */
    omniNexusBot.PIDEnable(KC, TAUI, TAUD, SAMPLETIME);
}

void main_program()
{
#ifdef DEBUG
    debug_init();
#endif

#ifdef DEMO     // --> For DEMO
    pinMode(NEXUS_STT_LED, OUTPUT);
    sttLED_Flash();

    // ledBlink_demo();
    // ledBlinkPWM_demo();
    // externalInterrupt_demo();
    // dbgPrintf_demo();
    // pidMotorControl_demo();
    // tmrInterrupt_demo();
    // ros_PubSub_demo();
    // bmx160_demo();
    // lcd16x2_I2CLCD_demo();
    // nexusControl_demo();
    ros_Odom_demo();

#else           // --> For MAIN Program
    unsigned long previous_time_ms = 0;
    unsigned long current_time_ms = 0;
    float delta_t = 0;
    unsigned long delta_t_ms = 0;

    DEBUG_PRINTF("Start MAIN Program\n");

    hardware_Init();

    /* Start PID Regulate Task, periodic with SAMPLETIME = 2ms or 500Hz freq */
    attachTimerInterrupt(PID_TIMER_BASE, PID_TIMER_SYSCTL_PERIPH, &PID_TimerInterrupt_Handler, 500);
#if IMU_OPTION
    /* Start Reading IMU Task, periodic with 100Hz frequency */
    attachTimerInterrupt(IMU_TIMER_BASE, IMU_TIMER_SYSCTL_PERIPH, &IMU_TimerInterrupt_Handler, 100);
#endif
    
    // Initialize position and pose of the robot before the robot works.
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;

    // generate_robot_body(matrix_wheel_robot);
    generate_matrix_wheel_body(matrix_w_b, matrix_b_w);
    
    h_Node.initNode();
    /* Subscribe velocity data from ROS */
    h_Node.subscribe(sub_velTwist);
    h_Node.subscribe(sub_initPose);
    
#if IMU_OPTION    
    h_Node.advertise(pub_GyroAccel);
    h_Node.advertise(pub_Mag);
#endif

    h_Node.advertise(pub_Odom);
    
    // omniNexusBot.setCarRotateLeft(5);
    // omniNexusBot.setCarRight(50);
    previous_time_ms = millis();

    /* Main loop */
    for (;;)
    {
#if IMU_OPTION
        /* Publish IMU data to ROS */
        pub_GyroAccel.publish(&imu_GyroAccel_msg);
        pub_Mag.publish(&imu_Mag_msg);
#endif

#if CAL_ODOM

        if (initialPoseReceived)
        {
            odom_msg.pose.pose.position.x += odom_initpose.pose.pose.position.x;
            odom_msg.pose.pose.position.y += odom_initpose.pose.pose.position.y;
            odom_msg.pose.pose.position.z += odom_initpose.pose.pose.position.z;
            odom_msg.pose.pose.orientation.z += odom_initpose.pose.pose.orientation.z;
            initialPoseReceived = false;
        }
        current_time_ms = millis();
        get_speed_body_frame_from_encoderMMPS(robot_speed_body_frame, &omega);
        delta_t_ms = current_time_ms - previous_time_ms;
        delta_t = (float)delta_t_ms / 1000;
        robot_transform_body_to_inertial(robot_heading_inertial, robot_speed_body_frame, robot_speed_inertial_frame);
        robot_integrate_speed(robot_position_inertial, &robot_heading_inertial, robot_speed_inertial_frame, omega, delta_t);
        previous_time_ms = current_time_ms;
        odom_msg.header.stamp = h_Node.now();
        // Since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quater = tf::createQuaternionFromYaw(robot_heading_inertial);

        odom_msg.header.frame_id = "odom_p";
        odom_msg.child_frame_id = "base_footprint_p";
        // Set the position
        odom_msg.pose.pose.position.x = robot_position_inertial[0];
        odom_msg.pose.pose.position.y = robot_position_inertial[1];
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quater;
        // Set the velocity
        odom_msg.twist.twist.linear.x = robot_speed_inertial_frame[0];
        odom_msg.twist.twist.linear.y = robot_speed_inertial_frame[1];
        odom_msg.twist.twist.angular.z = robot_heading_inertial;

        sttLED_Flash();
        for (int i = 0; i < 36; i++)
        {
            if (i == 0 || i == 7)
            {
                odom_msg.pose.covariance[i] = 1e-3;
                odom_msg.twist.covariance[i] = 1e-3;
            }
            else if (i == 14 || i == 21 || i == 28)
            {
                odom_msg.pose.covariance[i] = 1e6;
                odom_msg.twist.covariance[i] = 1e6;
            }
            else if (i == 35)
            {
                odom_msg.pose.covariance[i] = 1e3;
                odom_msg.twist.covariance[i] = 1e3;
            }
            else
            {
                odom_msg.pose.covariance[i] = 0;
                odom_msg.twist.covariance[i] = 0;
            }
        }
        /* Publish Odom data to ROS */
        pub_Odom.publish(&odom_msg);
#endif
        h_Node.spinOnce();
        delay(5);
    }
#endif          // --> Endif DEMO
}

void attachTimerInterrupt(uint32_t ui32Base, uint32_t ui32Peripheral, void (*p_TmrHandler)(), unsigned int tmrFreq)
{
    // Timer Period calculation
    unsigned long ulPeriod = SysCtlClockGet() / tmrFreq;
    // Timer interrupt Configuration
    MAP_SysCtlPeripheralEnable(ui32Peripheral);
    MAP_TimerConfigure(ui32Base, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(ui32Base, TIMER_A, ulPeriod - 1);
    TimerIntRegister(ui32Base, TIMER_A, p_TmrHandler);
    MAP_TimerIntEnable(ui32Base, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(ui32Base, TIMER_A);
}

void sttLED_Flash(void)
{
    digitalWrite(NEXUS_STT_LED, HIGH);
    delay(50);
    digitalWrite(NEXUS_STT_LED, LOW);
    delay(50);
    digitalWrite(NEXUS_STT_LED, HIGH);
    delay(50);
    digitalWrite(NEXUS_STT_LED, LOW);
    digitalWrite(NEXUS_STT_LED, HIGH);
    delay(50);
    digitalWrite(NEXUS_STT_LED, LOW);
}

void cmdTwistSpeedCallback(const geometry_msgs::Twist& cmd_twistSpeed)
{
    robot_control_wheel(cmd_twistSpeed.linear.x, cmd_twistSpeed.angular.z);
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& init_pose)
{
    odom_initpose.pose.pose.position.x = init_pose.pose.pose.position.x;
    odom_initpose.pose.pose.position.y = init_pose.pose.pose.position.y;
    odom_initpose.pose.pose.orientation.z = init_pose.pose.pose.orientation.z;
    odom_initpose.pose.pose.orientation.w = init_pose.pose.pose.orientation.w;
    initialPoseReceived = true;
}