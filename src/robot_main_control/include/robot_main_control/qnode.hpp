/**
 * @file /include/robot_main_control/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robot_main_control_QNODE_HPP_
#define robot_main_control_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/network.h>
//#endif

#include <string>
#include <QThread>
#include <cmath> // included by HSH
#include <QStringListModel>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sstream>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <robot_msgs/DCmotorCommand.h>
#include <robot_msgs/DCmotorStatus.h>
#endif

/*****************************************************************************
** Define
*****************************************************************************/
#define INIT_LOG_DATA "This core(v1.1.2) is compatible with TB3 Burger"

#define HARDWARE_VER "0.1.0"
#define SOFTWARE_VER "0.1.0"
#define FIRMWARE_VER "0.1.0"

#define CONTROL_MOTOR_SPEED_PERIOD          30   //hz
#define IMU_PUBLISH_PERIOD                  200  //hz
#define CMD_VEL_PUBLISH_PERIOD              30   //hz
#define DRIVE_INFORMATION_PUBLISH_PERIOD    30   //hz
#define VERSION_INFORMATION_PUBLISH_PERIOD  1    //hz

#define WHEEL_NUM                           2
#define WHEEL_RADIUS                        0.0675           // meter
#define WHEEL_SEPARATION                    0.300 //0.28  //0.300           // meter
#define TURNING_RADIUS                      0.330 //0.14  //0.330           // meter
#define ROBOT_RADIUS                        0.360 //0.15  //0.360           // meter

#define ENCODER_MIN                         -32767  //-2147483648     // raw //Modified by HSH
#define ENCODER_MAX                         32767  //2147483648      // raw //Modified by HSH

#define LEFT                                0
#define RIGHT                               1

#define LINEAR                              0
#define ANGULAR                             1

#define MAX_LINEAR_VELOCITY                 1.0320156  // m/s
#define MAX_ANGULAR_VELOCITY                3.0    // rad/s //Modified by HSH

#define TICK2RAD 			0.0042792244821763855// Modified by HSH
//0.0042792244821763855				Test1
//0.004289742136396249				Test2
//#define TICK2RAD                             0.0004756481564487489 
//#define TICK2RAD                            0.00553845 // [deg] * 3.14159265359 / 180


#define DEG2RAD(x)                          (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                          (x * 57.2957795131)  // *180/PI

#define TEST_DISTANCE                       0.300     // meter
#define TEST_RADIAN                         3.14      // 180 degree

#define VELOCITY_UNIT                       2
#define VELOCITY_CONSTANT_VALUE		135 //  
//V_wheel/WHEEL_RADIUS/TICK2RAD/2.55*0.1(1/sec to 1/100ms) // Modified by HSH

//#define VELOCITY_CONSTANT_VALUE             156.148708285 // V = r * w = r      *       (RPM              * ( 2 * 3.14 / 60))
                                                          //           = 0.0675 * (0.906 * Goal_Encoder) * 0.10472)
                                                          // Goal_Encoder = V  * 156.148708285

#define TIMER_COUNT_MAX                     43200000    // 50 Day  = 10msec * 60s * 60m * 24h * 50d

#define LIMIT_X_MAX_VELOCITY		100 // Modified by HSH
//#define LIMIT_X_MAX_VELOCITY                161 //(100ms, ppr, 2ch) // MAX RPM is 146 when powered 12.0v so input 161 => actually 141
                                                                    // 146 / 0.906 (RPM) = 161.147...
/*****************************************************************************
** Added by HSH
*****************************************************************************/

#define ROS_LOOP_RATE				100
#define DRIVE_INFO_UPDATE_LOOP_NUM		10 
#define CONTROL_INFO_UPDATE_LOOP_NUM		10
#define TICK_GAP				32767  //Criteria for TICK_GAP determin

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_main_control {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

        /*******************************************************************************
	** Logging
        *******************************************************************************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

        //QNode qnode_object;
  /*******************************************************************************
	** Button 
  *******************************************************************************/




Q_SIGNALS:
        void loggingUpdated();
        void rosShutdown();

public:

protected:

private:
        int init_argc;
        char** init_argv;
        int robot_image_num;
        int send_time;


        /*******************************************************************************
        ** Subscriber
        *******************************************************************************/
        // Callback function for image msg

        // Callback function for cmd_vel msg
        void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
        ros::Subscriber cmd_vel_sub;

        // Callback function for motor_powermsg
        void motorPowerCallback(const std_msgs::Bool& power_msg);
        ros::Subscriber motor_power_sub;

        // Callback function for reset msg
        void resetCallback(const std_msgs::Empty& reset_msg);
        ros::Subscriber reset_sub;

        // Callback function for encoder msg
        void encoderMsgCallback(const robot_msgs::DCmotorStatus& encoder_msg);
        ros::Subscriber encoder_sub;

        // Callback function for imu msg        
        void openCRImuMsgCallback(const sensor_msgs::Imu& msg);
        ros::Subscriber imu_sub;
        ros::Subscriber openCRimu_sub;

        // Callback function for navi msg
        void naviMsgCallback(const geometry_msgs::PoseStamped& msg);
        ros::Subscriber navi_sub;


        /*******************************************************************************
        ** Publisher
        *******************************************************************************/
        ros::Publisher chatter_publisher;

        //encoders of robot
        ros::Publisher sensor_encoder_pub;

        // Version information of robot
        ros::Publisher version_info_pub;

        // IMU of robot
        sensor_msgs::Imu imu_msg;
        ros::Publisher imu_pub;
        void publishImuMsg();

        // Command velocity of robot using joystic remote controller
        geometry_msgs::Twist cmd_vel_joy_msg;
        ros::Publisher cmd_vel_joy_pub;
        ros::Publisher motor_control_pub;
        void publishCmdVelFromJoyMsg();

        // Odometry of robot
        nav_msgs::Odometry odom;
        ros::Publisher odom_pub;
        void updateOdometry();


        // Joint state of robot
        sensor_msgs::JointState joint_states;
        ros::Publisher joint_states_pub;
        void publishDriveInformation();

        // Battey state of robot
        sensor_msgs::BatteryState battery_state_msg;
        ros::Publisher battery_state_pub;

        // Magnetic field
        sensor_msgs::MagneticField mag_msg;
        ros::Publisher mag_pub;

        // Navi msg
        ros::Publisher navi_pub;

        // tf msg
        ros::Publisher tf_pub;

        // moter vel
        int moter_vel[WHEEL_NUM];

        /*******************************************************************************
        * Update motor information
        *******************************************************************************/
        void updateMotorInfo(int left_tick, int right_tick);
        int last_tick[WHEEL_NUM] = {0, 0};

        /*******************************************************************************
        * Update the joint states
        *******************************************************************************/
        void updateJointStates();
        double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

        /*******************************************************************************
        * Update Goal Velocity
        *******************************************************************************/
        void updateGoalVelocity();

        /*******************************************************************************
        ** aaaa
        *******************************************************************************/
        QStringListModel logging_model;
        sensor_msgs::Image ros_image;
        void initJointStates();

        /*******************************************************************************
        * Declaration for odometry
        *******************************************************************************/
        double encoder_vel[WHEEL_NUM];// = {0.0, 0.0};

        /*******************************************************************************
        * Declaration for controllers
        *******************************************************************************/
        float goal_velocity[VELOCITY_UNIT] = {0.0, 0.0};
        //float goal_velocity_from_button[VELOCITY_UNIT] = {0.0, 0.0};
        float goal_velocity_from_cmd[VELOCITY_UNIT] = {0.0, 0.0};
        //float goal_velocity_from_rc100[VELOCITY_UNIT] = {0.0, 0.0};

        /*******************************************************************************
        * Declaration for SLAM and navigation
        *******************************************************************************/
        unsigned long prev_update_time;
        float odom_pose[3];
        double odom_vel[3];

        /*******************************************************************************
        * Calculate the minimam vel, maxiam vel
        *******************************************************************************/
        float constrain(float x, float a, float b);

        /*******************************************************************************
        * Calculate the odometry
        *******************************************************************************/
        bool calcOdometry(double diff_time);
        bool init_encoder = true;
        int last_diff_tick[WHEEL_NUM];// = {0, 0};
        double  last_rad[WHEEL_NUM];//       = {0.0, 0.0};

        /*******************************************************************************
        * Calculate the tf
        *******************************************************************************/
        void updateTF(geometry_msgs::TransformStamped& odom_tf);
        geometry_msgs::TransformStamped odom_tf;

        /*******************************************************************************
        * CalcUpdateulate the Left/Right moter speed
        *******************************************************************************/
        void setMotorControlSpeed(float spd_h, float spd_v);
        void setMotorControlSpeed(const float wheel_separation, float* value);

        /*******************************************************************************
        * Calculate usinged int imu data to singed int imu data
        *******************************************************************************/
        void getQ(float *q);
        void getValues(float * values);
        float invSqrt(float number);
        void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
        unsigned long lastUpdate, now;
        float sampleFreq;
        float q0, q1, q2, q3;
        volatile float twoKp; // 2 * proportional gain (Kp)
        volatile float twoKi; // 2 * integral gain (Ki)
        volatile float integralFBx, integralFBy, integralFBz;
        float qOrientation[4];

        /*******************************************************************************
        ** ROS Timer
        *******************************************************************************/
        ros::Timer timer;
        void timerCallback(const ros::TimerEvent& event);
        int timer_count;

};

}  // namespace robot_main_control

#endif /* robot_main_control_QNODE_HPP_ */
