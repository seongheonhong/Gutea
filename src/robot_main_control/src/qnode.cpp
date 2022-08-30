/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
//#ifndef Q_MOC_RUN
#include "../include/robot_main_control/qnode.hpp"
//#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

//using namespace std;
namespace robot_main_control {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
  {

  }

QNode::~QNode()
{
  if(ros::isStarted())
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc,init_argv,"robot_main_control");
  if ( ! ros::master::check() )
  {
    return false;
  }

  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  // Add your ros communications here.
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1);
  ////////////////////////////////////////////////////////////////////
  /*********************
  ** Publisher
  **********************/
  imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10);
  cmd_vel_joy_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_joy", 10);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  joint_states_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
  navi_pub = n.advertise<geometry_msgs::PoseStamped>("sub_geometry_msgs", 10);
  motor_control_pub = n.advertise<robot_msgs::DCmotorCommand>("robot_Arduino_Control/DCmotor", 10);
  tf_pub = n.advertise<geometry_msgs::TransformStamped>("robot_tf", 10);

  /*********************
  ** Subscriber
  **********************/
  cmd_vel_sub = n.subscribe("/cmd_vel", 10, &QNode::commandVelocityCallback, this);
  encoder_sub = n.subscribe("/robot_Arduino_Status/DCmotor", 10, &QNode::encoderMsgCallback, this);
  navi_sub = n.subscribe("/pub_geometry_msgs", 10, &QNode::naviMsgCallback, this);
  openCRimu_sub = n.subscribe("/imu", 10, &QNode::openCRImuMsgCallback, this);

  /*********************
  ** ROS Timer
  **********************/
  timer_count = 0;
  initJointStates();

  q0 = q1 = q2 = 0;
  lastUpdate = 0;  
  for(int a; a<3; a++)
  {
    odom_pose[a] = 0;
    odom_vel[a] = 0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
  encoder_vel[LEFT] = 0.0;
  encoder_vel[RIGHT] = 0.0;

  start();
  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url)
{
  std::map<std::string,std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings,"robot_main_control");
  if ( ! ros::master::check() )
  {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  // Add your ros communications here.
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1);
  ////////////////////////////////////////////////////////////////////
  /*********************
  ** Publisher
  **********************/
  imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10);
  cmd_vel_joy_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_joy", 10);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  joint_states_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
  navi_pub = n.advertise<geometry_msgs::PoseStamped>("sub_geometry_msgs", 10);
  motor_control_pub = n.advertise<robot_msgs::DCmotorCommand>("robot_Arduino_Control/DCmotor", 10);
  tf_pub = n.advertise<geometry_msgs::TransformStamped>("robot_tf", 10);

  /*********************
  ** Subscriber
  **********************/
  cmd_vel_sub = n.subscribe("/cmd_vel", 10, &QNode::commandVelocityCallback, this);
  encoder_sub = n.subscribe("/robot_Arduino_Status/DCmotor", 10, &QNode::encoderMsgCallback, this);
  navi_sub = n.subscribe("/pub_geometry_msgs", 10, &QNode::naviMsgCallback, this);
  openCRimu_sub = n.subscribe("/imu", 10, &QNode::openCRImuMsgCallback, this);
  /*********************
  ** ROS Timer
  **********************/
  timer_count = 0;
  initJointStates();

  q0 = q1 = q2 = 0;
  lastUpdate = 0;
  for(int a; a<3; a++)
  {
    odom_pose[a] = 0;
    odom_vel[a] = 0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
  encoder_vel[LEFT] = 0.0;
  encoder_vel[RIGHT] = 0.0;

  start();
  return true;
}

void QNode::run()
{
  ros::Rate loop_rate(ROS_LOOP_RATE); //10ms
  int count = 0;
  int tTime[10] = { 0 };
  while ( ros::ok() )
  {
    if((timer_count - tTime[0]) >= CONTROL_INFO_UPDATE_LOOP_NUM) 
    {
	// CONTROL_MOTOR_SPEED_PERIOD //Modified by HSH
      updateGoalVelocity();
      publishCmdVelFromJoyMsg();
      tTime[0] = timer_count;
    }

    if((timer_count - tTime[1]) >= DRIVE_INFO_UPDATE_LOOP_NUM) 
    {
	//DRIVE_INFORMATION_PUBLISH_PERIOD //Modified by HSH
      updateMotorInfo(encoder_vel[LEFT], encoder_vel[RIGHT]);
      publishDriveInformation();
      tTime[1] = timer_count;
    }

    ros::spinOnce();
    loop_rate.sleep();
    timer_count ++;
  }
  Q_EMIT rosShutdown();
  // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg)
{
//  logging_model.insertRows(logging_model.rowCount(),1);
//  std::stringstream logging_model_msg;
  switch ( level )
  {
  case(Debug) :
  {
    ROS_DEBUG_STREAM(msg);
    break;
  }
  case(Info) :
  {
    ROS_INFO_STREAM(msg);
    break;
  }
  case(Warn) :
  {
    ROS_WARN_STREAM(msg);
    break;
  }
  case(Error) :
  {
    ROS_ERROR_STREAM(msg);
    break;
  }
  case(Fatal) :
  {
    ROS_FATAL_STREAM(msg);
    break;
  }
  }
//	QVariant new_row(QString(logging_model_msg.str().c_str()));
//	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
//	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::initJointStates()
{
  static char *joint_states_name[] = {"wheel_left_joint", "wheel_right_joint"};
  joint_states.name.resize(2);

  joint_states.header.frame_id    = "base_link";
  joint_states.name[0]            = joint_states_name[0];
  joint_states.name[1]            = joint_states_name[1];
}

/*******************************************************************************
** Button 
*******************************************************************************/


/*******************************************************************************
** ROS Timer callback
*******************************************************************************/
//void QNode::timerCallback(const ros::TimerEvent& event)
//{
//  timer_count ++;
//  if(timer_count == TIMER_COUNT_MAX) timer_count = 0;
//  ROS_INFO_STREAM("timer count : " << timer_count);
//}


/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void QNode::publishImuMsg()
{
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = "imu_link";

  imu_pub.publish(imu_msg);
}

/*******************************************************************************
* Publish msgs (CMD Velocity data from Joystic : angular velocity, linear velocity)
*******************************************************************************/
void QNode::publishCmdVelFromJoyMsg()
{
  robot_msgs::DCmotorCommand motor_msg;
  float wheel_velocity_cmd[2];

  float lin_vel = goal_velocity[LINEAR];
  float ang_vel = goal_velocity[ANGULAR]; //maybe ANGULAR by HSH

  wheel_velocity_cmd[LEFT]   = lin_vel - (ang_vel * WHEEL_SEPARATION / 2);
  wheel_velocity_cmd[RIGHT]  = lin_vel + (ang_vel * WHEEL_SEPARATION / 2);

  wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);

  cmd_vel_joy_msg.linear.x =  goal_velocity_from_cmd[LINEAR];
  cmd_vel_joy_msg.angular.z = goal_velocity_from_cmd[ANGULAR];

  cmd_vel_joy_pub.publish(cmd_vel_joy_msg);
}

/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void QNode::publishDriveInformation()
{
  unsigned long time_now = timer_count;
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = ros::Time::now();

  // calculate odometry  //Modified by HSH
  calcOdometry(1.0 / (double)ROS_LOOP_RATE * (double)DRIVE_INFO_UPDATE_LOOP_NUM);

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_pub.publish(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(joint_states);
}


/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void QNode::commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR]*MAX_LINEAR_VELOCITY,  (-1)*MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR]*MAX_ANGULAR_VELOCITY, (-1)*MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

  setMotorControlSpeed(WHEEL_SEPARATION, goal_velocity_from_cmd);
}

/*******************************************************************************
* Callback function for motor_power msg
*******************************************************************************/
void QNode::motorPowerCallback(const std_msgs::Bool &power_msg)
{

}


/*******************************************************************************
* Callback function for reset msg
*******************************************************************************/
void QNode::resetCallback(const std_msgs::Empty &reset_msg)
{

}

/*******************************************************************************
* Callback function for encoder msg
*******************************************************************************/
void QNode::encoderMsgCallback(const robot_msgs::DCmotorStatus &encoder_msg)
{
  encoder_vel[LEFT] += encoder_msg.encoder_L;
  encoder_vel[RIGHT] += encoder_msg.encoder_R;
}


/*******************************************************************************
* Callback function for imu msg
*******************************************************************************/
void QNode::openCRImuMsgCallback(const sensor_msgs::Imu &msg)
{
  imu_msg = msg;
}


/*******************************************************************************
* Callback function for navigation msg
*******************************************************************************/
void QNode::naviMsgCallback(const geometry_msgs::PoseStamped &msg)
{
}

/*******************************************************************************
* update function for odometry
*******************************************************************************/
void QNode::updateOdometry()
{
  odom.header.frame_id = "odom";
  odom.child_frame_id  = "base_link";

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation.x = 0;
  odom.pose.pose.orientation.y = 0;
  odom.pose.pose.orientation.z = odom_pose[2];
  odom.pose.pose.orientation.w = 0;

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.linear.y  = odom_vel[1];
  odom.twist.twist.angular.z = odom_vel[2];
}

/*******************************************************************************
* Update motor information
*******************************************************************************/
void QNode::updateMotorInfo(int left_tick, int right_tick)
{
  int current_tick = 0;

  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index]      = 0;
      last_rad[index]       = 0;

      last_velocity[index]  = 0.0;
    }

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];

//
  if (last_diff_tick[LEFT] >= TICK_GAP)
	{last_diff_tick[LEFT] -= 2*(int)(TICK_GAP);}
  else if (last_diff_tick[LEFT] <= -TICK_GAP)
	{last_diff_tick[LEFT] += 2*(int)(TICK_GAP);}
//Added by HSH

  last_tick[LEFT]      = current_tick;
  last_rad[LEFT]       += TICK2RAD * last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];

//
  if (last_diff_tick[RIGHT] >= TICK_GAP)
	{last_diff_tick[RIGHT] -= 2*(int)(TICK_GAP);}
  else if (last_diff_tick[RIGHT] <= -TICK_GAP)
	{last_diff_tick[RIGHT] += 2*(int)(TICK_GAP);}
//Added by HSH

  last_tick[RIGHT]      = current_tick;
  last_rad[RIGHT]       += TICK2RAD * last_diff_tick[RIGHT];

 if (encoder_vel[LEFT] > 2*TICK_GAP)
	encoder_vel[LEFT] -= 2*TICK_GAP;
 else if (encoder_vel[LEFT] < -2 *TICK_GAP)
	encoder_vel[LEFT] += 2*TICK_GAP;
 if (encoder_vel[RIGHT] > 2*TICK_GAP)
	encoder_vel[RIGHT] -= 2*TICK_GAP;
 else if (encoder_vel[RIGHT] < -2 *TICK_GAP)
	encoder_vel[RIGHT] += 2*TICK_GAP;
}

/*******************************************************************************
* Update the joint states
*******************************************************************************/
void QNode::updateJointStates()
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT]  = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position.resize(2);
  joint_states.velocity.resize(2);

  joint_states.position[LEFT] = joint_states_pos[LEFT];
  joint_states.position[RIGHT] = joint_states_pos[RIGHT];
  joint_states.velocity[LEFT] = joint_states_vel[LEFT];
  joint_states.velocity[RIGHT] = joint_states_vel[RIGHT];
}

/*******************************************************************************
* Update Goal Velocity
*******************************************************************************/
void QNode::updateGoalVelocity()
{
  goal_velocity[LINEAR]  = goal_velocity_from_cmd[LINEAR] ;
  goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR] ;
}

/*******************************************************************************
* Calculation function
*******************************************************************************/
float QNode::constrain(float x, float a, float b)
{
  if(x <= a) x = a;
  else  if(x >= b) x = b;

  return x;
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool QNode::calcOdometry(double diff_time)
{
  float orientation[4] ={ 0.0 };
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if (std::isnan(wheel_l))
    wheel_l = 0.0;

  if (std::isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  theta       = - atan2(0.5 - imu_msg.orientation.y * imu_msg.orientation.y - imu_msg.orientation.z * imu_msg.orientation.z,
                imu_msg.orientation.x * imu_msg.orientation.y + imu_msg.orientation.w * imu_msg.orientation.z);
//  theta       = 0.5*atan2f(imu_msg.orientation.x * imu_msg.orientation.y + imu_msg.orientation.w * imu_msg.orientation.z,
//                0.5f - imu_msg.orientation.y * imu_msg.orientation.y - imu_msg.orientation.z * imu_msg.orientation.z);
  //theta value : only by imu

  delta_theta = theta - last_theta;
  v = delta_s / step_time;
  w = delta_theta / step_time;

  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel[0] = v * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_vel[1] = v * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_vel[2] = w;

  last_theta = theta;

  return true;
}

/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
void QNode::updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

/*******************************************************************************
* CalcUpdateulate the Left/Right moter speed
*******************************************************************************/
void QNode::setMotorControlSpeed(float spd_h, float spd_v)
{
}

void QNode::setMotorControlSpeed(const float wheel_separation, float* value)
{
  robot_msgs::DCmotorCommand motor_msg;

  float wheel_velocity_cmd[2];

  float lin_vel = value[LINEAR];
  float ang_vel = value[ANGULAR];

  wheel_velocity_cmd[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity_cmd[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);

  motor_msg.L = wheel_velocity_cmd[LEFT];
  motor_msg.R = wheel_velocity_cmd[RIGHT];

  motor_control_pub.publish(motor_msg);

}

/*******************************************************************************
* Calculate usinged int imu data to singed int imu data
*******************************************************************************/
void QNode::getValues(float * values)
{
  values[0] = imu_msg.angular_velocity.x;
  values[1] = imu_msg.angular_velocity.y;
  values[2] = imu_msg.angular_velocity.z;
  values[3] = imu_msg.linear_acceleration.x;
  values[4] = imu_msg.linear_acceleration.y;
  values[5] = imu_msg.linear_acceleration.z;
  values[6] = values[7] = values[8] = 0;
}

void QNode::getQ(float *q)
{
  float val[9];
  getValues(val);

  ros::Time time = ros::Time::now();
  now = timer_count; //10ms
  sampleFreq = 1.0 / ((now - lastUpdate) / 10000.0);//20000
  lastUpdate = now;
  // gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
  // use the call below when using a 6DOF IMU
  AHRSupdate(val[0], val[1], val[2], val[3], val[4], val[5], 0, 0, 0);
  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

void QNode::AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
  float qa, qb, qc;

  // Auxiliary variables to avoid repeated arithmetic
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f))
  {
    float halfvx, halfvy, halfvz;

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex += (ay * halfvz - az * halfvy);
    halfey += (az * halfvx - ax * halfvz);
    halfez += (ax * halfvy - ay * halfvx);
  }

  // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
  if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f)
  {
    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f)
    {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx; // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

float QNode::invSqrt(float number)
{
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );

  return y;
}

}  // namespace robot_main_control
