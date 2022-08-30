#include <ros/ros.h>
#include <sound_play/sound_play.h>


int main(int argc, char ** argv) 
{
  ros::init(argc, argv, "example");

  ros::NodeHandle nh;
  sound_play::SoundClient sc;

  usleep(1000000);
  sc.playWave("/home/rastech/catkin_ws/src/robot_sound/sounds/output.wav");
  usleep(1000000);
  ros::spin();
}
