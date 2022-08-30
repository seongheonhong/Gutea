#ifndef _ROS_robot_msgs_Ultrasonic_h
#define _ROS_robot_msgs_Ultrasonic_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot_msgs
{

  class Ultrasonic : public ros::Msg
  {
    public:
      typedef uint8_t _frist_type;
      _frist_type frist;
      typedef uint8_t _second_type;
      _second_type second;
      typedef uint8_t _third_type;
      _third_type third;

    Ultrasonic():
      frist(0),
      second(0),
      third(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->frist >> (8 * 0)) & 0xFF;
      offset += sizeof(this->frist);
      *(outbuffer + offset + 0) = (this->second >> (8 * 0)) & 0xFF;
      offset += sizeof(this->second);
      *(outbuffer + offset + 0) = (this->third >> (8 * 0)) & 0xFF;
      offset += sizeof(this->third);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->frist =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->frist);
      this->second =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->second);
      this->third =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->third);
     return offset;
    }

    const char * getType(){ return "robot_msgs/Ultrasonic"; };
    const char * getMD5(){ return "d554dea4722e3146474da07949201b15"; };

  };

}
#endif