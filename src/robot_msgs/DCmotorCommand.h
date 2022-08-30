#ifndef _ROS_robot_msgs_DCmotorCommand_h
#define _ROS_robot_msgs_DCmotorCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot_msgs
{

  class DCmotorCommand : public ros::Msg
  {
    public:
      typedef int8_t _L_type;
      _L_type L;
      typedef int8_t _R_type;
      _R_type R;

    DCmotorCommand():
      L(0),
      R(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_L;
      u_L.real = this->L;
      *(outbuffer + offset + 0) = (u_L.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->L);
      union {
        int8_t real;
        uint8_t base;
      } u_R;
      u_R.real = this->R;
      *(outbuffer + offset + 0) = (u_R.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->R);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_L;
      u_L.base = 0;
      u_L.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->L = u_L.real;
      offset += sizeof(this->L);
      union {
        int8_t real;
        uint8_t base;
      } u_R;
      u_R.base = 0;
      u_R.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->R = u_R.real;
      offset += sizeof(this->R);
     return offset;
    }

    const char * getType(){ return "robot_msgs/DCmotorCommand"; };
    const char * getMD5(){ return "04c4156612db1058aa5bc4782a9f949e"; };

  };

}
#endif
