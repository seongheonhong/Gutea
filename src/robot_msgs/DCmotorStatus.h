#ifndef _ROS_robot_msgs_DCmotorStatus_h
#define _ROS_robot_msgs_DCmotorStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot_msgs
{

  class DCmotorStatus : public ros::Msg
  {
    public:
      typedef int16_t _encoder_L_type;
      _encoder_L_type encoder_L;
      typedef int16_t _encoder_R_type;
      _encoder_R_type encoder_R;

    DCmotorStatus():
      encoder_L(0),
      encoder_R(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_encoder_L;
      u_encoder_L.real = this->encoder_L;
      *(outbuffer + offset + 0) = (u_encoder_L.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoder_L.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoder_L);
      union {
        int16_t real;
        uint16_t base;
      } u_encoder_R;
      u_encoder_R.real = this->encoder_R;
      *(outbuffer + offset + 0) = (u_encoder_R.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoder_R.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoder_R);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_encoder_L;
      u_encoder_L.base = 0;
      u_encoder_L.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoder_L.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoder_L = u_encoder_L.real;
      offset += sizeof(this->encoder_L);
      union {
        int16_t real;
        uint16_t base;
      } u_encoder_R;
      u_encoder_R.base = 0;
      u_encoder_R.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoder_R.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoder_R = u_encoder_R.real;
      offset += sizeof(this->encoder_R);
     return offset;
    }

    const char * getType(){ return "robot_msgs/DCmotorStatus"; };
    const char * getMD5(){ return "523bc11b668d11e6814cf1d850bb0422"; };

  };

}
#endif