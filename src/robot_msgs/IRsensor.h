#ifndef _ROS_robot_msgs_IRsensor_h
#define _ROS_robot_msgs_IRsensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot_msgs
{

  class IRsensor : public ros::Msg
  {
    public:
      typedef bool _IR_sensor_type;
      _IR_sensor_type IR_sensor;

    IRsensor():
      IR_sensor(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_IR_sensor;
      u_IR_sensor.real = this->IR_sensor;
      *(outbuffer + offset + 0) = (u_IR_sensor.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->IR_sensor);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_IR_sensor;
      u_IR_sensor.base = 0;
      u_IR_sensor.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->IR_sensor = u_IR_sensor.real;
      offset += sizeof(this->IR_sensor);
     return offset;
    }

    const char * getType(){ return "robot_msgs/IRsensor"; };
    const char * getMD5(){ return "d4f8c122e8e332d40b29b13f809cae31"; };

  };

}
#endif