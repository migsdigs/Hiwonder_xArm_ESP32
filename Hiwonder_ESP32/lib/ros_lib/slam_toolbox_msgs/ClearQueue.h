#ifndef _ROS_SERVICE_ClearQueue_h
#define _ROS_SERVICE_ClearQueue_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace slam_toolbox_msgs
{

static const char CLEARQUEUE[] = "slam_toolbox_msgs/ClearQueue";

  class ClearQueueRequest : public ros::Msg
  {
    public:

    ClearQueueRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return CLEARQUEUE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ClearQueueResponse : public ros::Msg
  {
    public:
      typedef bool _status_type;
      _status_type status;

    ClearQueueResponse():
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    virtual const char * getType() override { return CLEARQUEUE; };
    virtual const char * getMD5() override { return "3a1255d4d998bd4d6585c64639b5ee9a"; };

  };

  class ClearQueue {
    public:
    typedef ClearQueueRequest Request;
    typedef ClearQueueResponse Response;
  };

}
#endif
