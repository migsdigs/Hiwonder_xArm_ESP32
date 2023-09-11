#ifndef _ROS_SERVICE_LoopClosure_h
#define _ROS_SERVICE_LoopClosure_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace slam_toolbox_msgs
{

static const char LOOPCLOSURE[] = "slam_toolbox_msgs/LoopClosure";

  class LoopClosureRequest : public ros::Msg
  {
    public:

    LoopClosureRequest()
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

    virtual const char * getType() override { return LOOPCLOSURE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class LoopClosureResponse : public ros::Msg
  {
    public:

    LoopClosureResponse()
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

    virtual const char * getType() override { return LOOPCLOSURE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class LoopClosure {
    public:
    typedef LoopClosureRequest Request;
    typedef LoopClosureResponse Response;
  };

}
#endif
