#ifndef _ROS_SERVICE_SaveMap_h
#define _ROS_SERVICE_SaveMap_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/String.h"

namespace slam_toolbox_msgs
{

static const char SAVEMAP[] = "slam_toolbox_msgs/SaveMap";

  class SaveMapRequest : public ros::Msg
  {
    public:
      typedef std_msgs::String _name_type;
      _name_type name;

    SaveMapRequest():
      name()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->name.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->name.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SAVEMAP; };
    virtual const char * getMD5() override { return "0fce35bd9f5b27a63eb9b0e831759a0b"; };

  };

  class SaveMapResponse : public ros::Msg
  {
    public:

    SaveMapResponse()
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

    virtual const char * getType() override { return SAVEMAP; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SaveMap {
    public:
    typedef SaveMapRequest Request;
    typedef SaveMapResponse Response;
  };

}
#endif
