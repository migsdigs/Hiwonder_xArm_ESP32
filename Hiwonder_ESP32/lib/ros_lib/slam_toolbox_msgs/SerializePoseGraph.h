#ifndef _ROS_SERVICE_SerializePoseGraph_h
#define _ROS_SERVICE_SerializePoseGraph_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace slam_toolbox_msgs
{

static const char SERIALIZEPOSEGRAPH[] = "slam_toolbox_msgs/SerializePoseGraph";

  class SerializePoseGraphRequest : public ros::Msg
  {
    public:
      typedef const char* _filename_type;
      _filename_type filename;

    SerializePoseGraphRequest():
      filename("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_filename = strlen(this->filename);
      varToArr(outbuffer + offset, length_filename);
      offset += 4;
      memcpy(outbuffer + offset, this->filename, length_filename);
      offset += length_filename;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_filename;
      arrToVar(length_filename, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_filename; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_filename-1]=0;
      this->filename = (char *)(inbuffer + offset-1);
      offset += length_filename;
     return offset;
    }

    virtual const char * getType() override { return SERIALIZEPOSEGRAPH; };
    virtual const char * getMD5() override { return "030824f52a0628ead956fb9d67e66ae9"; };

  };

  class SerializePoseGraphResponse : public ros::Msg
  {
    public:

    SerializePoseGraphResponse()
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

    virtual const char * getType() override { return SERIALIZEPOSEGRAPH; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SerializePoseGraph {
    public:
    typedef SerializePoseGraphRequest Request;
    typedef SerializePoseGraphResponse Response;
  };

}
#endif
