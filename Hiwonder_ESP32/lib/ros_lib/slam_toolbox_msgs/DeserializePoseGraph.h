#ifndef _ROS_SERVICE_DeserializePoseGraph_h
#define _ROS_SERVICE_DeserializePoseGraph_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose2D.h"

namespace slam_toolbox_msgs
{

static const char DESERIALIZEPOSEGRAPH[] = "slam_toolbox_msgs/DeserializePoseGraph";

  class DeserializePoseGraphRequest : public ros::Msg
  {
    public:
      typedef const char* _filename_type;
      _filename_type filename;
      typedef int8_t _match_type_type;
      _match_type_type match_type;
      typedef geometry_msgs::Pose2D _initial_pose_type;
      _initial_pose_type initial_pose;
      enum { UNSET =  0 };
      enum { START_AT_FIRST_NODE =  1 };
      enum { START_AT_GIVEN_POSE =  2 };
      enum { LOCALIZE_AT_POSE =  3 };

    DeserializePoseGraphRequest():
      filename(""),
      match_type(0),
      initial_pose()
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
      union {
        int8_t real;
        uint8_t base;
      } u_match_type;
      u_match_type.real = this->match_type;
      *(outbuffer + offset + 0) = (u_match_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->match_type);
      offset += this->initial_pose.serialize(outbuffer + offset);
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
      union {
        int8_t real;
        uint8_t base;
      } u_match_type;
      u_match_type.base = 0;
      u_match_type.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->match_type = u_match_type.real;
      offset += sizeof(this->match_type);
      offset += this->initial_pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return DESERIALIZEPOSEGRAPH; };
    virtual const char * getMD5() override { return "29a9bb432c3daccc49d63131eece4576"; };

  };

  class DeserializePoseGraphResponse : public ros::Msg
  {
    public:

    DeserializePoseGraphResponse()
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

    virtual const char * getType() override { return DESERIALIZEPOSEGRAPH; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class DeserializePoseGraph {
    public:
    typedef DeserializePoseGraphRequest Request;
    typedef DeserializePoseGraphResponse Response;
  };

}
#endif
