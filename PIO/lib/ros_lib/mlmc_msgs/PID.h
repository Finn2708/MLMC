#ifndef _ROS_mlmc_msgs_PID_h
#define _ROS_mlmc_msgs_PID_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mlmc_msgs
{

  class PID : public ros::Msg
  {
    public:
      typedef float _p_type;
      _p_type p;
      typedef float _i_type;
      _i_type i;
      typedef float _d_type;
      _d_type d;
      typedef float _ffd0_type;
      _ffd0_type ffd0;
      typedef float _ffd1_type;
      _ffd1_type ffd1;

    PID():
      p(0),
      i(0),
      d(0),
      ffd0(0),
      ffd1(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_p;
      u_p.real = this->p;
      *(outbuffer + offset + 0) = (u_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->p);
      union {
        float real;
        uint32_t base;
      } u_i;
      u_i.real = this->i;
      *(outbuffer + offset + 0) = (u_i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i);
      union {
        float real;
        uint32_t base;
      } u_d;
      u_d.real = this->d;
      *(outbuffer + offset + 0) = (u_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_d.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->d);
      union {
        float real;
        uint32_t base;
      } u_ffd0;
      u_ffd0.real = this->ffd0;
      *(outbuffer + offset + 0) = (u_ffd0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ffd0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ffd0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ffd0.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ffd0);
      union {
        float real;
        uint32_t base;
      } u_ffd1;
      u_ffd1.real = this->ffd1;
      *(outbuffer + offset + 0) = (u_ffd1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ffd1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ffd1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ffd1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ffd1);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_p;
      u_p.base = 0;
      u_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->p = u_p.real;
      offset += sizeof(this->p);
      union {
        float real;
        uint32_t base;
      } u_i;
      u_i.base = 0;
      u_i.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_i.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_i.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_i.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->i = u_i.real;
      offset += sizeof(this->i);
      union {
        float real;
        uint32_t base;
      } u_d;
      u_d.base = 0;
      u_d.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_d.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_d.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_d.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->d = u_d.real;
      offset += sizeof(this->d);
      union {
        float real;
        uint32_t base;
      } u_ffd0;
      u_ffd0.base = 0;
      u_ffd0.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ffd0.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ffd0.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ffd0.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ffd0 = u_ffd0.real;
      offset += sizeof(this->ffd0);
      union {
        float real;
        uint32_t base;
      } u_ffd1;
      u_ffd1.base = 0;
      u_ffd1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ffd1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ffd1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ffd1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ffd1 = u_ffd1.real;
      offset += sizeof(this->ffd1);
     return offset;
    }

    virtual const char * getType() override { return "mlmc_msgs/PID"; };
    virtual const char * getMD5() override { return "a76881aad551f4c46d9753b2549d471b"; };

  };

}
#endif
