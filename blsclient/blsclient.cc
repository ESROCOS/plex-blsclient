/* User code: This file will not be overwritten by TASTE. */

#include "blsclient.h"
#include "base_support/Base-samples-RigidBodyStateConvert.hpp"
#include "base_support/Base-TimeConvert.hpp"
#include <base_support/Base-commands-Motion2DConvert.hpp>
#include "base_support/OpaqueConversion.hpp"
#include <base/samples/RigidBodyState.hpp>
#include <iostream>
#include <cstring>
#include <cmath>
#include <bridgetAPI.h>

static asn1SccBase_samples_RigidBodyState bs;
static base::commands::Motion2D base_mc;

void init_rbs(asn1SccBase_samples_RigidBodyState *rbs)
{
   memset(rbs, 0, sizeof(asn1SccBase_samples_RigidBodyState));
   rbs->position.data.nCount = 3;
   rbs->cov_position.data.nCount = 9;
   rbs->orientation.im.nCount = 3;
   rbs->cov_orientation.data.nCount = 9;
   rbs->velocity.data.nCount = 3;
   rbs->cov_velocity.data.nCount = 9;
   rbs->angular_velocity.data.nCount = 3;
   rbs->cov_angular_velocity.data.nCount = 9;
}

BridgetAPI rover;

void blsclient_startup()
{
   base::Vector3d translation(1.0, 0.0, 0.0);
   init_rbs(&bs);
   asn1Scc_Vector3d_toAsn1(bs.position, translation);
   rover = BridgetAPI("127.0.0.1",1023);
}

void blsclient_PI_motion_command(const asn1SccBase_commands_Motion2D *IN_mc)
{
  asn1SccBase_commands_Motion2D_fromAsn1(base_mc, *IN_mc);
#ifdef DEBUG
  std::cout << "[blsclient motion_command] " << base_mc.translation << " " << base_mc.rotation << std::endl;
#endif
}

void blsclient_PI_clock(){

  base::Vector3d translation(0.0,0.0,0.0);
  base::Quaterniond orientation;
  // get current values
  asn1Scc_Vector3d_fromAsn1(translation, bs.position);
  asn1Scc_Quaterniond_fromAsn1(orientation, bs.orientation);
  
  float f_orientation = base::getYaw(orientation); 
  float x = translation[0]; 
  float y = translation[1];

  // calculate new values

  float x_ = base_mc.translation * cos(f_orientation);
  float y_ = base_mc.translation * sin(f_orientation); 

  base::Vector3d translation_(x+x_,y+y_,0.0);

  float f_orientation_ = fmod((f_orientation + base_mc.rotation*M_PI/4.0),2*M_PI);

  base::Quaterniond orientation_(base::AngleAxisd(f_orientation_, base::Vector3d::UnitZ()));
  base::Time time = base::Time::now();
  // new Body state
  asn1Scc_Vector3d_toAsn1(bs.position, translation_);
  asn1Scc_Quaterniond_toAsn1(bs.orientation, orientation_);
  asn1SccBase_Time_toAsn1(bs.time, time);

#ifdef DEBUG
  std::cout << "[blsclient_PI_clock] " << translation_.transpose() << std::endl;
#endif
  blsclient_RI_rigidBodyState(&bs);
}

void blsclient_PI_pan_tilt(const asn1SccBase_commands_Joints *IN_cmd)
{
#ifdef DEBUG
    std::cout << "[blsclient_PI_pan_tilt] Would move pan tilt unit\n";
#endif
}
