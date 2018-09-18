/* User code: This file will not be overwritten by TASTE. */

#include "blsclient.h"
#include "base_support/Base-samples-RigidBodyStateConvert.hpp"
#include "base_support/Base-TimeConvert.hpp"
#include "base_support/Base-commands-JointsConvert.hpp"
#include <base_support/Base-commands-Motion2DConvert.hpp>
#include "base_support/OpaqueConversion.hpp"
#include <base/samples/RigidBodyState.hpp>
#include <iostream>
#include <cstring>
#include <cmath>
#include <bridgetAPI.h>

static asn1SccBase_samples_RigidBodyState bs;
static base::commands::Motion2D base_mc;
#ifdef DUMMY
  // nothing to do
#else
static bridgetAPI::BridgetAPI rover;
#endif

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

void blsclient_startup()
{
   base::Vector3d translation(1.0, 0.0, 0.0);
   init_rbs(&bs);
   asn1Scc_Vector3d_toAsn1(bs.position, translation);

   #ifdef DEBUG
     std::cout << "[blsclient startup] rover connected: " << rover.isConnected()  << std::endl;
   #endif
}

void blsclient_PI_motion_command(const asn1SccBase_commands_Motion2D *IN_mc)
{
  asn1SccBase_commands_Motion2D_fromAsn1(base_mc, *IN_mc);

#ifdef DEBUG
  std::cout << "[blsclient_PI_motion_command] " << base_mc.translation << " " << base_mc.rotation << std::endl;
#endif

#ifdef DUMMY
  // nothing to do
#else
  double speed = base_mc.translation;
  if (!rover.isBridgetConnected()){     
    #ifdef DEBUG
      std::cout << "[blsclient_PI_motion_command] rover not connected" << std::endl;
    #endif
    return;
  }

  double angle = base_mc.rotation;

  speed = speed > 0.04? 0.04 : speed;
  speed = speed < -0.04? -0.04 : speed;

  angle = angle > 0.48? 0.48 : angle;
  angle = angle < -0.48? -0.48 : angle;

  rover.startMovementControl();
  rover.ackermannMove(speed,angle);
#endif
}

void blsclient_PI_clock(){

#ifdef DEBUG
  std::cout << "[blsclient_PI_clock] tick" << std::endl;
#endif

  base::Vector3d translation_;
  base::Quaterniond orientation_;

#ifdef DUMMY

  // get old values
  base::Vector3d translation(0.0,0.0,0.0);
  base::Quaterniond orientation;

  asn1Scc_Vector3d_fromAsn1(translation, bs.position);
  asn1Scc_Quaterniond_fromAsn1(orientation, bs.orientation);
  
  const float x = translation[0]; 
  const float y = translation[1];

  const float f_orientation = base::getYaw(orientation); 

  // calculate new values

  const float x_ = base_mc.translation * cos(f_orientation);
  const float y_ = base_mc.translation * sin(f_orientation); 

  const float f_orientation_ = fmod((f_orientation + base_mc.rotation*M_PI/4.0),2*M_PI);

  // assign new values

  translation_ = base::Vector3d(x+x_,y+y_,0.0);
  orientation_ = base::Quaterniond (base::AngleAxisd(f_orientation_, base::Vector3d::UnitZ()));

#else 
  if (!rover.isBridgetConnected()){     
    #ifdef DEBUG
      std::cout << "[blsclient_PI_clock] rover not connected" << std::endl;
    #endif
    return;
  }
  // retrieve current values

  bridgetAPI::Position p;
  rover.getPositionTelemetry(p);
  
  // assing new values

  translation_ = base::Vector3d(p.getPosX(), p.getPosY(), p.getPosZ());
  orientation_ = base::AngleAxisd(p.getPosYaw(), base::Vector3d::UnitZ()) *
                 base::AngleAxisd(p.getPosPitch(), base::Vector3d::UnitY()) *
                 base::AngleAxisd(p.getPosRoll(), base::Vector3d::UnitX());

#endif

  base::Time time = base::Time::now();

  // fill new Body state
  asn1Scc_Vector3d_toAsn1(bs.position, translation_);
:q
:q
:wq
  asn1Scc_Quaterniond_toAsn1(bs.orientation, orientation_);
  asn1SccBase_Time_toAsn1(bs.time, time);

#ifdef DEBUG
  std::cout << "[blsclient_PI_clock] " << translation_.transpose() << std::endl;
#endif

  blsclient_RI_rigidBodyState(&bs);
}

void blsclient_PI_whiteLightsOn(){
#ifdef DEBUG
    std::cout << "[blsclient_whiteLightsOn] called" << std::endl;
#endif

#ifdef DUMMY
  // nothing to do
#else
  if (!rover.isBridgetConnected()){     
    #ifdef DEBUG
      std::cout << "[blsclient_whiteLightsOn] rover not connected" << std::endl;
    #endif
    return;
  }

  rover.turnWhiteLightsOn();
#endif
}

void blsclient_PI_whiteLightsOff(){
#ifdef DEBUG
    std::cout << "[blsclient_whiteLightsOff] called" << std::endl;
#endif
    
#ifdef DUMMY
  // nothing to do
#else
  if (!rover.isBridgetConnected()){     
    #ifdef DEBUG
      std::cout << "[blsclient_whiteLightsOff] rover not connected" << std::endl;
    #endif
    return;
  }

  rover.turnWhiteLightsOff();
#endif
}

void blsclient_PI_pan_tilt(const asn1SccBase_commands_Joints *IN_cmd)
{
#ifdef DEBUG
    std::cout << "[blsclient_PI_pan_tilt] Would move pan tilt unit\n";
#endif

#ifdef DUMMY
  // nothing to do
#else  
  if (!rover.isBridgetConnected()){     
    #ifdef DEBUG
      std::cout << "[blsclient_PI_pan_tilt] rover not connected" << std::endl;
    #endif
    return;
  }

  base::samples::Joints base_joints;

  double pan = 0.0;
  double tilt = 0.0;

  asn1SccBase_commands_Joints_fromAsn1(base_joints, *IN_cmd);

  pan =  base_joints.elements[0].position;
  tilt = base_joints.elements[1].position;

  pan = pan > 144.0 ? 144.0 : pan;
  pan = pan < -144.0 ? -144.0 : pan;

  tilt = tilt > 28.0 ? 28.0 : tilt;
  tilt = tilt < -28.0 ? -28.0 : tilt;

  rover.setCameraAngle(pan, tilt);
#endif
}
