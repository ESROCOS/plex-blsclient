/* User code: This file will not be overwritten by TASTE. */
#define _USE_MATH_DEFINES

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

static double pan = 0, tilt = 0;
static double pspeed = 0, tspeed = 0;

static double speed = 0, angle = 0;
static bool whiteLightsOn = false;
static bool UVLightsOn = false;

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

void updatePanTilt(){
#ifdef DUMMY
  // nothing to do
#else    
  if (!rover.isBridgetConnected()){     
    #ifdef DEBUG
      std::cout << "[blsclient_updatePanTilt] rover not connected" << std::endl;
    #endif 
  } else {
  // get current pan tilt

    bridgetAPI::Position p;
    rover.getPositionTelemetry(p);
//    pan = p.getPanAngle();
//    tilt = p.getTiltAngle();

  #ifdef DEBUG 
    std::cout << "[blsclient_updatePanTilt] pan: " << pan << " tilt: " << tilt << std::endl;
  #endif
  }
#endif

  // current target pan/tilt + speed * delta_t in degree
 
  pan = pan + 0.1 * pspeed;
  tilt = tilt + 0.1 * tspeed;

  pan = pan > 144.0 ? 144.0 : pan;
  pan = pan < -144.0 ? -144.0 : pan;

  tilt = tilt > 28.0 ? 28.0 : tilt;
  tilt = tilt < -28.0 ? -28.0 : tilt;
}
void updateBodyState(){

  base::Vector3d translation_(0,0,0);
  base::Quaterniond orientation_(0,0,0,1);

#ifdef DUMMY
  // get old values
  base::Vector3d translation();
  base::Quaterniond orientation();

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
      std::cout << "[blsclient_updateBodyState] rover not connected" << std::endl;
    #endif 
  } else {
  // retrieve current values

  bridgetAPI::Position p;
  rover.getPositionTelemetry(p);
  
  // assing new values
  translation_ = base::Vector3d(p.getPosX(), p.getPosY(), p.getPosZ());
  orientation_ = base::AngleAxisd(p.getPosRoll(), base::Vector3d::UnitX()) *
                 base::AngleAxisd(p.getPosPitch(), base::Vector3d::UnitY()) *
                 base::AngleAxisd(p.getPosYaw(), base::Vector3d::UnitZ());
  }
#endif
  // create new timestamp
  base::Time time = base::Time::now();

  // fill body state with current telemetry data
  asn1Scc_Vector3d_toAsn1(bs.position, translation_);
  asn1Scc_Quaterniond_toAsn1(bs.orientation, orientation_);
  asn1SccBase_Time_toAsn1(bs.time, time);

#ifdef DEBUG
  std::cout << "[blsclient_updateBodyState]\n\t" << translation_.transpose() << "\n\t" << orientation_.vec().transpose() << std::endl;
#endif
}

void blsclient_startup()
{
   init_rbs(&bs);
   #ifdef DUMMY
     // do nothing
   #else
     rover.switchControlMode();
     rover.startMovementControl();
   #endif

   #ifdef DEBUG
     std::cout << "[blsclient startup] rover connected: " << rover.isBridgetConnected()  << std::endl;
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
  // calculate new values for ackermann movement
  speed = base_mc.translation*100;
  angle = base_mc.rotation;

  // cap values to min / max
  speed = speed > 4 ? 4 : speed;
  speed = speed < -4 ? -4 : speed;

  angle = angle > 0.48? 0.48 : angle;
  angle = angle < -0.48? -0.48 : angle;
#endif
}

void blsclient_PI_clock(){
#ifdef DEBUG
  std::cout << "[blsclient_PI_clock] tick" << std::endl;
#endif

  updateBodyState();
  updatePanTilt();

  blsclient_RI_rigidBodyState(&bs);

#ifndef DUMMY
  if (!rover.isBridgetConnected()){     
    #ifdef DEBUG
      std::cout << "[blsclient_clock] rover not connected" << std::endl;
    #endif
  } else {
  
    #ifdef DEBUG
      

      bridgetAPI::Telemetry t;
      rover.getTelemetry(t);
      const int lcm = t.getLocomotionMode();
      const int bls = t.getBlsMode();

      std::cout << "[blsclient_clock] call bridget api\n"
                << "bls mode (1): " << bls << "\n"
                << "lc  mode (0): " << lcm << "\n"
                << "white lights: " << whiteLightsOn << "\n"
                << "uv lights:    " << UVLightsOn << "\n"
                << "move speed:   " << speed << " r: " << angle << "\n"
                << "target pan:   " << pan << " tilt: " << tilt << std::endl;
    #endif

    whiteLightsOn ? rover.turnWhiteLightsOn() : rover.turnWhiteLightsOff();
    UVLightsOn ? rover.turnUVLightsOn() : rover.turnUVLightsOff();
    rover.ackermannMove(speed,angle);
    rover.setCameraAngle(pan, tilt);
  }
#endif
}

void blsclient_PI_setWhiteLights(const asn1SccT_Boolean * on){

#ifdef DEBUG
    std::cout << "[blsclient_setWhiteLights] on: " << *on  << std::endl;
#endif

  whiteLightsOn = *on;
}

void blsclient_PI_setUVLights(const asn1SccT_Boolean * on){

#ifdef DEBUG
    std::cout << "[blsclient_UVLights] on: " << *on << std::endl;
#endif

  UVLightsOn = *on;
}


void blsclient_PI_pan_tilt(const asn1SccBase_commands_Joints *IN_cmd)
{

#ifdef DUMMY
  // nothing to do
#else  
  base::samples::Joints base_joints;
  asn1SccBase_commands_Joints_fromAsn1(base_joints, *IN_cmd);
    
  pspeed = base_joints.elements[0].speed * 180 / M_PI;    
  tspeed = base_joints.elements[1].speed * 180 / M_PI;
  
  #ifdef DEBUG
    std::cout << "[blsclient_PI_pan_tilt] Move pan tilt unit pan: " << pspeed << " tilt: " << tspeed << std::endl;
  #endif
#endif
}
