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
#include <unistd.h>
#ifdef DUMMY
  // do not include bridget API
#else
#include <bridgetAPI.h>
#endif

static asn1SccBase_samples_RigidBodyState bs;
static base::commands::Motion2D base_mc;

// degrees
static double pan = 0, tilt = 0;

// angular speeds in deg / s 
static double pspeed = 0, tspeed = 0;

// speed in cm / s angle in radian
static double speed = 0, angle = 0;

// state flags
static bool pointTurnOn = false;
static bool togglePointTurn = false;
static bool whiteLightsOn = false;
static bool UVLightsOn = false;

// seconds
const double DELTA_T = 0.05;
// centimeters
const double WHEEL_BASE = 119.0;
// centimeters
const double WHEEL_RADIUS = 15.0;

// number
const int NUMBER_OF_PHASES = 4;

#ifdef DUMMY
  // nothing to do
#else
static bridgetAPI::BridgetAPI rover;
static bridgetAPI::Telemetry latest_t;
static bridgetAPI::Position latest_p;
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
      
  // UNVERIFIED START 

  // get current pan tilt
  
    pan = latest_p.getPanAngle();
    tilt = latest_p.getTiltAngle();
   //UNVERIFIED END

  }
#endif

  // current target pan/tilt + speed * delta_t in degree * extrapolation factor
  
  pan =  pan  + DELTA_T * pspeed * 5.0;
  tilt = tilt + DELTA_T * tspeed * 5.0;

  pan = pan > 144.0 ? 144.0 : pan;
  pan = pan < -144.0 ? -144.0 : pan;

  tilt = tilt > 28.0 ? 28.0 : tilt;
  tilt = tilt < -28.0 ? -28.0 : tilt;

  #ifdef DEBUG 
    std::cout << "[blsclient_updatePanTilt] pan: " << pan << " tilt: " << tilt << std::endl;
  #endif
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
  
  const double x = translation[0]; 
  const double y = translation[1];

  const double f_orientation = base::getYaw(orientation); 

  // calculate new values
  const double x_ = base_mc.translation * cos(f_orientation);
  const double y_ = base_mc.translation * sin(f_orientation); 
  const double f_orientation_ = fmod((f_orientation + base_mc.rotation*M_PI/4.0),2*M_PI);

  // assign new values
  translation_ = base::Vector3d(x+x_,y+y_,0.0);
  orientation_ = base::Quaterniond (base::AngleAxisd(f_orientation_, base::Vector3d::UnitZ()));

#else 
  // UNVERIFIED START
  if (!rover.isBridgetConnected()){     
    #ifdef DEBUG
      std::cout << "[blsclient_updateBodyState] rover not connected" << std::endl;
    #endif
  } else {
  /*
  // retrieve current values

  
  // assing new values
  translation_ = base::Vector3d(p.getPosX(), p.getPosY(), p.getPosZ());
  orientation_ = base::AngleAxisd(p.getPosRoll(), base::Vector3d::UnitX()) *
                 base::AngleAxisd(p.getPosPitch(), base::Vector3d::UnitY()) *
                 base::AngleAxisd(p.getPosYaw(), base::Vector3d::UnitZ());
  */
  // UNVERIFIED END

  // THIS CODE APPROXIMATES THE ACKERMANN KINEMATIC 
  // note: this is because at the time of testing there
  // was no odometry/imu data available on the test
  // system

  bridgetAPI::WheelData w_data;
  bridgetAPI::SteerData s_data;
  bridgetAPI::Wheels currentSteering;
  bridgetAPI::Wheels currentSpeeds;
  w_data = latest_t.getWheelData();
  s_data = latest_t.getSteerData();
  currentSteering = s_data.getSteerAngleData();
  currentSpeeds = w_data.getWheelSpeedData();
 
  const double currentSpeed = WHEEL_RADIUS * (currentSpeeds.getMiddleRight() + currentSpeeds.getMiddleLeft()) / 2.0;
  const double currentAngle = std::abs((currentSteering.getFrontRight() + currentSteering.getFrontLeft()) / 2.0) +
                        std::abs((currentSteering.getRearRight() + currentSteering.getRearLeft()) / 2.0);
#ifdef DEBUG
  std::cout << "[blsclient updateBodyState] current speed: " << currentSpeed << std::endl; 
#endif

  //const double currentSpeed = speed;
  //const double currentAngle = angle;

  // Ackermann Steering equation
  // The testing system employs double ackermann steering,

  const double angular_v = currentSpeed * std::tan(currentAngle) / WHEEL_BASE;


  base::Vector3d translation(0,0,0);
  base::Quaterniond orientation(0,0,0,1);

  asn1Scc_Vector3d_fromAsn1(translation, bs.position);
  asn1Scc_Quaterniond_fromAsn1(orientation, bs.orientation);
  
  const double x = translation[0]; 
  const double y = translation[1];

  const double f_orientation = base::getYaw(orientation); 

  // calculate new values
  const double x_ = currentSpeed/100.0 * cos(f_orientation) * DELTA_T;
  const double y_ = currentSpeed/100.0 * sin(f_orientation) * DELTA_T; 

  const double f_orientation_ = fmod((f_orientation + angular_v * DELTA_T),2*M_PI);

  // assign new values
  translation_ = base::Vector3d(x+x_,y+y_,0.0);
  orientation_ = base::Quaterniond (base::AngleAxisd(f_orientation_, base::Vector3d::UnitZ()));
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
   std::cout << "[blsclient_startup] startup " << std::endl;
#ifdef DEBUG
   std::cout << "[blsclient_startup] DEBUG mode" << std::endl;
#endif
   init_rbs(&bs);
   #ifdef DUMMY
     // do nothing
   #else
     std::cout << "[blsclient startup] trying to connect with rover" << std::endl;
     while (!rover.isBridgetConnected()) {
       std::cout << "[blsclient startup] waiting for bridget to connect" << std::endl;     
       usleep(500000);
     }
     std::cout << "[blsclient startup] bridget connected!" << std::endl;

#ifdef DEBUG
   std::cout << "[blsclient_startup] start movement control" << std::endl;
#endif
     rover.startMovementControl();
#ifdef DEBUG
   std::cout << "[blsclient_startup] started movement control" << std::endl;
#endif
#ifdef DEBUG
   std::cout << "[blsclient_startup] try to get telemetry" << std::endl;
#endif
     rover.getTelemetry(latest_t);
#ifdef DEBUG
   std::cout << "[blsclient_startup] got telemetry" << std::endl;
#endif

     if(latest_t.getBlsMode() != 1) {
       std::cout << "[blsclient startup] switching control mode" << std::endl;
       rover.switchControlMode();
     }
      
     if(latest_t.getLocomotionMode() != 0) {
       std::cout << "[blsclient startup] switching to ackermann mode" << std::endl;
       rover.pointTurnToggle();
     }
   #endif
     std::cout << "[blsclient startup] startup finished" << std::endl;
}

void blsclient_PI_motion_command(const asn1SccBase_commands_Motion2D *IN_mc)
{
#ifdef DEBUG
    std::cout << "[blsclient_PI_motion_command] !" << std::endl;
#endif
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
  static int phase = 0;

#ifdef DEBUG
  std::cout << "[blsclient_PI_clock] tick" << std::endl;
#endif

#ifndef DUMMY
  if (!rover.isBridgetConnected()){     
    #ifdef DEBUG
      std::cout << "[blsclient_clock] rover not connected" << std::endl;
    #endif
  } else {

    // only send pantilt / ackermann commands phase shifted
    switch (phase){
    case 0: // telemetry
 // UNVERIFIED START 
         
      rover.getPositionTelemetry(latest_p);
      rover.getTelemetry(latest_t);

    #ifdef DEBUG
      {
        const int lcm = latest_t.getLocomotionMode();
        const int bls = latest_t.getBlsMode();
      
        std::cout << "[blsclient_clock] call bridget api\n"
                  << "bls mode (1): " << bls << "\n"
                  << "lc  mode (0): " << lcm << "\n"
                  << "white lights: " << whiteLightsOn << "\n"
                  << "uv lights:    " << UVLightsOn << "\n"
                  << "move speed:   " << speed << " r: " << angle << "\n"
                  << "target pan:   " << pan << " tilt: " << tilt << std::endl;
      }
    #endif
// UNVERIFIED END
      break;
    case 1: // movement
      if (pointTurnOn){
        rover.pointTurnMove(angle);
      } else {
        rover.ackermannMove(speed,angle);
      }
      break;
    case 2: // toggle flags
         whiteLightsOn ? rover.turnWhiteLightsOn() : rover.turnWhiteLightsOff();
      // UVLightsOn ? rover.turnUVLightsOn() : rover.turnUVLightsOff();
         if(togglePointTurn) {
             rover.pointTurnToggle();
             // race conditions could arise when the PI is called in between the line above and below
             pointTurnOn = !pointTurnOn;
             togglePointTurn = false;
         }
      break;
    case 3: // pan/tilt
      rover.setCameraAngle(pan, tilt);
      break;
    default:
      break;
    }
  }
#endif

  updateBodyState();
  updatePanTilt();
  blsclient_RI_rigidBodyState(&bs);

  phase = (phase + 1)%NUMBER_OF_PHASES;
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

void blsclient_PI_setPointTurn(const asn1SccT_Boolean * on){
#ifdef DEBUG
    std::cout << "[blsclient_setPointTurn] on: " << *on << std::endl;
#endif

  if(*on != pointTurnOn) {
      togglePointTurn = true;
  } else {
      togglePointTurn = false;
  }
}

void blsclient_PI_pan_tilt(const asn1SccBase_commands_Joints *IN_cmd)
{
#ifdef DEBUG
    std::cout << "[blsclient_PI_pan_tilt] !" << std::endl;
#endif

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
