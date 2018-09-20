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
#include <unistd.h>

static asn1SccBase_samples_RigidBodyState bs;
static base::commands::Motion2D base_mc;

// degrees
static double pan = 0, tilt = 0;

// angular speeds in deg / s 
static double pspeed = 0, tspeed = 0;

// speed in cm / s angle in radian
static double speed = 0, angle = 0;
static bool whiteLightsOn = false;
static bool UVLightsOn = false;

// seconds
const double DELTA_T = 0.1;
// centimeters
const double WHEEL_BASE = 119.0;
// centimeters
const double WHEEL_RADIUS = 15.0;

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
      
  // UNVERIFIED START 
  //
  // get current pan tilt
  
//    bridgetAPI::Position p;
//    rover.getPositionTelemetry(p);
//    pan = p.getPanAngle();
//    tilt = p.getTiltAngle();
   //UNVERIFIED END

  }
#endif

  // current target pan/tilt + speed * delta_t in degree
 
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

  bridgetAPI::Position p;
  rover.getPositionTelemetry(p);
  
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
  bridgetAPI::Telemetry t_data;
  bridgetAPI::Wheels currentSteering;
  bridgetAPI::Wheels currentSpeeds;
  rover.getTelemetry(t_data);
  w_data = t_data.getWheelData();
  s_data = t_data.getSteerData();
  currentSteering = s_data.getSteerAngleData();
  currentSpeeds = w_data.getWheelSpeedData();

  /*
  const double currentSpeed = WHEEL_RADIUS * (currentSpeeds.getMiddleRight() + currentSpeeds.getMiddleLeft()) / 2.0;
  const double currentAngle = std::abs((currentSteering.getFrontRight() + currentSteering.getFrontLeft()) / 2.0) +
                        std::abs((currentSteering.getRearRight() + currentSteering.getRearLeft()) / 2.0);
  std::cout << "[blsclient updateBodyState] current speed: " << currentSpeed << std::endl; 
  */

  const double currentSpeed = speed;
  const double currentAngle = angle;

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
   init_rbs(&bs);
   #ifdef DUMMY
     // do nothing
   #else
     // UNVERIFIED START
     while (!rover.isBridgetConnected()) {
       std::cout << "[blsclient startup] waiting for bridget to connect" << std::endl;     
       usleep(500000);
     }

     rover.startMovementControl();

     bridgetAPI::Telemetry t;
     rover.getTelemetry(t);

     if(t.getBlsMode() != 1) {
       rover.switchControlMode();
       std::cout << "[blsclient startup] switching control mode" << std::endl;
     }
      
     if(t.getLocomotionMode() != 0) {
       rover.pointTurnToggle();
       std::cout << "[blsclient startup] switching to ackermann mode" << std::endl;
     }
     // UNVERIFIED END
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
  static int phase = 0;

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
 // UNVERIFIED START 
    #ifdef DEBUG
/*      
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
*/
    #endif
   // UVLightsOn ? rover.turnUVLightsOn() : rover.turnUVLightsOff();
// UNVERIFIED END
    whiteLightsOn ? rover.turnWhiteLightsOn() : rover.turnWhiteLightsOff();

    // only send pantilt / ackermann commands phase shifted
    if(phase%2 == 0){
      rover.ackermannMove(speed,angle);
    } else {
      rover.setCameraAngle(pan, tilt);
    }

  }
#endif
  phase++;
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
