/* User code: This file will not be overwritten by TASTE. */

#include "blsclient.h"
#include "base_support/Base-samples-RigidBodyStateConvert.hpp"
#include <base_support/Base-commands-Motion2DConvert.hpp>
#include "base_support/OpaqueConversion.hpp"
#include <cmath>

asn1SccBase_samples_RigidBodyState bs;

void blsclient_startup()
{
   base::Vector3d translation(1.0, 0.0, 0.0);
   asn1Scc_Vector3d_toAsn1(bs.position, translation);
}

void blsclient_PI_motion_command(const asn1SccBase_commands_Motion2D *IN_mc)
{
  base::commands::Motion2D base_mc;
  base::Vector3d translation(0.0,0.0,0.0);
  base::Quaterniond orientation;

  asn1SccBase_commands_Motion2D_fromAsn1(base_mc, *IN_mc);
 
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

  // new Body state
  asn1Scc_Vector3d_toAsn1(bs.position, translation_);
  asn1Scc_Quaterniond_toAsn1(bs.orientation, orientation_);
  blsclient_RI_rigidBodyState(&bs);
}

