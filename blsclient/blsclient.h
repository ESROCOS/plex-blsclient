/* This file was generated automatically: DO NOT MODIFY IT ! */

/* Declaration of the functions that have to be provided by the user */

#ifndef __USER_CODE_H_blsclient__
#define __USER_CODE_H_blsclient__

#include "C_ASN1_Types.h"

#ifdef __cplusplus
extern "C" {
#endif

void blsclient_startup();

void blsclient_PI_motion_command(const asn1SccBase_commands_Motion2D *);

extern void blsclient_RI_rigidBodyState(const asn1SccBase_samples_RigidBodyState *);

#ifdef __cplusplus
}
#endif


#endif
