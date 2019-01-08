#pragma once
/*
* dynamic_module.h
*
*  Created on: Jul 27, 2018
*      Author: hqi
*/

#ifndef FORCE_CONTROL_DYNAMIC_MODULE_INCLUDE_DYNAMIC_MODULE_H_
#define FORCE_CONTROL_DYNAMIC_MODULE_INCLUDE_DYNAMIC_MODULE_H_

#define CART_TEST

//-----------------------------------------------------------------------
#include "matrix.h"
#include "robot_ctl_utility.h"
#include "dynamic.h"
#include "control_law.h"
#include "dynamic_profile.h"


//-----------------------------------------------------------------------

typedef enum
{
	SCARA = 4



}robot_type;

typedef enum
{
	IMPEDANCE_CONTROL_CARTESIAN = 0,
	IMPEDANCE_CONTROL_JOINT = 1,



}dynamic_type;

typedef enum
{
	TRACK = 0,
	DRAG = 1,



}trajectory_mode;

#pragma pack(4)
typedef struct DYNAMIC_MODULE
{
	robot_type			type;
	trajectory_mode     trajectory_mode;  //track or drag
	dynamic_type		dynamic_type;  //dynamic control mode,example: impedance control.

	dynamic 			dynamic;		//dynamic module
	control_law			control_law;    //control law of control system
	dynamic_profile		dynamic_profile;  //cart/joint pos,vel,acc and designed pos,vel,acc

	int16(*pfInitDynaimcModule)(struct DYNAMIC_MODULE* p_dym_m);
	//int16(*pfGetTrajectory)(joint_module* m_joint, cartesian_module* m_cart, dynamic_profile* d_prof, robot_config_module* m_cfg);


}dynamic_module;
#pragma pack()

int16 InitDynaimcModule(dynamic_module* p_dym_m);
//int16 GetTrajectory(joint_module* m_joint, cartesian_module* m_cart, dynamic_profile* d_prof, robot_config_module* m_cfg);

extern dynamic_module gDymMod;

#endif /* FORCE_CONTROL_DYNAMIC_MODULE_INCLUDE_DYNAMIC_MODULE_H_ */
