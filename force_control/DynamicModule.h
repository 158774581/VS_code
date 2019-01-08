#pragma once
/*
* dynamic_module.h
*
*  Created on: Aug 29, 2018
*      Author: hqi
*/

#ifndef FORCE_CONTROL_DYNAMIC_MODULE_H__
#define FORCE_CONTROL_DYNAMIC_MODULE_H__

#include "stdafx.h"
#include "robot_ctl_utility.h"
#include "type_def.h"
#include "matrix.h"
#include "DynamicProfile.h"
#include "dynamic2.h"

#pragma pack(4)
typedef struct LINK
{
	//the D-H is that o_{i} located in joint i+1 and can be see at rvctool's rne_dh.m
	Uint8			type; //the link type. 0 for rotation and 1 for prismatic,
	matxx			r;  //3*1 the centrol in {i}.
	matxx			I;  //3*3 the inertia matices.
	double			G;  //the gear ratio.
	double			Jm;  //the motor inertia (motor referred) =JL/G^2 +J.
	double			m;  //the mass the link.  Uint kg.
	double			B;  //the coef.
	double			Tc[2];  //the friction coef.
	double			qlim[2];  //the Llim and Ulim of joint.
	//the D-H.
	double			a;  //Uint m.
	double			alpha;  //Uint rad.
	double			d;  //Uint  m.
	double			theta;  //Uint rad.

}link;
#pragma pack()

#pragma pack(4)
typedef struct DYNAMIC_PRM
{
	//the D-H is that o_{i} located in joint i+1 and can be see at rvctool's rne_dh.m
	Uint8					robot_type;  //the type of robot.
	Uint8					law_type;  //0 for joint impedance control and 1 for cartesian impedance control. 
	Uint8					link_num;  //the number of robot.
	link					links[JOINT_COORDINATE_NUM];  //the link of robot.

	matxx					gravity;  //3*1 the gravity.

	double					Kv[JOINT_COORDINATE_NUM];
	double					Kd[JOINT_COORDINATE_NUM];


}DynamicPrm;

typedef struct DYNAMIC_MDL
{
	DynamicPrm				dprm;  //prm
	DynamicPrf				dprf;  //cart/joint pos,vel,acc and designed pos,vel,acc
	Dynamics			    Dyn;  //dyanmic var

	int16		(* PfinitDynamicMdl)(struct DYNAMIC_MDL* m_Dmdl);
}DynamicMdl;
#pragma pack()

int16 initDynamicMdl(DynamicMdl* m_Dmdl);

int16  NewtonEuler(Dynamics* p_dyn, DynamicPrf* p_prf, DynamicPrm* p_prm);







#endif //FORCE_CONTROL_DYNAMIC_MODULE_H__







