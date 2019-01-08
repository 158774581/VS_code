/*
* dynamic_module.h
*
*  Created on: Sep 3, 2018
*      Author: hqi
*/

#ifndef FORCE_CONTROL_ROBOT_DYNAMIC_INCLUDE_DYNAMIC2_H_
#define FORCE_CONTROL_ROBOT_DYNAMIC_INCLUDE_DYNAMIC2_H_

#include "matrix.h"
#include "type_def.h"

#pragma pack(4)
typedef struct DYNAMICS
{
	//calculate when init.
	matxx					pstar[JOINT_COORDINATE_NUM];	//0_{i-1} to 0_{i} in {i}.
	matxx					R[JOINT_COORDINATE_NUM];  //rotation matrix {i-1} to {i}.
	matxx					Rt[JOINT_COORDINATE_NUM];  //Rt=R'.

	matxx					Fm[JOINT_COORDINATE_NUM];  //Resultant force on link.
	matxx					Nm[JOINT_COORDINATE_NUM];  //Resultant torque on link
	matxx					torque;  //n*1 ,4*1 for scara.

	matxx					fext;  //3*1 the force from outside {fx fy fz }.
	matxx					Text;  //3*1 the torque from outside {nx ny nz}.
	matxx					wd;  //3*1  the angular acceleration.
	matxx					w;  //3*1  the angular velocity.
	matxx					vd;  //3*1  the acceleration.
	matxx					vcd;  //3*1  the acceleration of centriol
	matxx					f;  //3*1  the force
	matxx					T;  //3*1  the torque.

	matxx					tmp31;
	matxx					temp31;
	matxx					tp31;

	matxx					A;  //4*4  the Link transformation matrices
	matxx					U1;  //4*4 tmp
	matxx					U2;  //4*4  tmp;

	int16	( *PfinitDynamics)(struct DYNAMICS *p_dyn);
}Dynamics;
#pragma pack()

int16  InitDynamics(Dynamics* p_dyn);

#endif // FORCE_CONTROL_ROBOT_DYNAMIC_INCLUDE_DYNAMIC2_H_
