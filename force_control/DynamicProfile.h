#pragma once
#pragma once
/*
* dynamic_profile.h
*
*  Created on: Jul 27, 2018
*      Author: hqi
*/

#ifndef FORCE_CONTROL_DYNAMIC_PROFILE_INCLUDE_DYNAMIC_PROFILE_H_H
#define FORCE_CONTROL_DYNAMIC_PROFILE_INCLUDE_DYNAMIC_PROFILE_H_H

//-------------------------------------------------------------------------
#include "matrix.h"
#include "robot_ctl_utility.h"
//-------------------------------------------------------------------------
#pragma pack(4)
typedef struct DYNAMIC_PRF
{
	Uint32 		current_time;  //

	double		joint_pos[JOINT_COORDINATE_NUM];  //joint real position from feedback
	double		joint_vel[JOINT_COORDINATE_NUM];  //joint real velocity
	double		joint_acc[JOINT_COORDINATE_NUM];  //joint real acceleration
	double		joint_pos_d[JOINT_COORDINATE_NUM];  //desired position from motion planning
	double		joint_vel_d[JOINT_COORDINATE_NUM];  //desired velocity from motion planning
	double		joint_acc_d[JOINT_COORDINATE_NUM];  //desired acceleration from motion planning

	double		aq[JOINT_COORDINATE_NUM];  //control law.

	matxx		cart_pos;  //cartesian real position from feedback
	matxx		cart_vel;  //cartesian real velocity
	matxx		cart_acc;  //cartesian real acceleration
	matxx		cart_pos_d;  //desired position from motion planning
	matxx		cart_vel_d;  //desired velocity from motion planning
	matxx		cart_acc_d;  //desired acceleration from motion planning
	matxx		E;  //E  = cart_pos-cart_pos_d
	matxx		ax;  //control law.
	matxx		tmp21;
	matxx		temp21;
	matxx		tmp22;
	matxx		temp22;

	matxx		jacobi;	//transpos of jacobi matrix.
	matxx		inv_jacobi;	//transpos of jacobi matrix.
	matxx		jacobi_dt;	//transpos of jacobi matrix.

	double		sample_time;  //sample time :1ms




	int16(* pfInitDynamicPrf)(struct DYNAMIC_PROFILE* d_prf);

}DynamicPrf;
#pragma pack()




int16 InitDynamicPrf(DynamicPrf* D_prf);







#endif /* FORCE_CONTROL_DYNAMIC_PROFILE_INCLUDE_DYNAMIC_PROFILE_H_H */

