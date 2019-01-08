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
typedef struct CARTESIAN_PROFILE
{
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


}cartesian_profile;
#pragma pack()

#pragma pack(4)
typedef struct JOINT_PROFILE
{
	matxx		joint_pos;  //joint real position from feedback
	matxx		joint_vel;  //joint real velocity
	matxx		joint_acc;  //joint real acceleration
	matxx		joint_pos_d;  //desired position from motion planning
	matxx		joint_vel_d;  //desired velocity from motion planning
	matxx		joint_acc_d;  //desired acceleration from motion planning

	matxx		aq;  //control law.

	double		sample_time;  //sample time :1ms




}joint_profile;
#pragma pack()

#pragma pack(4)
typedef struct DYNAMIC_PROFILE
{
	Uint32 		current_time;  //
	cartesian_profile 			cart;
	joint_profile 				joint;


	int16(*pfInitDynamicProfile)(struct DYNAMIC_PROFILE* d_prf);

}dynamic_profile;
#pragma pack()




int16 InitDynamicProfile(dynamic_profile* d_prf);







#endif /* FORCE_CONTROL_DYNAMIC_PROFILE_INCLUDE_DYNAMIC_PROFILE_H_H */

