/*
* robot_device.c
*
*  Created on: Jul 30, 2018
*      Author: hqi
*/
#include "stdafx.h"
#include "type_def.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "robot_ctl_utility.h"
#include "matrix.h"
#include "dynamic_profile.h"

#include "dynamic_module.h"

//-------------------------------------------------------------------------------

int16 InitDynamicProfile(dynamic_profile* d_prf)
{
	int i = 0;

	d_prf->cart.sample_time = 0.001;
	d_prf->joint.sample_time = 0.001;

	d_prf->current_time = 0;

	if (gDymMod.type == SCARA)
	{
		matxx_malloc(&d_prf->cart.cart_pos, 2, 1);
		matxx_malloc(&d_prf->cart.cart_pos_d, 2, 1);
		matxx_malloc(&d_prf->cart.cart_vel, 2, 1);
		matxx_malloc(&d_prf->cart.cart_vel_d, 2, 1);
		matxx_malloc(&d_prf->cart.cart_acc, 2, 1);
		matxx_malloc(&d_prf->cart.cart_acc_d, 2, 1);
		matxx_malloc(&d_prf->cart.E, 2, 1);
		matxx_malloc(&d_prf->cart.ax, 2, 1);
		matxx_malloc(&d_prf->cart.tmp21, 2, 1);
		matxx_malloc(&d_prf->cart.temp21, 2, 1);
		matxx_malloc(&d_prf->cart.tmp22, 2, 2);
		matxx_malloc(&d_prf->cart.temp22, 2, 2);
		matxx_malloc(&d_prf->cart.jacobi, 2, 2);
		matxx_malloc(&d_prf->cart.jacobi_dt, 2, 2);
		matxx_malloc(&d_prf->cart.inv_jacobi, 2, 2);

		matxx_malloc(&d_prf->joint.joint_pos, 2, 1);
		matxx_malloc(&d_prf->joint.joint_pos_d, 2, 1);
		matxx_malloc(&d_prf->joint.joint_vel, 2, 1);
		matxx_malloc(&d_prf->joint.joint_vel_d, 2, 1);
		matxx_malloc(&d_prf->joint.joint_acc, 2, 1);
		matxx_malloc(&d_prf->joint.joint_acc_d, 2, 1);
		matxx_malloc(&d_prf->joint.aq, 2, 1);
	}

	//GetTrajectory(&gRobot.robot_dev[i].module.joint[0], &gRobot.robot_dev[i].module.cart, \
//		&gRobot.robot_dev[i].module.dym_mod.dynamic_profile, &gRobot.robot_dev[i].cfg);

	d_prf->pfInitDynamicProfile = InitDynamicProfile;

	return 0;

};


