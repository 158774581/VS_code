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
#include "DynamicProfile.h"

//#include "DynamicModule.h"

//-------------------------------------------------------------------------------

int16 InitDynamicPrf(DynamicPrf* d_prf)
{
	int i = 0;

	d_prf->sample_time = 0.001;
	d_prf->sample_time = 0.001;

	d_prf->current_time = 0;


	matxx_malloc(&d_prf->cart_pos, 2, 1);
	matxx_malloc(&d_prf->cart_pos_d, 2, 1);
	matxx_malloc(&d_prf->cart_vel, 2, 1);
	matxx_malloc(&d_prf->cart_vel_d, 2, 1);
	matxx_malloc(&d_prf->cart_acc, 2, 1);
	matxx_malloc(&d_prf->cart_acc_d, 2, 1);
	matxx_malloc(&d_prf->E, 2, 1);
	matxx_malloc(&d_prf->ax, 2, 1);
	matxx_malloc(&d_prf->tmp21, 2, 1);
	matxx_malloc(&d_prf->temp21, 2, 1);
	matxx_malloc(&d_prf->tmp22, 2, 2);
	matxx_malloc(&d_prf->temp22, 2, 2);
	matxx_malloc(&d_prf->jacobi, 2, 2);
	matxx_malloc(&d_prf->jacobi_dt, 2, 2);
	matxx_malloc(&d_prf->inv_jacobi, 2, 2);

	matxx_malloc(&d_prf->joint_pos, 2, 1);
	matxx_malloc(&d_prf->joint_pos_d, 2, 1);
	matxx_malloc(&d_prf->joint_vel, 2, 1);
	matxx_malloc(&d_prf->joint_vel_d, 2, 1);
	matxx_malloc(&d_prf->joint_acc, 2, 1);
	matxx_malloc(&d_prf->joint_acc_d, 2, 1);
	matxx_malloc(&d_prf->aq, 2, 1);

	//GetTrajectory(&gRobot.robot_dev[i].module.joint[0], &gRobot.robot_dev[i].module.cart, \
//			&gRobot.robot_dev[i].module.dym_mod.dynamic_profile, &gRobot.robot_dev[i].cfg);

	d_prf->pfInitDynamicPrf = InitDynamicPrf;

	return 0;

};


