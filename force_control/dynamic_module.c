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

#include "dynamic_module.h"



//--------------------------------------------------
dynamic_module gDymMod;

int16 InitDynaimcModule(dynamic_module *p_dym_m)
{
	p_dym_m->type = SCARA;
	p_dym_m->dynamic_type = IMPEDANCE_CONTROL_JOINT;
	p_dym_m->trajectory_mode = DRAG;

	InitDynamic(&p_dym_m->dynamic);
	InitControlLaw(&p_dym_m->control_law);
	InitDynamicProfile(&p_dym_m->dynamic_profile);



	p_dym_m->pfInitDynaimcModule = InitDynaimcModule;

	return 0;

};

/*
int16 GetTrajectory(joint_module* m_joint, cartesian_module* m_cart, dynamic_profile* d_prof, robot_config_module* m_cfg)
{
	int index;
	double old;
	//get cartesian data.
	for (index = 0; index<m_cfg->cart_dim; index++)
	{
		(*(d_prof->cart.cart_pos.point))[index] = m_cart->cart_pos_fd[index];
		(*(d_prof->cart.cart_vel.point))[index] = m_cart->cart_vel_fd[index];

		if (gRobot.robot_dev[0].module.dym_mod.trajectory_mode == TRACK)
		{
			old = (*(d_prof->cart.cart_vel_d.point))[index];

			(*(d_prof->cart.cart_vel_d.point))[index] = (m_cart->cart_pos_cmd[index] - \
				(*(d_prof->cart.cart_pos_d.point))[index]) / d_prof->cart.sample_time;

			(*(d_prof->cart.cart_acc_d.point))[index] = ((*(d_prof->cart.cart_vel_d.point))[index] - \
				old) / d_prof->cart.sample_time;

			(*(d_prof->cart.cart_pos_d.point))[index] = m_cart->cart_pos_cmd[index];
		}
	}

	//get joint data.
	for (index = 0; index<m_cfg->joint_dim; index++)
	{
		(*(d_prof->joint.joint_pos.point))[index] = m_joint[index].joint_pos_fd;
		(*(d_prof->joint.joint_vel.point))[index] = m_joint[index].joint_vel_fd;

		if (gRobot.robot_dev[0].module.dym_mod.trajectory_mode == TRACK)
		{
			old = (*(d_prof->joint.joint_vel_d.point))[index];

			(*(d_prof->joint.joint_vel_d.point))[index] = (m_joint[index].joint_pos_cmd - \
				(*(d_prof->joint.joint_pos_d.point))[index]) / d_prof->joint.sample_time;

			(*(d_prof->joint.joint_acc_d.point))[index] = ((*(d_prof->joint.joint_vel_d.point))[index] - \
				old) / d_prof->joint.sample_time;

			(*(d_prof->joint.joint_pos_d.point))[index] = m_joint[index].joint_pos_cmd;
		}

	}
	return 0;


};
*/

















