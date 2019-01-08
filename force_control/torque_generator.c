/*
* torque_generator.c
*
*  Created on: Jul 30, 2018
*      Author: hqi
*/

//--------------------------------------
#include "stdafx.h"
#include "type_def.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"


#include "robot_ctl_utility.h"
#include "matrix.h"

#include "torque_generator.h"

#include "dynamic_module.h"
#include "robot_config.h"
#include "robot_module.h"

extern int16 adaptive_control(dynamic* m_dym, joint_profile* p_joint, control_law* ctl_law, dynamic_prm* p_prm);

//--------------------------------------
//this function is called from 1ms interrupt
//--------------------------------------
double gTmp[6];
double gTmp1[6];
int16 torque_generator(dynamic_module* p_dym_m)
{
	extern fp14;
	int i	= 0;
	cartesian_profile* p_cart					= &p_dym_m->dynamic_profile.cart;
	cartsian_impedance_control_law*	p_law		= &p_dym_m->control_law.cart_law;

	joint_profile* p_joint						= &p_dym_m->dynamic_profile.joint;
	dynamic_prm* p_prm							= &p_dym_m->dynamic.dynamic_prm;

	double det = 0;

	double tmp;
	double s1, c1,s2,c2, s12, c12;
	//for test
	double data[4] = { 0,0,0,0 };
	double data1[4] = { 2,0 ,0,0 };
	double data1j[4]={0,0,0,0};

	p_dym_m->dynamic_profile.current_time++;

	s1 = sin((*(p_joint->joint_pos.point))[0]/180.0*3.1415926);
	s2 = sin((*(p_joint->joint_pos.point))[1] / 180.0*3.1415926);
	s12 = sin(((*(p_joint->joint_pos.point))[0] + (*(p_joint->joint_pos.point))[1]) / 180.0*3.1415926);
	c1 = cos(((*(p_joint->joint_pos.point))[0]) / 180.0*3.1415926);
	c2 = cos(((*(p_joint->joint_pos.point))[1]) / 180.0*3.1415926);
	c12= cos(((*(p_joint->joint_pos.point))[0] +(*(p_joint->joint_pos.point))[1]) / 180.0*3.1415926);
	
	if (gDymMod.type == SCARA)
	{
		p_dym_m->trajectory_mode = TRACK;//for test

		if (p_dym_m->trajectory_mode == DRAG)
		{
			matxx_assign(&p_cart->cart_acc_d, data, 4);
			matxx_assign(&p_cart->cart_vel_d, data, 4);
			matxx_assign(&p_cart->cart_pos_d, data, 4);

			matxx_assign(&p_joint->joint_acc_d, data, 4);
			matxx_assign(&p_joint->joint_vel_d, data, 4);
			matxx_assign(&p_joint->joint_pos_d, data, 4);

			
			/*
			//calculate acc
			matxx_multiply(&p_law->Md, &p_law->Fext, &p_cart->cart_acc_d);
			//calculate vel
			matxx_k_mac(p_cart->sample_time, &p_cart->cart_acc_d, &p_cart->cart_vel_d);
			//calculate pos
			matxx_k_mac(p_cart->sample_time, &p_cart->cart_vel_d, &p_cart->cart_pos_d);
			matxx_k_mac(0.5*p_cart->sample_time*p_cart->sample_time, &p_cart->cart_acc_d, &p_cart->cart_pos_d);
			*/
		}
		else if (p_dym_m->trajectory_mode == TRACK)
		{
			extern robot_module gRobotMod;
			robot_module* p_RobMod = &gRobotMod;
			for (i = 0; i < 6; i++)
			{
				(*(p_cart->cart_pos_d.point))[i]		= p_RobMod->cart.cart_pos_cmd[i];
				(*(p_joint->joint_pos_d.point))[i]		= p_RobMod->joint[i].joint_pos_cmd;

				(*(p_cart->cart_vel_d.point))[i]		= p_RobMod->cart.cart_vel_cmd[i];
				(*(p_joint->joint_vel_d.point))[i]		= p_RobMod->joint[i].joint_vel_cmd;
				
				(*(p_cart->cart_acc_d.point))[i]		= (p_RobMod->cart.cart_vel_cmd[i]-gTmp[i])/\
														  p_dym_m->dynamic_profile.cart.sample_time;
				(*(p_joint->joint_acc_d.point))[i]		= (p_RobMod->joint[i].joint_vel_cmd-gTmp1[i])/\
														  p_dym_m->dynamic_profile.joint.sample_time;
				gTmp[i] = p_RobMod->cart.cart_vel_cmd[i];
				gTmp1[i] = p_RobMod->joint[i].joint_vel_cmd;
			}
		}
		else
		{
			return -1;
		}
		//p_dym_m->dynamic_type = IMPEDANCE_CONTROL_CARTESIAN;  //for test
		p_dym_m->dynamic_type = IMPEDANCE_CONTROL_JOINT;  //for test

		if (p_dym_m->dynamic_type == IMPEDANCE_CONTROL_CARTESIAN)
		{
			// jacobi matrix
			(*(p_cart->jacobi.point))[0] = -p_dym_m->dynamic.dynamic_prm.L[0] * s1 - \
				p_dym_m->dynamic.dynamic_prm.L[1] * s12;

			(*(p_cart->jacobi.point + 1))[0] = -p_dym_m->dynamic.dynamic_prm.L[1] * s12;

			(*(p_cart->jacobi.point))[1] = p_dym_m->dynamic.dynamic_prm.L[0] * c1 + \
				p_dym_m->dynamic.dynamic_prm.L[1] * c12;

			(*(p_cart->jacobi.point + 1))[1] = p_dym_m->dynamic.dynamic_prm.L[1] * c12;

			// inv Jacobi of SCARA:
			/////////////////////////////////////////////////matxx_inv(&p_cart->jacobi, &p_cart->inv_jacobi);

			// d jacobi/dt matrix
			(*(p_cart->jacobi_dt.point))[0] = -(*(p_joint->joint_vel.point))[0] * p_dym_m->dynamic.dynamic_prm.L[0] * \
				c1*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD - ((*(p_joint->joint_vel.point))[0] + (*(p_joint->joint_vel.point))[1])*\
				p_dym_m->dynamic.dynamic_prm.L[1] * c12*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;

			(*(p_cart->jacobi_dt.point))[1] = -(*(p_joint->joint_vel.point))[0] * p_dym_m->dynamic.dynamic_prm.L[0] * \
				s1*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD - ((*(p_joint->joint_vel.point))[0] + (*(p_joint->joint_vel.point))[1])*\
				p_dym_m->dynamic.dynamic_prm.L[1] * s12*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;

			(*(p_cart->jacobi_dt.point + 1))[0] = -p_dym_m->dynamic.dynamic_prm.L[1] * c12*((*(p_joint->joint_vel.point))[0] + \
				(*(p_joint->joint_vel.point))[1])*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;

			(*(p_cart->jacobi_dt.point + 1))[1] = -p_dym_m->dynamic.dynamic_prm.L[1] * s12*((*(p_joint->joint_vel.point))[0] + \
				(*(p_joint->joint_vel.point))[1])*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;

			//control law
			//  ax=d^2 Xd/ dt^2 + Md^-1 ( Bd*dE/dt + Kd*E + Fext )
			matxx_copy(&p_cart->cart_pos_d, &p_cart->E);
			matxx_k_mac(-1.0, &p_cart->cart_pos, &p_cart->E);
			
			matxx_multiply(&p_law->Bd, &p_cart->E, &p_cart->ax);

			matxx_copy(&p_cart->cart_vel_d, &p_cart->E);
			matxx_k_mac(-1.0, &p_cart->cart_vel, &p_cart->E);

			matxx_multiply(&p_law->Kd, &p_cart->E, &p_cart->temp21);

			matxx_k_mac(1.0, &p_cart->temp21, &p_cart->ax);
			matxx_k_mac(1.0, &p_law->Fext, &p_cart->ax);

			diag_inv(&p_law->Md, &p_cart->tmp22);

			matxx_multiply(&p_cart->tmp22, &p_cart->ax, &p_cart->temp21);

			matxx_copy(&p_cart->cart_acc_d, &p_cart->ax);
			matxx_k_mac(1.0, &p_cart->temp21, &p_cart->ax);

			//change ax to aq
			//aq=J^-1*ax-J^ - 1*dJ/dt*dq/dt
			matxx_multiply(&p_cart->inv_jacobi, &p_cart->ax, &p_joint->aq);

			matxx_multiply(&p_cart->inv_jacobi, &p_cart->jacobi_dt, &p_cart->tmp22);
			matxx_multiply(&p_cart->tmp22, &p_joint->joint_vel, &p_cart->tmp21);

			matxx_k_mult(MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD, &p_cart->tmp21);

			matxx_k_mac(-1.0, &p_cart->tmp21, &p_joint->aq);
			//matxx_k_mult(MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE, &p_joint->aq);

		}
		else
		{
			//calculate aq.
			//aq=d^2 qd/ dt^2 + Bd*dE/dt + Kd*E + Text 
			matxx_copy(&p_joint->joint_pos, &p_cart->E);
			matxx_k_mac(-1.0, &p_joint->joint_pos_d, &p_cart->E);

			matxx_multiply(&p_law->Bd, &p_cart->E, &p_joint->aq);

			matxx_copy(&p_joint->joint_vel, &p_cart->E);
			matxx_k_mac(-1.0, &p_joint->joint_vel_d, &p_cart->E);

			matxx_multiply(&p_law->Kd, &p_cart->E, &p_cart->temp21);

			matxx_k_mac(1.0, &p_cart->temp21, &p_joint->aq);
			matxx_k_mac(1.0, &p_law->Fext, &p_joint->aq);

			diag_inv(&p_law->Md, &p_cart->tmp22);

			matxx_multiply(&p_cart->tmp22, &p_joint->aq, &p_cart->temp21);

			matxx_copy(&p_joint->joint_acc_d, &p_joint->aq);
			matxx_k_mac(-1.0, &p_cart->temp21, &p_joint->aq);

			matxx_k_mult(MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD, &p_joint->aq);
			
		}
		fprintf(fp14, "%lf\n", (*(p_joint->aq.point))[1]);
		//robust_control(&p_dym_m->control_law, p_joint);
		//matxx_printf(&p_joint->aq);

		//calculate torque
		//F  = Mx*ax + Cx + Gx + Fx
		//fist step :calculate M ,C ,G ,F
		
		calculate_dynamic_module(&p_dym_m->dynamic, p_joint, p_cart,p_prm);
		//adaptive_control(&p_dym_m->dynamic, p_joint, &p_dym_m->control_law, p_prm);

	}



	return 0;
};













