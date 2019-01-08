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

//--------------------------------------
//this function is called from 1ms interrupt
//--------------------------------------
int16 torque_generator(dynamic_module* p_dym_m)
{
	//int i	= 0;
	cartesian_profile* p_cart = &p_dym_m->dynamic_profile.cart;
	cartsian_impedance_control_law*	p_law = &p_dym_m->control_law.cart_law;
	//cartesian_dynamic* p_cdym								= &p_dym_m->dynamic.cart_dynamic;
	joint_dynamic* p_jdym = &p_dym_m->dynamic.joint_dynamic;
	joint_profile* p_joint = &p_dym_m->dynamic_profile.joint;
	dynamic_prm* p_prm = &p_dym_m->dynamic.dynamic_prm;


	double det = 0;
	double s1, c1, s2, c2, s12, c12;

	p_dym_m->dynamic_profile.current_time++;

	s1 = sin((*(p_joint->joint_pos.point))[0] / 180.0*3.1415926);
	s2 = sin((*(p_joint->joint_pos.point))[1] / 180.0*3.1415926);
	s12 = sin(((*(p_joint->joint_pos.point))[0] + (*(p_joint->joint_pos.point))[1]) / 180.0*3.1415926);
	c1 = cos(((*(p_joint->joint_pos.point))[0]) / 180.0*3.1415926);
	c2 = cos(((*(p_joint->joint_pos.point))[1]) / 180.0*3.1415926);
	c12 = cos(((*(p_joint->joint_pos.point))[0] + (*(p_joint->joint_pos.point))[1]) / 180.0*3.1415926);

	if (gDymMod.type == SCARA)
	{
		if (p_dym_m->dynamic_type == IMPEDANCE_CONTROL_CARTESIAN)
		{

			if (p_dym_m->trajectory_mode == DRAG)
			{
				//for test
				double data[4] = { 0,0,0,0 };
				double data1[4] = { 2,0 ,0,0 };
				matxx_assign(&p_cart->cart_acc_d, data, 4);
				matxx_assign(&p_cart->cart_vel_d, data, 4);
				matxx_assign(&p_cart->cart_pos_d, data1, 4);

				matxx_assign(&p_joint->joint_acc_d, data, 4);
				matxx_assign(&p_joint->joint_vel_d, data, 4);
				matxx_assign(&p_joint->joint_pos_d, data, 4);
				/*
				//calculate acc
				matxx_mul(&p_law->Md, &p_law->Fext, &p_cart->cart_acc_d);
				//calculate vel
				matxx_k_mac(p_cart->sample_time, &p_cart->cart_acc_d, &p_cart->cart_vel_d);
				//calculate pos
				matxx_k_mac(p_cart->sample_time, &p_cart->cart_vel_d, &p_cart->cart_pos_d);
				matxx_k_mac(0.5*p_cart->sample_time*p_cart->sample_time, &p_cart->cart_acc_d, &p_cart->cart_pos_d);
				*/
			}
			else if (p_dym_m->trajectory_mode == TRACK)
			{
				;
			}
			else
			{
				return -1;
			}
			// jacobi matrix
			(*(p_cart->jacobi.point))[0] = -p_dym_m->dynamic.dynamic_prm.L[0] * s1 - \
				p_dym_m->dynamic.dynamic_prm.L[1] * s12;

			(*(p_cart->jacobi.point + 1))[0] = -p_dym_m->dynamic.dynamic_prm.L[1] * s12;

			(*(p_cart->jacobi.point))[1] = p_dym_m->dynamic.dynamic_prm.L[0] * c1 + \
				p_dym_m->dynamic.dynamic_prm.L[1] * c12;

			(*(p_cart->jacobi.point + 1))[1] = p_dym_m->dynamic.dynamic_prm.L[1] * c12;

			// inv Jacobi of SCARA:
			matxx_inv(&p_cart->jacobi, &p_cart->inv_jacobi);

			// d jacobi/dt matrix
			(*(p_cart->jacobi_dt.point))[0] = -(*(p_joint->joint_vel.point))[0] * p_dym_m->dynamic.dynamic_prm.L[0] * \
				c1 - ((*(p_joint->joint_vel.point))[0] + \
				(*(p_joint->joint_vel.point))[1])*p_dym_m->dynamic.dynamic_prm.L[1] * c12;

			(*(p_cart->jacobi_dt.point))[1] = -(*(p_joint->joint_vel.point))[0] * p_dym_m->dynamic.dynamic_prm.L[0] * \
				s1 - ((*(p_joint->joint_vel.point))[0] + (*(p_joint->joint_vel.point))[1])*p_dym_m->dynamic.dynamic_prm.L[1] * s12;

			(*(p_cart->jacobi_dt.point + 1))[0] = -p_dym_m->dynamic.dynamic_prm.L[1] * c12*((*(p_joint->joint_vel.point))[0] + \
				(*(p_joint->joint_vel.point))[1]);

			(*(p_cart->jacobi_dt.point + 1))[1] = -p_dym_m->dynamic.dynamic_prm.L[1] * s12*((*(p_joint->joint_vel.point))[0] + \
				(*(p_joint->joint_vel.point))[1]);
			matxx_k_mult(1.0 / (180.0 / 3.1415926), &p_cart->jacobi_dt);
			//control law
			//  ax=d^2 Xd/ dt^2 - Md^-1 ( Bd*dE/dt + Kd*E + Fext )
			matxx_copy(&p_cart->cart_pos_d, &p_cart->E);
			matxx_k_mac(-1.0, &p_cart->cart_pos, &p_cart->E);
			matxx_printf(&p_cart->E);
			matxx_mul(&p_law->Bd, &p_cart->E, &p_cart->ax);

			matxx_copy(&p_cart->cart_vel_d, &p_cart->E);
			matxx_k_mac(-1.0, &p_cart->cart_vel, &p_cart->E);

			matxx_mul(&p_law->Kd, &p_cart->E, &p_cart->temp21);

			matxx_k_mac(1.0, &p_cart->temp21, &p_cart->ax);
			//matxx_k_mac(1.0, &p_law->Fext, &p_cart->ax);

			diag_inv(&p_law->Md, &p_cart->tmp22);

			matxx_mul(&p_cart->tmp22, &p_cart->ax, &p_cart->temp21);

			matxx_copy(&p_cart->cart_acc_d, &p_cart->ax);
			matxx_k_mac(1.0, &p_cart->temp21, &p_cart->ax);
			//matxx_printf(&p_cart->jacobi);
			//matxx_printf(&p_cart->inv_jacobi);
			//matxx_printf(&p_cart->jacobi_dt);
			//matxx_printf(&p_cart->ax);
			//double data[4] = { 0,0,0,0 };
			//matxx_assign(&p_cart->jacobi_dt, data, 4);
			//change ax to aq
			matxx_mul(&p_cart->inv_jacobi, &p_cart->ax, &p_joint->aq);

			matxx_mul(&p_cart->inv_jacobi, &p_cart->jacobi_dt, &p_cart->tmp22);
			matxx_mul(&p_cart->tmp22, &p_joint->joint_vel, &p_cart->tmp21);

			matxx_k_mac(-1.0, &p_cart->tmp21, &p_joint->aq);
			//matxx_k_mult(180.0 / 3.1415926, &p_joint->aq);

			//matxx_printf(&p_cart->ax);
			//-------------------------joint-----------------------------------
			int go = 1;
			if (go == 0)
			{
				matxx_copy(&p_joint->joint_pos, &p_cart->E);
				matxx_k_mac(-1.0, &p_joint->joint_pos_d, &p_cart->E);

				matxx_mul(&p_law->Bd, &p_cart->E, &p_joint->aq);

				matxx_copy(&p_joint->joint_vel, &p_cart->E);
				matxx_k_mac(-1.0, &p_joint->joint_vel_d, &p_cart->E);


				matxx_mul(&p_law->Kd, &p_cart->E, &p_cart->temp21);

				matxx_k_mac(1.0, &p_cart->temp21, &p_joint->aq);
				matxx_k_mac(1.0, &p_law->Fext, &p_joint->aq);

				diag_inv(&p_law->Md, &p_cart->tmp22);

				matxx_mul(&p_cart->tmp22, &p_joint->aq, &p_cart->temp21);

				matxx_copy(&p_joint->joint_acc_d, &p_joint->aq);
				matxx_k_mac(-1.0, &p_cart->temp21, &p_joint->aq);
			}

			//------------------------------------------------------------

			//matxx_printf(&p_joint->aq);
			//calculate torque
			//F  = Mx*ax + Cx + Gx + Fx
			//fist step :calculate Mx ,Cx ,Gx ,Fx
			(*(p_jdym->M.point))[0] = pow(p_prm->L[1], 2)*p_prm->m[1] + 2 * p_prm->L[0] * p_prm->L[1] * p_prm->m[1] * c2 + \
				pow(p_prm->L[0], 2)*(p_prm->m[0] + p_prm->m[1]);

			(*(p_jdym->M.point + 1))[0] = pow(p_prm->L[1], 2)*p_prm->m[1] + p_prm->L[0] * p_prm->L[1] * p_prm->m[1] * c2;
			(*(p_jdym->M.point))[1] = pow(p_prm->L[1], 2)*p_prm->m[1] + p_prm->L[0] * p_prm->L[1] * p_prm->m[1] * c2;
			(*(p_jdym->M.point + 1))[1] = pow(p_prm->L[1], 2)*p_prm->m[1];

			(*(p_jdym->C.point))[0] = -p_prm->L[0] * p_prm->L[1] * p_prm->m[1] * s2*pow((*(p_joint->joint_vel.point))[1], 2) - \
				2 * p_prm->L[0] * p_prm->L[1] * p_prm->m[1] * s2*(*(p_joint->joint_vel.point))[0] * (*(p_joint->joint_vel.point))[1];
			(*(p_jdym->C.point))[1] = p_prm->L[0] * p_prm->L[1] * p_prm->m[1] * s2*pow((*(p_joint->joint_vel.point))[0], 2);

			(*(p_jdym->G.point))[0] = 0;
			(*(p_jdym->G.point))[1] = 0;

			(*(p_jdym->F.point))[0] = 0;
			(*(p_jdym->F.point))[1] = 0;

			//then torque in joint.
			matxx_mul(&p_jdym->M, &p_joint->aq, &p_cart->temp21);

			matxx_copy(&p_jdym->C, &p_jdym->T);

			matxx_k_mac(1.0, &p_cart->temp21, &p_jdym->T);

			matxx_k_mac(1.0, &p_jdym->G, &p_jdym->T);

			matxx_k_mac(1.0, &p_jdym->F, &p_jdym->T);
			//matxx_printf(&p_jdym->T);
			/*
			//calculate torque need be generated by motor
			matxx_mul(&p_dym_m->dynamic.dynamic_prm.Jm, &p_dym_m->dynamic.dynamic_prm.inv_ratio, &p_cart->tmp22);
			matxx_mul(&p_cart->tmp22, &p_joint->joint_acc_d, &p_cart->tmp21);

			matxx_mul(&p_dym_m->dynamic.dynamic_prm.Bm, &p_dym_m->dynamic.dynamic_prm.inv_ratio, &p_cart->tmp22);
			matxx_mul(&p_cart->tmp22, &p_joint->joint_vel_d, &p_cart->temp21);

			matxx_k_mac(1.0, &p_cart->tmp21, &p_cart->temp21);

			matxx_mul(&p_dym_m->dynamic.dynamic_prm.inv_ratio, &p_dym_m->dynamic.joint_dynamic.T, &p_cart->tmp21);

			matxx_k_mac(1.0, &p_cart->temp21, &p_cart->tmp21);

			matxx_copy(&p_cart->tmp21, &p_dym_m->dynamic.Tm);

			//calculate the current flow through the motor
			matxx_mul(&p_dym_m->dynamic.dynamic_prm.TorCoe, &p_dym_m->dynamic.Tm, &p_dym_m->dynamic.CURm);
			*/

		}
	}



	return 0;
};













