/*
* robot_device.c
*
*  Created on: Jul 30, 2018
*      Author: hqi
*/
#include "stdafx.h"
#include "dynamic.h"
#include "dynamic_module.h"
#include "math.h"
//-----------------------------------------------------------------------

int16 InitDynamic(dynamic* m_dym)
{
	Uint8 i = 0;

	//init mass,inertia,centroid and length.
	m_dym->dynamic_prm.m[0] = 2.247;
	m_dym->dynamic_prm.m[1] = 11.078;
	m_dym->dynamic_prm.I[0] = 0.029008;
	m_dym->dynamic_prm.I[1] = 0.079803;
	m_dym->dynamic_prm.c[0] = 0.11652;
	m_dym->dynamic_prm.c[1] = 0.1166;
	m_dym->dynamic_prm.L[0] = 0.25;
	m_dym->dynamic_prm.L[1] = 0.25;

	m_dym->dynamic_prm.fc0[0] = 1;
	m_dym->dynamic_prm.fc0[1] = 1;
	m_dym->dynamic_prm.fc1[0] = 0.05;
	m_dym->dynamic_prm.fc1[1] = 0.05;
	m_dym->dynamic_prm.fv[0] = 0.25;
	m_dym->dynamic_prm.fv[1] = 0.25;


	if (gDymMod.type == SCARA)
	{
		//init inertia,centrifugal,grivaty,and friction matrix.
		matxx_malloc(&m_dym->M, 2, 2);
		matxx_malloc(&m_dym->C, 2, 1);
		matxx_malloc(&m_dym->G, 2, 2);
		matxx_malloc(&m_dym->F, 2, 2);
		matxx_malloc(&m_dym->T, 2, 1);
		matxx_malloc(&m_dym->Tx, 2, 1);

		matxx_malloc(&m_dym->Y, 2, 4);
		matxx_malloc(&m_dym->cof, 4, 1);
		matxx_malloc(&m_dym->cofd, 4, 1);
		matxx_malloc(&m_dym->Ga, 4, 4);

		matxx_malloc(&m_dym->tmp44, 4, 4);
		matxx_malloc(&m_dym->temp44, 4, 4);
		matxx_malloc(&m_dym->tmp42, 4, 2);
		matxx_malloc(&m_dym->tmp41, 4, 1);
		matxx_malloc(&m_dym->temp41, 4, 1);
		matxx_malloc(&m_dym->tmp22, 2, 2);
		matxx_malloc(&m_dym->tmp24, 2, 4);

		(*(m_dym->cofd.point))[0] = 0;
		(*(m_dym->cofd.point))[1] = 0;
		(*(m_dym->cofd.point))[2] = 0;
		(*(m_dym->cofd.point))[3] = 0;

		(*(m_dym->Ga.point + 0))[0] = 0.4648;
		(*(m_dym->Ga.point + 1))[0] = 0.0612;
		(*(m_dym->Ga.point + 2))[0] = -0.1900;
		(*(m_dym->Ga.point + 3))[0] = -0.0142;
		(*(m_dym->Ga.point + 0))[1] = 0.0612;
		(*(m_dym->Ga.point + 1))[1] = 0.4196;
		(*(m_dym->Ga.point + 2))[1] = -0.1106;
		(*(m_dym->Ga.point + 3))[1] = -0.3110;
		(*(m_dym->Ga.point + 0))[2] = -0.1900;
		(*(m_dym->Ga.point + 1))[2] = 0.1106;
		(*(m_dym->Ga.point + 2))[2] = 0.3865;
		(*(m_dym->Ga.point + 3))[2] = -0.1721;
		(*(m_dym->Ga.point + 0))[3] = -0.0142;
		(*(m_dym->Ga.point + 1))[3] = -0.3110;
		(*(m_dym->Ga.point + 2))[3] = -0.1721;
		(*(m_dym->Ga.point + 3))[3] = 0.4884;


		matxx_malloc(&m_dym->dynamic_prm.Jm, 2, 2);
		matxx_malloc(&m_dym->dynamic_prm.Bm, 2, 2);
		matxx_malloc(&m_dym->dynamic_prm.ratio, 2, 2);
		matxx_malloc(&m_dym->dynamic_prm.inv_ratio, 2, 2);
		matxx_malloc(&m_dym->dynamic_prm.TorCoe, 2, 2);

		matxx_malloc(&m_dym->CURm, 2, 1);
		matxx_malloc(&m_dym->Tm, 2, 1);

		matxx_malloc(&m_dym->int_E, 2, 1);

		(*(m_dym->dynamic_prm.Jm.point))[0] = 117.494618E-6;
		(*(m_dym->dynamic_prm.Jm.point))[1] = 0;
		(*(m_dym->dynamic_prm.Jm.point + 1))[1] = 104.4268E-6;
		(*(m_dym->dynamic_prm.Jm.point + 1))[0] = 0;

		(*(m_dym->dynamic_prm.Bm.point))[0] = 0.05;
		(*(m_dym->dynamic_prm.Bm.point))[1] = 0;
		(*(m_dym->dynamic_prm.Bm.point + 1))[1] = 0.05;
		(*(m_dym->dynamic_prm.Bm.point + 1))[0] = 0;

		(*(m_dym->dynamic_prm.TorCoe.point))[0] = 0.524;
		(*(m_dym->dynamic_prm.TorCoe.point))[1] = 0;
		(*(m_dym->dynamic_prm.TorCoe.point + 1))[1] = 0.476;
		(*(m_dym->dynamic_prm.TorCoe.point + 1))[0] = 0;

		(*(m_dym->dynamic_prm.ratio.point))[0] = 1.0 / 80.0;
		(*(m_dym->dynamic_prm.ratio.point + 1))[0] = 0;
		(*(m_dym->dynamic_prm.ratio.point + 1))[1] = 1.0 / 50.0;
		(*(m_dym->dynamic_prm.ratio.point))[1] = 0;

		(*(m_dym->dynamic_prm.inv_ratio.point))[0] = 80.0;
		(*(m_dym->dynamic_prm.inv_ratio.point + 1))[0] = 0;
		(*(m_dym->dynamic_prm.inv_ratio.point + 1))[1] = 50.0;
		(*(m_dym->dynamic_prm.inv_ratio.point))[1] = 0;
	}

	m_dym->tor_add_flag = 0;  //torque addendum.
	m_dym->en_int_flag = 0;

	m_dym->K1 = 10.0;
	m_dym->K2 = 10.0;
	m_dym->K3 = 10.0;
	m_dym->K4 = 0.01;

	m_dym->max_tur[0] = 0.5;
	m_dym->max_tur[1] = 0.5;

	(*(m_dym->int_E.point))[0] = 0;
	(*(m_dym->int_E.point))[1] = 0;

	matxx_malloc(&m_dym->C_add, 2, 1);
	matxx_malloc(&m_dym->T_add, 2, 1);

	m_dym->pfInitDynamic = InitDynamic;

	return 0;
};



int16 calculate_dynamic_module(dynamic* m_dym, joint_profile* p_joint, cartesian_profile* p_cart, dynamic_prm* p_prm)
{

	double s1, c1, s2, c2, s12, c12;
	int i;

	s1 = sin((*(p_joint->joint_pos.point))[0] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
	s2 = sin((*(p_joint->joint_pos.point))[1] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
	s12 = sin(((*(p_joint->joint_pos.point))[0] + (*(p_joint->joint_pos.point))[1])*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
	c1 = cos(((*(p_joint->joint_pos.point))[0])*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
	c2 = cos(((*(p_joint->joint_pos.point))[1])*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
	c12 = cos(((*(p_joint->joint_pos.point))[0] + (*(p_joint->joint_pos.point))[1])*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);

	//calculate mass matrix.
	(*(m_dym->M.point))[0]		= pow(p_prm->c[0], 2)*p_prm->m[0] + p_prm->I[0] + p_prm->I[1] + \
								  p_prm->m[1] * (pow(p_prm->L[0], 2) + pow(p_prm->c[1], 2) + 2 * \
								  p_prm->L[0] * p_prm->c[1] * c2);

	(*(m_dym->M.point + 1))[0]  = p_prm->m[1] * (pow(p_prm->c[1], 2) + p_prm->L[0] * p_prm->c[1] * \
								  c2) + p_prm->I[1];
	(*(m_dym->M.point))[1]		= (*(m_dym->M.point + 1))[0];
	(*(m_dym->M.point + 1))[1]  = pow(p_prm->c[1], 2) *p_prm->m[1] + p_prm->I[1];

	//matxx_printf(&m_dym->M);
	//calculate centrifugal and coriolis force.
	(*(m_dym->C.point))[0] = pow(MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD,2)*(-p_prm->m[1] * p_prm->L[0] * p_prm->c[1] * s2)*\
		(pow((*(p_joint->joint_vel.point))[1], 2)+2* (*(p_joint->joint_vel.point))[0]* (*(p_joint->joint_vel.point))[1]);

	(*(m_dym->C.point))[1]		 = (p_prm->m[1] * p_prm->L[0] * p_prm->c[1] * s2)*\
		pow((*(p_joint->joint_vel.point))[0]*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD, 2);

	matxx_printf(&m_dym->C);

	//calculate gravity
	(*(m_dym->G.point))[0] = 0;
	(*(m_dym->G.point))[1] = 0;

	//calculate friction
	//case 1 : F = fc1*sign(vel)*(1-e^(-fc0|vel|)) + fc2*vel
	//case 2: F = fc1*sign(vel) + fc2*vel
	for (i = 0; i < 2; i++)
	{
		//case 1
		//(*(m_dym->F.point))[i] = -m_dym->dynamic_prm.fc1[i] * pow(-1, ((*(p_joint->joint_vel.point))[i] > MOTION_MODULE_CONSTANT_MIN_POSITIVE))*\
								 (1 - pow(2.718281828459, -m_dym->dynamic_prm.fc0[i] * fabs((*(p_joint->joint_vel.point))[i]))) - \
//								 m_dym->dynamic_prm.fv[i] * (*(p_joint->joint_vel.point))[i] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;

		//case 2
		(*(m_dym->F.point))[i] = -20*m_dym->dynamic_prm.fc1[i] * pow(-1, ((*(p_joint->joint_vel.point))[i] > MOTION_MODULE_CONSTANT_MIN_POSITIVE)) - \
								 m_dym->dynamic_prm.fv[i] * (*(p_joint->joint_vel.point))[i] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
		(*(m_dym->F.point))[i] = 0;
	}
	//matxx_printf(&m_dym->F);
	//then torque in joint.
	matxx_multiply(&m_dym->M, &p_joint->aq, &p_cart->temp21);

	matxx_copy(&m_dym->C, &m_dym->T);

	matxx_k_mac(1.0, &p_cart->temp21, &m_dym->T);

	matxx_k_mac(1.0, &m_dym->G, &m_dym->T);

	matxx_k_mac(1.0, &m_dym->F, &m_dym->T);

	m_dym->tor_add_flag = 0;
	//  torque addendum
	//  by An addendum on "Robust control of robots by the computed torque method"
	// T=T+T_add+C_add
	if (m_dym->tor_add_flag == 1)
	{
		//  C_add = C*(dE/dt + K1*E)
		matxx_copy(&p_joint->joint_pos_d, &p_cart->E);
		matxx_k_mac(-1.0, &p_joint->joint_pos, &p_cart->E);

		//save the E and integral it
		if (!(fabs((*(m_dym->int_E.point))[0])*m_dym->K4 > m_dym->max_tur[0]&& \
			((*(m_dym->int_E.point))[0]* (*(p_cart->E.point))[0])>0))
			(*(m_dym->int_E.point))[0] += (*(p_cart->E.point))[0]*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
		if (!(fabs((*(m_dym->int_E.point))[1])*m_dym->K4 > m_dym->max_tur[1] && \
			((*(m_dym->int_E.point))[1] * (*(p_cart->E.point))[1])>0))
			(*(m_dym->int_E.point))[1] += (*(p_cart->E.point))[1]* MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
		else
		{
			static int i=0;

			i++;
		}

		matxx_k_mult(m_dym->K1, &p_cart->E);

		matxx_copy(&p_joint->joint_vel_d, &p_cart->tmp21);
		matxx_k_mac(-1.0, &p_joint->joint_vel, &p_cart->tmp21);
		matxx_copy(&p_cart->tmp21, &m_dym->T_add);

		matxx_k_mac(1.0, &p_cart->E, &p_cart->tmp21);

		matxx_multiply(&m_dym->C, &p_cart->tmp21, &m_dym->C_add);

		matxx_k_mult(MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD, &m_dym->C_add);

		// T_add = K2*dE/dt + K3*E
		matxx_k_mult(m_dym->K2, &m_dym->T_add);

		matxx_k_mac(m_dym->K3 / m_dym->K1, &p_cart->E, &m_dym->T_add);
		matxx_k_mult(MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD, &m_dym->T_add);

		m_dym->en_int_flag = 1;
		if (m_dym->en_int_flag==1)
		{
			matxx_copy(&m_dym->int_E, &p_cart->tmp21);
			matxx_k_mac(m_dym->K4, &p_cart->tmp21, &m_dym->T_add);
		
		}

		matxx_k_mac(1.0, &m_dym->C_add, &m_dym->T);
		matxx_k_mac(1.0, &m_dym->T_add, &m_dym->T);

	}


	//calculate torque need be generated by motor
	matxx_multiply(&m_dym->dynamic_prm.Jm, &m_dym->dynamic_prm.inv_ratio, &p_cart->tmp22);
	matxx_multiply(&p_cart->tmp22, &p_joint->joint_acc_d, &p_cart->tmp21);
	matxx_k_mult(MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD, &p_cart->tmp21);

	matxx_multiply(&m_dym->dynamic_prm.Bm, &m_dym->dynamic_prm.inv_ratio, &p_cart->tmp22);
	matxx_multiply(&p_cart->tmp22, &p_joint->joint_vel_d, &p_cart->temp21);
	matxx_k_mult(MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD, &p_cart->temp21);

	matxx_k_mac(1.0, &p_cart->tmp21, &p_cart->temp21);

	matxx_multiply(&m_dym->dynamic_prm.ratio, &m_dym->T, &p_cart->tmp21);

	matxx_k_mac(1.0, &p_cart->temp21, &p_cart->tmp21);

	matxx_copy(&p_cart->tmp21, &m_dym->Tm);

	//calculate the current flow through the motor
	matxx_multiply(&m_dym->dynamic_prm.TorCoe, &m_dym->Tm, &m_dym->CURm);

	//matxx_printf(&m_dym->T);
	return 0;
}



