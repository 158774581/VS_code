/*
* robot_device.c
*
*  Created on: Jul 30, 2018
*      Author: hqi
*/
#include "stdafx.h"
#include "control_law.h"
#include "dynamic_module.h"
#include "math.h"
//---------------------------------------------

int16 InitControlLaw(control_law* p_law)
{
	int i, j;
	double data[4] = { 1, 0 ,0, 1 };
	double datap[4] = { 8, 0 ,0, 8 };
	double datad[4] = { 16, 0 ,0, 16 };
	double data0[4] = { 0, 0 ,0, 0 };

	p_law->law_type = CARTSIAN_IMPEDANCE_CONTROL_LAW;

	if (gDymMod.type == SCARA)
	{
		matxx_malloc(&p_law->cart_law.Md, 2, 2);
		matxx_malloc(&p_law->cart_law.Bd, 2, 2);
		matxx_malloc(&p_law->cart_law.Kd, 2, 2);
		matxx_malloc(&p_law->cart_law.Fext, 2, 1);

		matxx_malloc(&p_law->joint_law.Kd, 2, 2);
		matxx_malloc(&p_law->joint_law.Kp, 2, 2);
		matxx_malloc(&p_law->joint_law.Text, 2, 1);
	
		matxx_assign(&p_law->cart_law.Md, data, 4);
		matxx_assign(&p_law->cart_law.Bd, data, 4);
		matxx_assign(&p_law->cart_law.Kd, data, 4);
		matxx_assign(&p_law->cart_law.Fext, data0, 4);

		matxx_malloc(&p_law->robust.B, 2, 4);
		matxx_malloc(&p_law->robust.e, 4, 1);
		matxx_malloc(&p_law->robust.delta, 2, 1);
		matxx_malloc(&p_law->robust.P, 4, 4);
		matxx_malloc(&p_law->robust.Q, 4, 4);

		matxx_malloc(&p_law->robust.tmp24, 2, 4);
		matxx_malloc(&p_law->robust.tmp21, 2, 1);

		for (i = 0; i < 2; i++) {
			for (j = 0; j < 4; j++) {
				(*(p_law->robust.B.point + j))[i] = 0;
			}
		}
		(*(p_law->robust.B.point + 3))[0] = 1;
		(*(p_law->robust.B.point + 2))[1] = 1;
	
		(*(p_law->robust.P.point + 0))[0] = 2.9840;
		(*(p_law->robust.P.point + 1))[0] = -0.7112;
		(*(p_law->robust.P.point + 2))[0] = 1.7879;
		(*(p_law->robust.P.point + 3))[0] = 0.2637;
		(*(p_law->robust.P.point + 0))[1] = -0.7112;
		(*(p_law->robust.P.point + 1))[1] = 4.6830;
		(*(p_law->robust.P.point + 2))[1] = -0.4402;
		(*(p_law->robust.P.point + 3))[1] = 2.8063;
		(*(p_law->robust.P.point + 0))[2] = 1.7879;
		(*(p_law->robust.P.point + 1))[2] = -0.4402;
		(*(p_law->robust.P.point + 2))[2] = 4.1396;
		(*(p_law->robust.P.point + 3))[2] = 1.2300;
		(*(p_law->robust.P.point + 0))[3] = 0.2637;
		(*(p_law->robust.P.point + 1))[3] = 2.8063;
		(*(p_law->robust.P.point + 2))[3] = 1.2300;
		(*(p_law->robust.P.point + 3))[3] = 4.2754;

		(*(p_law->robust.Q.point + 0))[0] = 5.3084;
		(*(p_law->robust.Q.point + 1))[0] = -1.2300;
		(*(p_law->robust.Q.point + 2))[0] = 2.8888;
		(*(p_law->robust.Q.point + 3))[0] = -1.7188;
		(*(p_law->robust.Q.point + 0))[1] = -0.2175;
		(*(p_law->robust.Q.point + 1))[1] = 2.3184;
		(*(p_law->robust.Q.point + 2))[1] = -4.7878;
		(*(p_law->robust.Q.point + 3))[1] = 1.6985;
		(*(p_law->robust.Q.point + 0))[2] = -1.5593;
		(*(p_law->robust.Q.point + 1))[2] = 0.8230;
		(*(p_law->robust.Q.point + 2))[2] = 0.7047;
		(*(p_law->robust.Q.point + 3))[2] = 1.3842;
		(*(p_law->robust.Q.point + 0))[3] = -6.2712;
		(*(p_law->robust.Q.point + 1))[3] = 2.8433;
		(*(p_law->robust.Q.point + 2))[3] = -1.4335;
		(*(p_law->robust.Q.point + 3))[3] = 4.3146;

		p_law->robust.max_e = 0;

	}
	p_law->pfInitControlLaw = InitControlLaw;

	return 0;

};

//error now.
//by springer_handbook
int16 robust_control(control_law* ctl_law, joint_profile* p_joint)
{
	double alpha;
	double belta;
	double norm;
	double norm_e;
	double norm_k;
	double rho;
	double min;
	double tmp;

	//robust control
	(*(ctl_law->robust.e.point))[0] = ((*(p_joint->joint_pos.point))[0] - (*(p_joint->joint_pos_d.point))[0])*\
		MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	(*(ctl_law->robust.e.point))[1] = ((*(p_joint->joint_pos.point))[1] - (*(p_joint->joint_pos_d.point))[1])*\
		MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;

	(*(ctl_law->robust.e.point))[2] = ((*(p_joint->joint_vel.point))[0] - (*(p_joint->joint_vel_d.point))[0])*\
		MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	(*(ctl_law->robust.e.point))[3] = ((*(p_joint->joint_vel.point))[1] - (*(p_joint->joint_vel_d.point))[1])*\
		MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;

	matxx_multiply(&ctl_law->robust.B, &ctl_law->robust.P, &ctl_law->robust.tmp24);

	//matxx_printf(&ctl_law->robust.tmp24);

	matxx_multiply(&ctl_law->robust.tmp24, &ctl_law->robust.e, &ctl_law->robust.tmp21);

	tmp = (*(ctl_law->robust.tmp21.point))[0];
	(*(ctl_law->robust.tmp21.point))[0] = (*(ctl_law->robust.tmp21.point))[1];
	(*(ctl_law->robust.tmp21.point))[1] = tmp;
	//matxx_printf(&p_dym_m->control_law.robust.tmp21);
	norm = matxx_EuclideanNorm2(&ctl_law->robust.tmp21);

	norm_e = matxx_EuclideanNorm2(&ctl_law->robust.e);

	norm_k = 1.4142;		//norm([kp kv])
	alpha = 0.1;	//0~1
	belta = 2.0;     //joint acc_max
	min = 1.0E-5;

	rho = 21.0 / (1.0-alpha)*(alpha*belta + norm_k*norm_e);  //get rho
	tmp = -rho / norm;

	if (norm > min)
	{
		matxx_k_mult(tmp, &ctl_law->robust.tmp21);

		//matxx_printf(&ctl_law->robust.tmp21);
		matxx_k_mac(1.0, &ctl_law->robust.tmp21, &p_joint->aq);

	}
	else
	{
		matxx_k_mult(-rho / min, &ctl_law->robust.tmp21);
		matxx_k_mac(1.0, &ctl_law->robust.tmp21, &p_joint->aq);

		//	matxx_printf(&p_dym_m->control_law.robust.tmp21);
	}


	return 0;

}

int16 adaptive_control(dynamic* m_dym,joint_profile* p_joint,control_law* ctl_law,  dynamic_prm* p_prm)
{
	double s1, s2, s12, c1, c2, c12;
//	Uint8 i;
	//
	matxx_k_mult(MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE, &p_joint->aq);

	s1 = sin((*(p_joint->joint_pos.point))[0] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
	s2 = sin((*(p_joint->joint_pos.point))[1] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
	s12 = sin(((*(p_joint->joint_pos.point))[0] + (*(p_joint->joint_pos.point))[1])*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
	c1 = cos(((*(p_joint->joint_pos.point))[0])*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
	c2 = cos(((*(p_joint->joint_pos.point))[1])*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
	c12 = cos(((*(p_joint->joint_pos.point))[0] + (*(p_joint->joint_pos.point))[1])*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);

	(*(ctl_law->robust.e.point))[0] = ((*(p_joint->joint_pos.point))[0] - (*(p_joint->joint_pos_d.point))[0])*\
									  MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	(*(ctl_law->robust.e.point))[1] = ((*(p_joint->joint_pos.point))[1] - (*(p_joint->joint_pos_d.point))[1])*\
									  MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;

	(*(ctl_law->robust.e.point))[2] = ((*(p_joint->joint_vel.point))[0] - (*(p_joint->joint_vel_d.point))[0])*\
									  MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	(*(ctl_law->robust.e.point))[3] = ((*(p_joint->joint_vel.point))[1] - (*(p_joint->joint_vel_d.point))[1])*\
									  MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;

	//calculate the regressor matrix Y(q,qd,qdd)
	(*(m_dym->Y.point))[0]     = (*(p_joint->aq.point))[0]* MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	(*(m_dym->Y.point+1))[0]   = c2*(2* (*(p_joint->aq.point))[0]+ (*(p_joint->aq.point))[1])*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD-
							     s2*(pow((*(p_joint->joint_vel.point))[0] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD, 2)+ \
							     2 * (*(p_joint->joint_vel.point))[0] * (*(p_joint->joint_vel.point))[1]*\
							     pow(MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD, 2));
	(*(m_dym->Y.point+2))[0]   = (*(p_joint->aq.point))[1] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;

	(*(m_dym->Y.point+0))[1]   = 0;
	(*(m_dym->Y.point+1))[1]   = c2*(*(p_joint->aq.point))[0] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD+\
							     s2*pow((*(p_joint->joint_vel.point))[0] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD, 2);
	(*(m_dym->Y.point+2))[1]   = ((*(p_joint->aq.point))[0]+ (*(p_joint->aq.point))[1]) *\
							     MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	//the friction item
	(*(m_dym->Y.point + 3))[0] = -m_dym->dynamic_prm.fc1[0] * pow(-1, ((*(p_joint->joint_vel.point))[0] > MOTION_MODULE_CONSTANT_MIN_POSITIVE))*\
								 (1 - pow(2.718281828459, -m_dym->dynamic_prm.fc0[0] * fabs((*(p_joint->joint_vel.point))[0]))) - \
								 m_dym->dynamic_prm.fv[0] * (*(p_joint->joint_vel.point))[0] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	(*(m_dym->Y.point + 3))[1] = -m_dym->dynamic_prm.fc1[1] * pow(-1, ((*(p_joint->joint_vel.point))[1] > MOTION_MODULE_CONSTANT_MIN_POSITIVE))*\
								 (1 - pow(2.718281828459, -m_dym->dynamic_prm.fc0[1] * fabs((*(p_joint->joint_vel.point))[1]))) - \
								 m_dym->dynamic_prm.fv[1] * (*(p_joint->joint_vel.point))[1] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;

	(*(m_dym->cof.point))[0]   = p_prm->m[0] * pow(p_prm->c[0], 2) + p_prm->m[1] * (pow(p_prm->c[1], 2) + pow(p_prm->L[0], 2)) + \
							     p_prm->I[0] + p_prm->I[1];

	(*(m_dym->cof.point))[1]   = p_prm->m[1] * p_prm->c[1] * p_prm->L[0];

	(*(m_dym->cof.point))[2]   = p_prm->m[1] * pow(p_prm->c[1], 2) + p_prm->I[1];

	(*(m_dym->cof.point))[3]   = 1.0;

	//adaptive law
	//cof=cof + cofd
	//cofd=Ga*(M^-1*Y)^T*B*P*e
	////////////////////////////////////matxx_inv(&m_dym->M, &m_dym->tmp22);
	matxx_multiply(&m_dym->tmp22, &m_dym->Y, &m_dym->tmp24);
	
	matxx_transpose(&m_dym->tmp24, &m_dym->tmp42);

	matxx_multiply(&m_dym->tmp42, &ctl_law->robust.B, &m_dym->tmp44);

	matxx_multiply(&m_dym->tmp44, &ctl_law->robust.P, &m_dym->temp44);

	matxx_multiply(&m_dym->temp44, &ctl_law->robust.e, &m_dym->tmp41);

	matxx_multiply(&m_dym->Ga, &m_dym->tmp41,&m_dym->temp41);

	matxx_k_mac(-1.0, &m_dym->temp41, &m_dym->cofd);
	matxx_k_mac(1.0, &m_dym->temp41, &m_dym->cof);

	matxx_multiply(&m_dym->Y, &m_dym->cof, &m_dym->T);
	//matxx_printf(&m_dym->cof);
	return 0;
}





