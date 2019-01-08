/*
* lib_scara.c
*
*  Created on: Mar 21, 2018
*      Author: root
*/

#include "stdafx.h"
#include "type_def.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "matrix.h"
#include "robot_ctl_utility.h"
#include "robot_config.h"
#include "cartesian_module.h"
#include "joint_module.h"
#include "kinematics.h"
#include "lib_scara.h"

//----------------------------------------------------------------------------
//some parameter input maybe not used ,buf save them for other robot type,because we want the same function pointer
// forward kinematics of scara robot
//----------------------------------------------------------------------------
int16 ScaraFK(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config, double* pose, double* joint)
{
	//----------------------------------------------------------------------------
	double theta1 = 0.0;
	double theta2 = 0.0;
	double X = 0.0;
	double Y = 0.0;
	double Z = 0.0;
	double L1 = 0.0;
	double L2 = 0.0;
	//double L3 = 0.0;
	//----------------------------------------------------------------------------

	//mechanical method
	/* X = L1 * cos(theta1) + L2 * cos(theta1 + theta2)
	* Y = L1 * sin(theta1) + L2 * sin(theta1 + theta2)
	* */

	//unit change from degree to rad
	theta1 = joint[0] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD; //degree -->rad
	theta2 = joint[1] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	//----------------------------------------------------------------------------

	Z = joint[2];  //translation mm
	L1 = m_kine->prm.lenth_of_joint[0];
	L2 = m_kine->prm.lenth_of_joint[1];

	X = cos(theta1) * L1 + cos(theta1 + theta2) * L2;
	Y = sin(theta1) * L1 + sin(theta1 + theta2) * L2;
	//----------------------------------------------------------------------------

	pose[0] = X; /* mm */
	pose[1] = Y; /* mm */
	pose[2] = Z; /* mm */
				 //this is pose ,for scara we just have rotate around z axis
	pose[3] = 0;
	pose[4] = 0;
	pose[5] = joint[3]; //rotation unit :degree
						//----------------------------------------------------------------------------

	return 0;
}
//----------------------------------------------------------------------------
//some parameter input maybe not used ,buf save them for other robot type,because we want the same function pointer
// inverse kinematics of scara robot
//----------------------------------------------------------------------------
int16 ScaraIK(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config, double* pose, double* joint, Uint8 mode)
{
	double d_temp = 0.0;
	double X = 0.0; 					/* mm */
	double Y = 0.0; 					/* mm */
	double Z = 0.0; 					/* mm */

	double S = 0.0;
	double SS = 0.0;
	double L1 = 0.0;
	double L1L1 = 0.0;
	double L2 = 0.0;
	double L2L2 = 0.0;
	double theta1 = 0.0;
	double theta2 = 0.0;

	/* SS = X^2 + Y^2;
	*
	* case 1.Right Shouldered Configuration
	* theta1 = atan2(Y,X) - acos((S^2 + L1^2 - L2^2)/2/S/L1);
	* theta2 = pi - acos((L1^2 + L2^2 - S^2)/2/L1/L2);
	*
	* case 2.Left Shouldered Configuration
	* theta1 = atan2(Y,X) + acos((S^2 + L1^2 - L2^2)/2/S/L1);
	* theta2 = acos((L1^2 + L2^2 - S^2)/2/L1/L2) - pi;
	*
	* */
	X = pose[0];
	Y = pose[1];
	Z = pose[2];

	L1 = m_kine->prm.lenth_of_joint[0];
	L2 = m_kine->prm.lenth_of_joint[1];

	SS = X * X + Y * Y;
	//--------------------------------------
	// this situation can be avoided by limit
	// 1. special case: S <= 0;
	// this place do not do fabs,because we do square
	d_temp = (L1 - L2);
	d_temp *= d_temp;

	if (SS <= d_temp)
	{ 	// S <= |L1 - L2|
		joint[0] = 0.0;
		joint[1] = 180.0;
		joint[2] = Z;
		joint[3] = pose[5]; //degree
							//joint[4] = pose[4]; //degree
							//joint[5] = pose[3]; //degree
		return -1;
	}
	//--------------------------------------
	// 2. special case: S >= L1 + L2;
	d_temp = (L1 + L2);
	d_temp *= d_temp;

	if (SS >= d_temp)
	{ // = 0.0
		joint[0] = (atan2(Y, X)*MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE);
		joint[1] = 0.0;
		joint[2] = Z;
		joint[3] = pose[5]; //degree
							//joint[4] = pose[4]; //degree
							//joint[5] = pose[3]; //degree
		return -2;
	}
	//--------------------------------------
	//cosine law
	S = sqrt(SS);
	L1L1 = L1 * L1;
	L2L2 = L2 * L2;
	d_temp = 2.0 * S * L1;
	d_temp = acos((SS + L1L1 - L2L2) / d_temp);

	if (mode == 0)
	{
		switch (p_config->locat_tmp)
		{
		case KINEMATICS_LIBRARY_CONFIG_LEFTY:
		{
			theta1 = atan2(Y, X) + d_temp;
			d_temp = 2.0 * L1 * L2;
			d_temp = acos((L1L1 + L2L2 - SS) / d_temp);
			theta2 = d_temp - MOTION_MODULE_CONSTANT_PI;
			break;
		}
		case KINEMATICS_LIBRARY_CONFIG_RIGHTY:
		default:
		{
			theta1 = atan2(Y, X) - d_temp;
			d_temp = 2.0 * L1 * L2;
			d_temp = acos((L1L1 + L2L2 - SS) / d_temp);
			theta2 = MOTION_MODULE_CONSTANT_PI - d_temp;
			break;
		}
		}
	}
	else
	{
		//--------------------------------------
		//according the hand configuration
		switch (p_config->prm.location_config)
		{
		case KINEMATICS_LIBRARY_CONFIG_LEFTY:
		{
			theta1 = atan2(Y, X) + d_temp;
			d_temp = 2.0 * L1 * L2;
			d_temp = acos((L1L1 + L2L2 - SS) / d_temp);
			theta2 = d_temp - MOTION_MODULE_CONSTANT_PI;
			break;
		}
		case KINEMATICS_LIBRARY_CONFIG_RIGHTY:
		default:
		{
			theta1 = atan2(Y, X) - d_temp;
			d_temp = 2.0 * L1 * L2;
			d_temp = acos((L1L1 + L2L2 - SS) / d_temp);
			theta2 = MOTION_MODULE_CONSTANT_PI - d_temp;
			break;
		}
		}
	}

	//----------------------------------------------------------------
	joint[0] = theta1 * MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE; // rad  to degree
	joint[1] = theta2 * MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE; // rad  to degree
	joint[2] = Z;  												   // mm
	joint[3] = pose[5]; // degree
						//joint[4] = pose[4]; // degree
						//joint[5] = pose[3]; // degree

	return 0;
}
//----------------------------------------------------------------------------
int16 ScaraBasicIJ(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config, const double* pose, double* joint)
{
	//---------------------------------------------------------------------------
	double L1 = 0.0;
	double L2 = 0.0;
	double theta1 = 0.0;
	double theta2 = 0.0;
	double S1 = 0.0;
	double S12 = 0.0;
	double C1 = 0.0;
	double C12 = 0.0;
	double DET = 0.0;
	//---------------------------------------------------------------------------
	theta1 = m_joint[0].joint_pos_cmd * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	theta2 = m_joint[1].joint_pos_cmd * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;

	L1 = m_kine->prm.lenth_of_joint[0];
	L2 = m_kine->prm.lenth_of_joint[1];

	// Simplify!
	// Jacobn of SCARA:
	// [
	//		L2*C12/DET    L2*S12/DET    0    0
	// 	-(L1*C1+L2*C12)/DET -(L1*S1+L2*S12)/DET 0 0
	// 	0    0    1    0
	// 	0    0    0    1
	// ]

	S1 = sin(theta1);
	C1 = cos(theta1);
	S12 = sin(theta1 + theta2);
	C12 = cos(theta1 + theta2);
	DET = L1 * L2 * sin(theta2);
	//-----------------------------------------------------------------
	// When the inverse Jacobn does not exist!
	if (fabs(DET) < MOTION_MODULE_CONSTANT_MIN_POSITIVE)
	{
		joint[0] = sqrt(pose[0] * pose[0] + pose[1] * pose[1]) / (L1 + L2)* MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
		joint[1] = 0.0;
		joint[2] = pose[2];
		joint[3] = pose[5]; // degree
							//joint[4] = pose[4]; // degree
							//joint[5] = pose[3]; // degree
		return -1;
		//if the det can not satisfy the condition ,we should into emergency stop
	}
	//-----------------------------------------------------------------
	// There is the inverse Jacobn
	DET = 1.0 / DET;

	joint[0] = L2 * DET * (C12 * pose[0] + S12 * pose[1]) * MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
	joint[1] = -DET * ((L1 * C1 + L2 * C12) * pose[0] + (L1 * S1 + L2 * S12) * pose[1]) *MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
	joint[2] = pose[2];
	joint[3] = pose[5]; // degree
						//joint[4] = pose[4]; // degree
						//joint[5] = pose[3]; // degree

	return 0;
}

int16 ScaraIJ(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config, double* pose, double* joint)
{

	Uint8 cart_id = 0;
	Uint8 joint_id = 0;
	int16 rtn;

	double cal_vel_vector[6];
	double joint_vel_vector[6];

	//-----------------------------------------------------------------
	// assign pose velocity use cart vel command
	for (cart_id = 0; cart_id < p_config->cart_dim; cart_id++)
	{
		cal_vel_vector[cart_id] = pose[cart_id];
	}
	//----------------------------------------------------------
	rtn = ScaraBasicIJ(m_cart, m_joint, m_kine, p_config, cal_vel_vector, joint_vel_vector);
	if (rtn != 0)
	{
		//if the det can not satisfy the condition ,we should into emergency stop
		return rtn;
	}
	//-----------------------------------------------------------
	// get joint velocity cmd
	for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
	{
		(m_joint[joint_id]).joint_vel_cmd = joint_vel_vector[joint_id];
	}
	//-----------------------------------------------------------
	return 0;
}

int16 ScaraBasicFJ(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config, double* pose, double* joint)
{
	//---------------------------------------------------------------------------
	double L1 = 0.0;
	double L2 = 0.0;
	double theta1 = 0.0;
	double theta2 = 0.0;
	double S1 = 0.0;
	double S12 = 0.0;
	double C1 = 0.0;
	double C12 = 0.0;
	//---------------------------------------------------------------------------
	theta1 = m_joint[0].joint_pos_cmd * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	theta2 = m_joint[1].joint_pos_cmd * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;

	L1 = m_kine->prm.lenth_of_joint[0];
	L2 = m_kine->prm.lenth_of_joint[1];


	S1 = sin(theta1);
	C1 = cos(theta1);
	S12 = sin(theta1 + theta2);
	C12 = cos(theta1 + theta2);

	//-----------------------------------------------------------------
	/* There is the  Jacobn */
	/*	joint[0] = L2 * C12 * DET * pose[0] + L2 * S12 * DET * pose[1];*/
	pose[0] = (joint[0] * (-L1*S1 - L2*S12) + joint[1] * (-L2*S12))*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	pose[1] = (joint[0] * (L1*C1 + L2*C12) + joint[1] * L2*C12)*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	pose[2] = joint[2];
	pose[3] = joint[5];
	pose[4] = joint[4];
	pose[5] = joint[3];

	return 0;
}

int16 ScaraFJ(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config, double* pose, double* joint)
{

	Uint8 cart_id = 0;
	Uint8 joint_id = 0;


	double cal_vel_vector[3];
	double joint_vel_vector[4];

	//-----------------------------------------------------------------
	// assign pose velocity use cart vel command
	for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
	{
		joint_vel_vector[joint_id] = joint[joint_id];
	}
	//----------------------------------------------------------
	ScaraBasicFJ(m_cart, m_joint, m_kine, p_config, cal_vel_vector, joint_vel_vector);

	//-----------------------------------------------------------
	// get joint velocity cmd
	for (cart_id = 0; cart_id < p_config->cart_dim; cart_id++)
	{
		m_cart->cart_pos_cmd[cart_id] = cal_vel_vector[cart_id];
		pose[cart_id] = cal_vel_vector[cart_id];
	}

	//-----------------------------------------------------------
	return 0;
}
//---------------------------------------------------------------------
//joint to motor ,deg ---count
//---------------------------------------------------------------------
int16 ScaraJ2M(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config)
{
	Uint8 counter;
	//-------------------------------------------
	for (counter = 0; counter<p_config->joint_dim; counter++)
	{
		(m_joint[counter]).servo_pos_cmd = 0.0;
	}
	for (counter = 0; counter< m_kine->spmatxx.non_zero; counter++)
	{
		(m_joint[m_kine->spmatxx.data[counter].row]).servo_pos_cmd += \
			(m_joint[m_kine->spmatxx.data[counter].column]).joint_pos_cmd * \
			m_kine->spmatxx.data[counter].value;
	}
	//-------------------------------------------
	return 0;
}

//---------------------------------------------------------------------
//motor to joint ,count---deg
//---------------------------------------------------------------------
int16 ScaraM2J(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config)
{
	Uint8 counter;

	for (counter = 0; counter<p_config->joint_dim; counter++)
	{
		(m_joint[counter]).joint_pos_fd = 0.0;
	}
	for (counter = 0; counter< m_kine->inv_spmatxx.non_zero; counter++)
	{
		(m_joint[m_kine->inv_spmatxx.data[counter].row]).joint_pos_fd += \
			((m_joint[m_kine->inv_spmatxx.data[counter].column]).servo_pos_fb)* \
			m_kine->inv_spmatxx.data[counter].value;
	}

	return 0;
}
