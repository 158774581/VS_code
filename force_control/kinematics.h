#pragma once
/*
* kinematics_cfg.h
*
*  Created on: Mar 22, 2018
*      Author: root
*/

#ifndef ROBOT_CONTROL_ROBOT_LIBRARY_INCLUDE_KINEMATICS_CFG_H_
#define ROBOT_CONTROL_ROBOT_LIBRARY_INCLUDE_KINEMATICS_CFG_H_
//-----------------------------------------------------------------------
#include "type_def.h"
#include "matrix.h"
#include "cartesian_module.h"
#include "joint_module.h"
#include "robot_config.h"
#include "robot_ctl_utility.h"
//-------------------------------------------------------------------------------------------------
#pragma pack(4)
typedef struct KINEMATICS_PRM
{
	//-------------------------------------------------------------------------------------------------
	int16		dir[JOINT_COORDINATE_NUM];								// check joint move direction to match which direction we want,  value is  1 or -1
																		// generally for scara ,joint1 and joint2 z direct up(right hand rule)
																		// joint3 move down is positive,joint4 rotate z direct down (right hand rule)
	int32		encoder_lineNum[JOINT_COORDINATE_NUM];					// motor encoder line number
	double    	gear_ratio[JOINT_COORDINATE_NUM]; 						// motor reduction ratio
	double		mech_couple[JOINT_COORDINATE_NUM];						// mechanical couple items   default number is JOINT_COORDINATE_NUM,
																		// maybe used for 6 axis robot
	double    	lenth_of_joint[JOINT_COORDINATE_NUM];					// rod length 				unit:mm
	double		screw_pitch[JOINT_COORDINATE_NUM];						// screw pitch use for translate move joint unit mm

																		//-------------------------------------------------------------------------------------------------
	double    	dh_param_a[JOINT_COORDINATE_NUM]; 						// unit:mm					//D-H parameter
	double    	dh_param_d[JOINT_COORDINATE_NUM]; 						// unit:mm
	double    	dh_param_alpha[JOINT_COORDINATE_NUM]; 					// unit:deg
	double    	dh_param_theta[JOINT_COORDINATE_NUM]; 					// unit:deg
}kinematics_prm;
#pragma pack()
//-------------------------------------------------------------------------------------------------
#pragma pack(4)
typedef struct KINEMATICS
{
	kinematics_prm prm;

	//-------------------------------------------------------------------------------------------------
	//variable
	//-------------------------------------------------------------------------------------------------
	double		spg[JOINT_COORDINATE_NUM];								// spmatrix internal used g1,g2,g3,g4.....	unit:count/degree
	double    	inv_spg[JOINT_COORDINATE_NUM]; 							// 1/g1, 1/g2..., 							unit:deg/count
	double		spk[JOINT_COORDINATE_NUM];								// spmatrix internal used k1,k2,k3,k4.....	unit:count/degree
	double    	inv_spk[JOINT_COORDINATE_NUM]; 							// 1/k1, 1/k2..., 							unit:deg/count
	SPMatxx 	spmatxx;												// spmatrix use for joint 2 motor
	SPMatxx 	inv_spmatxx;											// spmatrix use for motor 2 joint
																		//---------------------------------------------------------------
																		// function pointer define
	int16(*pfFK)(cartesian_module * m_cart, joint_module* m_joint, struct KINEMATICS* m_kine, robot_config_module * m_cfg, double* pose, double* joint);
	int16(*pfIK)(cartesian_module * m_cart, joint_module* m_joint, struct KINEMATICS* m_kine, robot_config_module * m_cfg, double* pose, double* joint, Uint8 mode);
	int16(*pfIJ)(cartesian_module * m_cart, joint_module* m_joint, struct KINEMATICS* m_kine, robot_config_module * m_cfg, double* pose, double* joint);
	int16(*pfFJ)(cartesian_module * m_cart, joint_module* m_joint, struct KINEMATICS* m_kine, robot_config_module * m_cfg, double* pose, double* joint);
	int16(*pfM2J)(cartesian_module * m_cart, joint_module* m_joint, struct KINEMATICS* m_kine, robot_config_module * m_cfg);
	int16(*pfJ2M)(cartesian_module * m_cart, joint_module* m_joint, struct KINEMATICS* m_kine, robot_config_module * m_cfg);

	//----------------------------------------------------------------
	int16(*pfInitKinematics)(struct KINEMATICS * handle, robot_config_module* handle_config, Uint8 id);
	int16(*pfConfigKinematics)(struct KINEMATICS * handle, robot_config_module* handle_config, Uint8 id);
}kinematics;
#pragma pack()
//-----------------------------------------------------------------------
int16 InitKinematics(kinematics * handle, robot_config_module* handle_config, Uint8 id);
int16 configKinematics(kinematics* handle, robot_config_module* handle_config, Uint8 id);
//-----------------------------------------------------------------------

#endif /* ROBOT_CONTROL_ROBOT_LIBRARY_INCLUDE_KINEMATICS_CFG_H_ */
