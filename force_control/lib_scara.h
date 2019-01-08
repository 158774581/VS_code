#pragma once
/*
* lib_scara.h
*
*  Created on: Mar 21, 2018
*      Author: root
*/

#ifndef ROBOT_CONTROL_ROBOT_LIBRARY_INCLUDE_LIB_SCARA_H_
#define ROBOT_CONTROL_ROBOT_LIBRARY_INCLUDE_LIB_SCARA_H_

#include "type_def.h"

//--------------------------------------------------------------------------------------
// machinal decouple function j2m---joint to motor  m2j----motor to joint
//--------------------------------------------------------------------------------------
int16  ScaraJ2M(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config);

int16  ScaraM2J(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config);

//--------------------------------------------------------------------------------------
//forward kinematics
//--------------------------------------------------------------------------------------
int16  ScaraFK(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config, double* pose, double* joint);

//--------------------------------------------------------------------------------------
//inverse kinematics
//--------------------------------------------------------------------------------------
int16  ScaraIK(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config, double* pose, double* joint, Uint8 mode);

//--------------------------------------------------------------------------------------
//inverse jacobn
//--------------------------------------------------------------------------------------
int16  ScaraIJ(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config, double* pose, double* joint);

int16 ScaraFJ(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config, double* pose, double* joint);

int16 ScaraBasicFJ(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config, double* pose, double* joint);

int16 ScaraBasicIJ(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config, const double* pose, double* joint);

//----------------------------------------------------------------------------------

#endif /* ROBOT_CONTROL_ROBOT_LIBRARY_INCLUDE_LIB_SCARA_H_ */
