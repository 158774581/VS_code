#pragma once
/*
* robot_module.h
*
*  Created on: Mar 21, 2018
*      Author: root
*/

#ifndef ROBOT_CONTROL_ROBOT_MODULE_INCLUDE_ROBOT_MODULE_H_
#define ROBOT_CONTROL_ROBOT_MODULE_INCLUDE_ROBOT_MODULE_H_
//-------------------------------------------------------------------------------------------
#include "type_def.h"
#include "dynamic_module.h"
#include "cartesian_module.h"
#include "joint_module.h"
#include "trajectory_generator.h"
#include "kinematics.h"
#include "robot_coordinate_system.h"
//-------------------------------------------------------------------------------------------

#pragma pack(4)
typedef struct ROBOT_MODULE
{

	cartesian_module 		cart;																	//robot cartesian coordinate
	joint_module 			joint[JOINT_COORDINATE_NUM];											//robot joint coordinate
	trajectory_module 		trajectory;																//robot trajectory
	kinematics				kinematics;																//robot kinematics
	dynamic_module				dym_mod;

	robot_coordinate_system			coordinate;														//robot coordinate system
	//-------------------------------------------------------------------------------------------
	//---------------------------------------------------------------
	// function pointer define
	int16(*pfInitRobotModule)(struct ROBOT_MODULE * m_robot_module, robot_config_module* cfg, Uint8 id);					//robot module initialization
	int16(*pfSyncRobotModule)(struct ROBOT_MODULE * m_robot_module, robot_config_module* cfg, Uint8 id);					//robot module sync cmd with fd
	int16(*pfAsynCalcMotionProfile)(struct ROBOT_MODULE * m_robot_module, \
		robot_config_module* m_cfg);								//async calculate motion profile
	int16(*pfRtnCycleUpdata)(struct ROBOT_MODULE *m_robot_module, \
		robot_config_module* m_cfg, Uint8 station_id, Uint8 id);				//rtn cycle updata command and use feedback
	int16(*pfRobotModuleCtlSchedule)(struct ROBOT_MODULE *m_robot_module, \
		robot_config_module* cfg, Uint8 id);							//robot module control schedule
}robot_module;
#pragma pack()
//-------------------------------------------------------------------------------------------
int16 InitRobotModule(robot_module *m_robot_module, robot_config_module* cfg, Uint8 id);
int16 SyncRobotModule(robot_module * m_robot_module, robot_config_module* cfg, Uint8 id);
int16 AsynCalcMotionProfile(robot_module *m_robot_module, robot_config_module* m_cfg);
int16 RtnCycleUpdata(robot_module *m_robot_module, robot_config_module* m_cfg, Uint8 station_id, Uint8 id);
int16 RobotModuleCtlSchedule(robot_module* m_robot_module, robot_config_module* cfg, Uint8 id);
//-------------------------------------------------------------------------------------------
void  robot_module_network_interface(robot_module *m_robot_module, robot_config_module* m_cfg, Uint8 station_id);
//-------------------------------------------------------------------------------------------
#endif /* ROBOT_CONTROL_ROBOT_MODULE_INCLUDE_ROBOT_MODULE_H_ */
