/*
* robot_app.h
*
*  Created on: Apr 17, 2018
*      Author: root
*/

#ifndef ROBOT_CONTROL_ROBOT_MANAGER_INCLUDE_ROBOT_APP_H_
#define ROBOT_CONTROL_ROBOT_MANAGER_INCLUDE_ROBOT_APP_H_

#include "type_def.h"
#include "robot_ctl_utility.h"

#include "robot_config.h"
//#include "robot_mgr.h"
#include "joint_module.h"
#include "cartesian_module.h"
#include "kinematics.h"
#include "trajectory_generator.h"
#include "motion_planning_3rd.h"
#include "robot_coordinate_system.h"
#include "robot_module.h"
#include "conveyor_tracking.h"

#include "robot_device.h"
//#include "robot_app.h"
//--------------------------------------------------------------------
#pragma pack(4)
typedef struct ROBOT
{
	//---------------------------------------------------------------
	// common variable  define ,you can add additional variable define in here
	//---------------------------------------------------------------
	Uint16		 device_num;															//record devices numbers of the platform can support

																						//---------------------------------------------------------------
																						// robot device define
																						//---------------------------------------------------------------
	robot_device robot_dev[ROBOT_DEVICE_MAX_NUM];										//max support device object define

																						//---------------------------------------------------------------
																						// function pointer define
	int16(*pfInitRobot)(struct ROBOT * m_robot);									//max support robot device numbers initialization
}robot;
#pragma pack()

//-------------------------------------------------------------------------------------
//							 FUNCTION DECLARE
//-------------------------------------------------------------------------------------
int16	InitRobot(robot * m_robot);
//-------------------------------------------------------------------------------------
 extern robot gRobot;
//-------------------------------------------------------------------------------------
#endif /* ROBOT_CONTROL_ROBOT_MANAGER_INCLUDE_ROBOT_APP_H_ */
