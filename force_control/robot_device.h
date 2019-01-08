/*
* robot_device.h
*
*  Created on: Apr 16, 2018
*      Author: root
*/

#ifndef ROBOT_CONTROL_ROBOT_MANAGER_INCLUDE_ROBOT_DEVICE_H_
#define ROBOT_CONTROL_ROBOT_MANAGER_INCLUDE_ROBOT_DEVICE_H_
//--------------------------------------------------------------------------------

#include "type_def.h"

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

//#include "robot_device.h"
//#include "robot_app.h"
//--------------------------------------------------------------------------------
typedef enum
{
	ROBOT_DEV_MODE_MANUL = 1,						// manual mode
	ROBOT_DEV_MODE_AUTO = 0,						// auto mode

	ROBOT_DEV_MODE_SIM = 2,						//simulate mode
	ROBOT_DEV_MODE_RUN = 3,						//normal run mode
	REV_ROBOT_DEV_MODE = 0x8000
}robotDevMode;
//---------------------------------------------------------------------------------
typedef enum
{
	ROBOT_DEV_STS_IDLE = 0,						//idle status
	ROBOT_DEV_STS_RUN = 1,						//run status
	ROBOT_DEV_STS_STOP = 2,						//stop status
	REV_ROBOT_DEV_STS = 0x8000
}robotDevSts;
//---------------------------------------------------------------------------------
typedef enum
{
	ROBOT_NO_HOME = 0,						//not homed
	ROBOT_IS_HOMED = 1,						//has homed
	REV_ROBOT_HOME_STATUS = 0x8000
}homeStatus;

//---------------------------------------------------------------------------------
#pragma pack(4)
typedef struct ROBOT_DEVICE_PRM
{
	Uint16							enable;						//robot device enable 	1:enable 0:disable
	robotDevMode					dev_mode;					//robot device mode 	0:simulation mode 1:normal run mode
}robot_dev_prm;
#pragma pack()
//---------------------------------------------------------------------------------

#pragma pack(4)
typedef struct ROBOT_DEVICE
{
	//---------------------------------------------------------------------------------
	//robot device parameters define
	//---------------------------------------------------------------------------------
	robot_dev_prm				prm;							//robot device parameters define

																//---------------------------------------------------------------------------------
																//robot device variable define
																//---------------------------------------------------------------------------------
	Uint16						id;								//robot device id ,use to identify the robot
	Uint16 						type;							//robot type
	robotDevSts					status;							//device current status.
	robotDevMode				manual_auto;
	homeStatus	 				home_flag;						//home flag
	Uint16						station_id;						//rtn station id ,maybe equal to the id or maybe do not equal the id,you should configure this station_id

																//---------------------------------------------------------------------------------
																//robot device include other modules
																//---------------------------------------------------------------------------------
	robot_config_module 		cfg;							//robot configure module
	//robot_mgr					mgr;							//root manager module
	robot_module				module;							//robot modules include joint, Cartesian etc.
	//robot_mode					mode;							//robot mode,different motion mode
																//---------------------------------------------------------------
																// function pointer define

	conveyor_tracking			cnv_trck;
	int16(*pfInitRobotDevice)(struct ROBOT_DEVICE * m_robot_dev, Uint8 id);					//max support robot numbers initialization
	int16(*pfRobotDeviceStatus)(struct ROBOT_DEVICE * m_robot, Uint8 id);					//robot device status
}robot_device;
#pragma pack()

int16	InitRobotDevice(robot_device * m_robot_dev, Uint8 id);
int16 	RobotDeviceStatus(robot_device * m_robot, Uint8 id);
#endif /* ROBOT_CONTROL_ROBOT_MANAGER_INCLUDE_ROBOT_DEVICE_H_ */

