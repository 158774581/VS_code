#pragma once
/*
* robot_mode.h
*
*  Created on: Apr 17, 2018
*      Author: root
*/

#ifndef ROBOT_CONTROL_ROBOT_MODULE_ROBOT_MODE_INCLUDE_ROBOT_MODE_H_
#define ROBOT_CONTROL_ROBOT_MODULE_ROBOT_MODE_INCLUDE_ROBOT_MODE_H_
//-------------------------------------------------------------------------------------------

#include "type_def.h"
//#include "robot_jog_1.h"
//#include "robot_macro.h"
//-------------------------------------------------------------------------------------------
typedef	enum
{
	ROBOT_MODULE_IDLE_MODE = 0,													// robot in idle mode
	ROBOT_MODULE_HARD_ESTOP_MODE = 1,													// hard estop mode
	ROBOT_MODULE_SOFT_ESTOP_MODE = 2,													// soft estop mode
	ROBOT_MODULE_CHANGE_LOCAT_MODE = 3,													// change location
	ROBOT_MODULE_HOME_MODE = 4,													// home mode
	ROBOT_MODULE_LOCALBUFFER_MODE = 5,													// local buffer mode
	ROBOT_MODULE_MACRO_PROGRAM_MODE = 6,													// macro program parse mode
	ROBOT_MODULE_USER_PROGRAM_MODE = 7,													// user program parse mode
	ROBOT_MODULE_JOG_MODE = 8,													// jog mode
	ROBOT_MODULE_SCRIPT_MODE = 9,													// script mode
	REV_ROBOT_MODULE_MODE = 0x8000
}RobotMode;
//----------------------------------------------------------------------------------
#pragma pack(4)
typedef struct ROBOT_MODE
{
	RobotMode			mode;																			// robot mode define
	RobotMode			mode_tmp;																		// robot mode tmp define,use for change mode
	Uint16              resume_flag;																	// robot script move resume flag

	//robot_rapidstop		rstop;																			// rapid stop
	//robot_resume		res;																			// resume from soft estop
	//robot_jog           rjog;                                                                           // jog mode

	//robot_macro			rmacro;																			// macro mode
																										//----------------------------------------------------------------------------------
	int16(*pfInitRobotMode)(struct ROBOT_MODE *m_mode, Uint8 id);
	//int16(*pfRobotModeSwtich)(robot_module *m_mod, trajectory_module* m_traj, \
		robot_config_module* cfg, Uint8 id);

}robot_mode;
#pragma pack()
//----------------------------------------------------------------------------------
int16	InitRobotMode(robot_mode *m_mode, Uint8 id);
//int16	RobotModeSwtich(robot_module* m_mod, trajectory_module* m_traj, robot_config_module* cfg, Uint8 id);
//----------------------------------------------------------------------------------

#endif /* ROBOT_CONTROL_ROBOT_MODULE_ROBOT_MODE_INCLUDE_ROBOT_MODE_H_ */
