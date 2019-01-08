#pragma once
/*
* robot_config.h
*
*  Created on: Mar 21, 2018
*      Author: root
*/

#ifndef ROBOT_CONTROL_ROBOT_MODULE_INCLUDE_ROBOT_CONFIG_H_
#define ROBOT_CONTROL_ROBOT_MODULE_INCLUDE_ROBOT_CONFIG_H_

#include "type_def.h"

//---------------------------------------------------------------------------------

typedef	enum
{
	ROBOT_CMD_SRC_SCRIPT = 0,						// robot motion block command come from parse the script
	ROBOT_CMD_SRC_LOCAL_BUFFER = 1,						// robot motion block command come from local buffer
	ROBOT_CMD_SRC_EXT_MACRO = 2,						// robot motion block command come from external macro parse
	ROBOT_CMD_SRC_EXT_USERPROGRAM = 3,						// robot motion block command come from external user program
	REV_ROBOT_CMD_SRC = 0x8000
}robotCmdSrc;
//---------------------------------------------------------------------------------
typedef	enum
{
	ROBOT_CMD_SCRIPT_AUTO = 0,						// robot motion block command come from parse the script,and the script are loaded auto
	ROBOT_CMD_SCRIPT_MANUAL = 1,						// robot motion block command come from parse the script,and the script are loaded manual,which need shell
	REV_ROBOT_CMD_SCRIPT = 0x8000
}robotCmdScriptMode;

//---------------------------------------------------------------------------------
typedef enum
{
	ROBOT_JOG_JOINT = 0,						//joint coordinate jog
	ROBOT_JOG_CARTESIAN = 1,						//cartesian coordinate jog
	REV_ROBOT_JOG = 0x8000
}jogCoord;

//---------------------------------------------------------------------------------

typedef enum
{
	ROBOT_JOG_CONSTANT_MODE = 0,						//jog constant
	ROBOT_JOG_COUNTINUE_MODE = 1,						//jog continue
	REV_ROBOT_JOG_MODE = 0x8000
}jogMode;

//---------------------------------------------------------------------------------
//robot config module parameter

#pragma pack(4)
typedef struct
{
	robotCmdSrc			robot_cmd_src;											// robot motion block command source
	robotCmdScriptMode	robot_cmd_smod;											// script mode auto or manual
	Uint16    			robot_type;												// robot type define
	Uint16      		location_config;										// hand configure left or right, right :1 left:2
	jogCoord			jog_coordinate;											// jog coordinate config 1:cartesian 0:joint

}robot_config_module_prm;
#pragma pack()
//------------------------------------------------------------------

#pragma pack(4)
typedef struct ROBOT_CONFIG_MODULE
{
	//------------------------------------------------------------------
	robot_config_module_prm prm;
	//------------------------------------------------------------------
	Uint16    cart_dim;																			//Cartesian dimension
	Uint16    cart_trans_dim;																	//Cartesian translate dimension
	Uint16    cart_rotate_dim;																	//Cartesian rotation dimension
	Uint16    joint_dim;																		//joint dimension
	Uint16    locat_tmp;																		//locat change tmp variable
	Uint16	  locat_change_block_id;															//record locat block id
	Uint16    locat_change_flag;																//locat change flag
	Uint16    locat_change_finish;																//stop script xml file
	Uint16 	  stop_script_flag;
	//------------------------------------------------------------------
	// function pointer define
	int16(*pfInitRobotConfigModule)(struct ROBOT_CONFIG_MODULE * m_robot_cfg, Uint8 id);		// robot configure module initialization
	int16(*pfInitRobotConfigVar)(struct ROBOT_CONFIG_MODULE * m_robot_cfg, Uint8 id);		// robot configure module variable initialization

}robot_config_module;
#pragma pack()
//------------------------------------------------------------------

int16 InitRobotConfigModule(robot_config_module * m_robot_cfg, Uint8 id);
int16 InitRobotConfigVar(robot_config_module * m_robot_cfg, Uint8 id);

//------------------------------------------------------------------


#endif /* ROBOT_CONTROL_ROBOT_MODULE_INCLUDE_ROBOT_CONFIG_H_ */
