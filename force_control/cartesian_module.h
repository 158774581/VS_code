#pragma once
/*
* cartesian_module.h
*
*  Created on: Mar 21, 2018
*      Author: root
*/

#ifndef ROBOT_CONTROL_ROBOT_MODULE_INCLUDE_CARTESIAN_MODULE_H_
#define ROBOT_CONTROL_ROBOT_MODULE_INCLUDE_CARTESIAN_MODULE_H_
//------------------------------------------------------------------
#include "type_def.h"
#include "robot_ctl_utility.h"
//------------------------------------------------------------------


#define CART_CMD_BUFFER_NUM	32

//-------------------------------------------------------------------------------------------------

typedef enum
{
	CART_CMD_SRC_NORMAL = 0,
	CART_CMD_SRC_BUFFER = 1,
	REV_CART_CMD_SRC = 0x8000
}CartCmdSrc;


typedef struct CARTCMDBUF
{
	Uint16  move_type;
	double pose[6];

}CartCmdBuf;
//----------------------------------------------------------------------------------
#pragma pack(4)
typedef struct CARTESIAN_MODULE_PRM
{
	CartCmdSrc			cmd_src;												// Cartesian cmd source 0:
																				//-----------------------------------------------------
	Uint16				jog_enable;												// jog enable 1:enable 0:disable
	Uint16    			jog_dir;												// jog direction  0:positive 1:negative
	//jogMode				jog_mode;												// jog mode, continue move or move a constant distance
																				//-----------------------------------------------------
	double    			cart_trans_sys_vel;  									// Cartesian translate motion system velocity 		unit:mm/s
	double    			cart_trans_sys_acc;  									// Cartesian translate motion system acceleration   unit:mm/s^2
	double    			cart_trans_sys_dec;  									// Cartesian translate motion system deceleration	unit:mm/s^2
	double    			cart_rotate_sys_vel;  									// Cartesian rotation motion system velocity 		unit:rad/s
	double    			cart_rotate_sys_acc;  									// Cartesian rotation motion system acceleration 	unit:rad/s^2
	double    			cart_rotate_sys_dec;  									// Cartesian rotation motion system deceleration	unit:rad/s^2
	double				cart_trans_jog_vel_per;									// Cartesian translate motion system velocity percent
}cartesian_module_prm;
#pragma pack()
//------------------------------------------------------------------
#pragma pack(4)
typedef struct CARTESIAN_MODULE
{
	cartesian_module_prm prm;
	//-------------------------------------------------------------------------------------------------
	//variable
	//-------------------------------------------------------------------------------------------------
	double	  	cart_vel_fd[CART_COORDINATE_DIM];
	double    	cart_vel_cmd[CART_COORDINATE_DIM];
	double    	cart_pos_cmd[CART_COORDINATE_DIM];
	double    	cart_pos_fd[CART_COORDINATE_DIM];
	CartCmdBuf	cartCmdBuf[CART_CMD_BUFFER_NUM]; 				   												// cartesian buffer command
																												//------------------------------------------------------------------
																												// function pointer define
	int16(*pfInitCartesianModule)(struct CARTESIAN_MODULE * m_cart, robot_config_module * m_cfg, Uint8 id);	// robot Cartesian module initialization
	int16(*pfInitCartesianVar)(struct CARTESIAN_MODULE * m_cart, robot_config_module * m_cfg, Uint8 id);		// robot Cartesian module variable initialization
	Uint32(*pfCartSynCmdWithFd)(struct CARTESIAN_MODULE * m_cart, robot_config_module * m_cfg);				// robot Cartesian sync pos cmd with pos fd
}cartesian_module;
#pragma pack()
//------------------------------------------------------------------

int16  InitCartesianModule(cartesian_module * m_cart, robot_config_module * m_cfg, Uint8 id);
int16  InitCartesianVar(cartesian_module * m_cart, robot_config_module * m_cfg, Uint8 id);
Uint32 CartSynCmdWithFd(cartesian_module * m_cart, robot_config_module * m_cfg);

#endif /* ROBOT_CONTROL_ROBOT_MODULE_INCLUDE_CARTESIAN_MODULE_H_ */
