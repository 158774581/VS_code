#pragma once
/*
* joint_module.h
*
*  Created on: Mar 21, 2018
*      Author: root
*/

#ifndef ROBOT_CONTROL_ROBOT_MODULE_INCLUDE_JOINT_MODULE_H_
#define ROBOT_CONTROL_ROBOT_MODULE_INCLUDE_JOINT_MODULE_H_
//----------------------------------------------------------------------------------

#include "type_def.h"
//#include "SeqCtlKernel.h"
#include "robot_config.h"
//----------------------------------------------------------------------------------

#define JOINT_CMD_BUFFER_NUM	32

//-------------------------------------------------------------------------------------------------

typedef enum
{
	JOINT_CMD_SRC_NORMAL = 0,
	JOINT_CMD_SRC_BUFFER = 1,
	REV_JOINT_CMD_SRC = 0x8000
}JointCmdSrc;
//-------------------------------------------------------------------------------------------------

#pragma pack(4)
typedef struct
{
	JointCmdSrc cmd_src; 							// 0 for cartesian coordinate produce joint cmd,or for joint coordinate p2p cmd, 1 for joint cmd buffer(user set)
	Uint16		mapJ2S;								// mapping of joint id to servo axis
													//------------------------------------------------------------------------------------------------
	Uint16		jog_enable;							// jog enable 1:enable 0:disable
	Uint16    	jog_dir;							// jog direction  0:positive 1:negative
	jogMode		jog_mode;							// jog mode, continue move or move a constant distance
													//-------------------------------------------------------------------------------------------------
	double    	joint_hard_positive_limit;			// hard limit positive value
	double    	joint_hard_negative_limit;			// hard limit negative value
	double    	joint_soft_positive_limit;			// soft limit positive value
	double    	joint_soft_negative_limit;			// soft limit negtive value

													//-------------------------------------------------------------------------------------------------
	double    	joint_sys_vel;  					// joint system velocity unit:degree/s,or mm/s
	double    	joint_sys_acc;  					// joint system acceleration unit:degree/s^2,or mm/s^2
	double    	joint_sys_dec;						// joint system deceleration unit:degree/s^2,or mm/s^2
	double      joint_jog_vel_per;					// joint jog velocity percent

}joint_module_prm;
#pragma pack()
//-------------------------------------------------------------------------------------------------
#pragma pack(4)
typedef struct JOINT_MODULE
{
	//-------------------------------------------------------------------------------------------------
	//parameter
	joint_module_prm 	prm;

	//-------------------------------------------------------------------------------------------------
	//variable
	//-------------------------------------------------------------------------------------------------
	double    	joint_pos_fd;										// unit:deg
	double    	joint_pos_cmd; 										// unit:deg
	double    	joint_vel_fd;										// unit:deg/s
	double    	joint_vel_cmd;										// unit:deg/s
																	//-------------------------------------------------------------------------------------------------
	double      servo_pos_cmd;										// use for communicate with RTN unit:pulse send and receive to change to int64
	double      servo_pos_fb;								  		// use for communicate with RTN unit:pulse
	double 		ctrl_word;											// use for communicate with RTN
	double 		status_word; 										// use for communicate with RTN
																	//-------------------------------------------------------------------------------------------------
	double    	jointCmdBuf[JOINT_CMD_BUFFER_NUM]; 				   	// joint buffer command unit:deg
																	//-------------------------------------------------------------------------------------------------
																	// function pointer define
	int16(*pfInitJointModule)(struct JOINT_MODULE * m_joint, Uint8 id);	// robot joint module initialization
	int16(*pfInitJointVar)(struct JOINT_MODULE * m_joint, Uint8 id);		// robot joint module variable initialization
	int16(*pfMapJ2S)(struct JOINT_MODULE * m_joint);					// robot joint module mapping joint and servo
	int16(*pfCheckSoftStop)(struct JOINT_MODULE * m_joint);				// robot joint module check soft estop limit
	int16(*pfJointSynCmdWithFd)(struct JOINT_MODULE * m_joint);
}joint_module;
#pragma pack()
//-------------------------------------------------------------------------------------------------

int16 InitJointModule(joint_module * m_joint, Uint8 id);
int16 InitJointVar(joint_module * m_joint, Uint8 id);

int16 JointMapJ2S(joint_module* m_joint);
int16 JointPosCmdCheckLimit(joint_module * m_joint);
int16 JointSynCmdWithFd(joint_module * m_joint);
//-------------------------------------------------------------------------------------------------
#endif /* ROBOT_CONTROL_ROBOT_MODULE_INCLUDE_JOINT_MODULE_H_ */
