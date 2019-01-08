/*
* robot_config.c
*
*  Created on: Mar 21, 2018
*      Author: root
*/

#include "robot_ctl_utility.h"
#include "robot_config.h"
#include "stdbool.h"
//-------------------------------------------------------------------
//init robot config module
//-------------------------------------------------------------------
int16 InitRobotConfigModule(robot_config_module * m_robot_cfg, Uint8 id)
{
	//-------------------------------------------------------------------------------------
	//if (SEV_PRM_FROM_EX_MEM_EN == ENABLE)
	{

	}
	//else
	{
	/*	m_robot_cfg->prm.robot_cmd_src = ROBOT_CMD_SRC_SCRIPT;
		m_robot_cfg->prm.robot_cmd_smod = ROBOT_CMD_SCRIPT_MANUAL;
		m_robot_cfg->prm.robot_type = MOTION_MODULE_DEFAULT_ROBOT_TYPE_SCARAJOINT;
		m_robot_cfg->prm.location_config = KINEMATICS_LIBRARY_CONFIG_RIGHTY;
		m_robot_cfg->prm.jog_coordinate = ROBOT_JOG_JOINT;*/


	}
	m_robot_cfg->prm.location_config = KINEMATICS_LIBRARY_CONFIG_RIGHTY;
	//-------------------------------------------------------------------------------------
	m_robot_cfg->cart_dim = ROBOT_CONFIG_MODULE_CARTESIAN_6_DIM;
	m_robot_cfg->cart_trans_dim = ROBOT_CONFIG_MODULE_CARTESIAN_TRANS_3_DIM;
	m_robot_cfg->cart_rotate_dim = ROBOT_CONFIG_MODULE_CARTESIAN_ROTATE_3_DIM;
	m_robot_cfg->locat_tmp = KINEMATICS_LIBRARY_CONFIG_RIGHTY;
	m_robot_cfg->locat_change_block_id = MOTION_MODULE_DEFAULT_MOTION_BLOCK_SIZE;
	m_robot_cfg->locat_change_flag = false;
	m_robot_cfg->locat_change_finish = true;
	m_robot_cfg->stop_script_flag = false;

	switch (m_robot_cfg->prm.robot_type)
	{
	case MOTION_MODULE_DEFAULT_ROBOT_TYPE_SCARA:/* robot type: scara 4 axis */
		m_robot_cfg->joint_dim = ROBOT_CONFIG_MODULE_DEFAULT_JOINT_DIM_OF_SCARA;
		break;

		//add robot type here ,different type can have different joint
	case MOTION_MODULE_DEFAULT_ROBOT_TYPE_SCARAJOINT: /* robot type: Joint 2 axis*/
	default:
		m_robot_cfg->joint_dim = ROBOT_CONFIG_MODULE_DEFAULT_JOINT_DIM;
		break;
	}

	//-------------------------------------------------------------------------------------
	m_robot_cfg->pfInitRobotConfigModule = InitRobotConfigModule;
	m_robot_cfg->pfInitRobotConfigVar = InitRobotConfigVar;

	return 0;
}
//-------------------------------------------------------------------
//init robot config module var
//-------------------------------------------------------------------

int16 InitRobotConfigVar(robot_config_module * m_robot_cfg, Uint8 id)
{
	m_robot_cfg->cart_dim = ROBOT_CONFIG_MODULE_CARTESIAN_6_DIM;
	m_robot_cfg->cart_trans_dim = ROBOT_CONFIG_MODULE_CARTESIAN_TRANS_3_DIM;
	m_robot_cfg->cart_rotate_dim = ROBOT_CONFIG_MODULE_CARTESIAN_ROTATE_3_DIM;
	m_robot_cfg->locat_tmp = KINEMATICS_LIBRARY_CONFIG_RIGHTY;
	m_robot_cfg->locat_change_block_id = MOTION_MODULE_DEFAULT_MOTION_BLOCK_SIZE;
	m_robot_cfg->locat_change_flag = false;
	m_robot_cfg->locat_change_finish = true;
	m_robot_cfg->stop_script_flag = false;

	switch (m_robot_cfg->prm.robot_type)
	{
	case MOTION_MODULE_DEFAULT_ROBOT_TYPE_SCARA:/* robot type: scara 4 axis */
		m_robot_cfg->joint_dim = ROBOT_CONFIG_MODULE_DEFAULT_JOINT_DIM_OF_SCARA;
		break;

		//add robot type here ,different type can have different joint
	case MOTION_MODULE_DEFAULT_ROBOT_TYPE_SCARAJOINT: /* robot type: Joint 2 axis*/
	default:
		m_robot_cfg->joint_dim = ROBOT_CONFIG_MODULE_DEFAULT_JOINT_DIM;
		break;
	}

	return 0;
}
