/*
* joint_module.c
*
*  Created on: Mar 21, 2018
*      Author: root
*/
#include "stdio.h"
#include "stdlib.h"
#include "robot_ctl_utility.h"
#include "joint_module.h"


int16 InitJointModule(joint_module * m_joint, Uint8 id)
{
	Uint8 i;
	//-------------------------------------------------------------------------------------
	//if (SEV_PRM_FROM_EX_MEM_EN == ENABLE)
	{

	}
	//else
	{
		m_joint->prm.joint_sys_vel = 100.0;
		m_joint->prm.joint_sys_acc = 10.0;
		m_joint->prm.joint_sys_dec = 10.0;
		m_joint->prm.joint_jog_vel_per = 0.0;

		m_joint->prm.joint_hard_positive_limit = 180.0;
		m_joint->prm.joint_hard_negative_limit = -180.0;
		m_joint->prm.joint_soft_positive_limit = 180.0;
		m_joint->prm.joint_soft_negative_limit = -180.0;

		m_joint->prm.cmd_src = JOINT_CMD_SRC_NORMAL;
		m_joint->prm.mapJ2S = id;

	}

	//-------------------------------------------------------------------------------------
	m_joint->joint_pos_cmd = 0;
	m_joint->joint_pos_fd = 0;
	m_joint->joint_vel_cmd = 0;
	m_joint->joint_vel_fd = 0;

	m_joint->servo_pos_cmd = 0;
	m_joint->servo_pos_fb = 0;
	m_joint->ctrl_word = 0;
	m_joint->status_word = 0;
	//m_joint->free_flag = 0;
	for (i = 0; i<JOINT_CMD_BUFFER_NUM; ++i)
	{
		m_joint->jointCmdBuf[i] = 0;
	}
	//-------------------------------------------------------------------------------------
	m_joint->pfInitJointModule = InitJointModule;
	m_joint->pfInitJointVar = InitJointVar;
	m_joint->pfMapJ2S = JointMapJ2S;
	m_joint->pfCheckSoftStop = JointPosCmdCheckLimit;
	m_joint->pfJointSynCmdWithFd = JointSynCmdWithFd;

	return 0;
}
//-------------------------------------------------------------------------------------
int16 InitJointVar(joint_module * m_joint, Uint8 id)
{
	Uint8 i;
	m_joint->joint_pos_cmd = 0;
	m_joint->joint_pos_fd = 0;
	m_joint->joint_vel_cmd = 0;
	m_joint->joint_vel_fd = 0;

	m_joint->servo_pos_cmd = 0;
	m_joint->servo_pos_fb = 0;
	m_joint->ctrl_word = 0;
	m_joint->status_word = 0;

	for (i = 0; i<JOINT_CMD_BUFFER_NUM; ++i)
	{
		m_joint->jointCmdBuf[i] = 0;
	}

	return 0;
}
//-------------------------------------------------------------------------------------
// map joint and servo axis
//-------------------------------------------------------------------------------------
int16 JointMapJ2S(joint_module* m_joint)
{
#if (SINGLE_TEST_ROBOT == 1)
	//dummy define ,if the network added, then delete the dummy define
	double cmd = 0;
	double fd = 0;
#endif
	//-------------------------------------------------------------------------------------
	/* connect the servo */
	if (m_joint->prm.mapJ2S < 0)
	{
		/* no mapping */
		m_joint->servo_pos_cmd = 0.0;
		m_joint->servo_pos_fb = 0.0;
	}
	else
	{
		if (m_joint->prm.mapJ2S < 2)
		{
			/* SERVO on Master node. */
			//fd get from network, cmd send to network
			//because we have two axis in one cell ,so we send and receive from different address of the rtn
#if (SINGLE_TEST_ROBOT == 1)
			m_joint->servo_pos_fb = fd;
			cmd = m_joint->servo_pos_cmd;
#endif
			//-------------------------------------------------------------
			//if (gSys_cfg.mode_type == NETWORK_MODE)
			{
			//	get_rtn_recv_data(0, m_joint->prm.mapJ2S + 1, &m_joint->servo_pos_fb);
			//	set_rtn_send_data(0, m_joint->prm.mapJ2S, m_joint->servo_pos_cmd);
			//	get_rtn_recv_data(0, m_joint->prm.mapJ2S + 3, &m_joint->status_word);
			//	set_rtn_send_data(0, m_joint->prm.mapJ2S + 2, m_joint->ctrl_word);
			}

		}
		else
		{
			//fd get from network, cmd send to network
#if (SINGLE_TEST_ROBOT == 1)
			m_joint->servo_pos_fb = fd;
			cmd = m_joint->servo_pos_cmd;
#endif
			//-------------------------------------------------------------
			//if (gSys_cfg.mode_type == NETWORK_MODE)
			{
			//	get_rtn_recv_data(1, m_joint->prm.mapJ2S - 1, &m_joint->servo_pos_fb);
			//	set_rtn_send_data(1, m_joint->prm.mapJ2S - 2, m_joint->servo_pos_cmd);
			//	get_rtn_recv_data(1, m_joint->prm.mapJ2S + 1, &m_joint->status_word);
			//	set_rtn_send_data(1, m_joint->prm.mapJ2S + 0, m_joint->ctrl_word);
			}
		}
	}
	//-------------------------------------------------------------------------------------
	return 0;
}
//-------------------------------------------------------------------------------------
//soft stop check
int16 JointPosCmdCheckLimit(joint_module * m_joint)
{
	//if (m_joint->free_flag != 0)
	{
		if (m_joint->joint_pos_cmd > m_joint->prm.joint_soft_positive_limit)
		{
			m_joint->joint_pos_cmd = m_joint->prm.joint_soft_positive_limit;
		}
		if (m_joint->joint_pos_cmd < m_joint->prm.joint_soft_negative_limit)
		{
			m_joint->joint_pos_cmd = m_joint->prm.joint_soft_negative_limit;
		}
	}
	return 0;
}
//-------------------------------------------------------------------------------------
int16 JointSynCmdWithFd(joint_module * m_joint)
{
	m_joint->joint_pos_cmd = m_joint->joint_pos_fd;
	m_joint->servo_pos_cmd = m_joint->servo_pos_fb;

	return 0;
}
