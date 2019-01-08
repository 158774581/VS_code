/*
* cartesian_module.c
*
*  Created on: Mar 21, 2018
*      Author: root
*/
#include "type_def.h"
#include <stdio.h>
#include <stdlib.h>

#include "robot_ctl_utility.h"
#include "robot_config.h"
#include "cartesian_module.h"

//-------------------------------------------------------------------------------------
int16 InitCartesianModule(cartesian_module * m_cart, robot_config_module * m_cfg, Uint8 id)
{
	Uint8 i, j;
	//-------------------------------------------------------------------------------------
	//if (SEV_PRM_FROM_EX_MEM_EN == ENABLE)
	{

	}
	//else
	{
		m_cart->prm.cart_trans_sys_vel = 100.0;  // unit:mm/s
		m_cart->prm.cart_trans_sys_acc = 25.0;  // unit:mm/s^2
		m_cart->prm.cart_trans_sys_dec = 25.0;  // unit:mm/s^2
		m_cart->prm.cart_rotate_sys_vel = 3.0 / 3.1415926 * 180.0;  // unit:rad/s
		m_cart->prm.cart_rotate_sys_acc = 1.0 / 3.1415926 * 180.0;  // unit:rad/s^2
		m_cart->prm.cart_rotate_sys_dec = 1.0 / 3.1415926 * 180.0;  // unit:rad/s^2
		m_cart->prm.cart_trans_jog_vel_per = 0.0;
	}

	//-------------------------------------------------------------------------------------
	for (i = 0; i < m_cfg->cart_dim; ++i)
	{
		m_cart->cart_pos_cmd[i] = 0;
		m_cart->cart_pos_fd[i] = 0;
		m_cart->cart_vel_cmd[i] = 0;
		m_cart->cart_vel_fd[i] = 0;
	}
	for (i = 0; i < CART_CMD_BUFFER_NUM; ++i)
	{
		m_cart->cartCmdBuf[i].move_type = 0;

		for (j = 0; j < 6; ++j)
		{
			m_cart->cartCmdBuf[i].pose[j] = 0;
		}
	}
	//-------------------------------------------------------------------------------------
	m_cart->pfInitCartesianModule = InitCartesianModule;
	m_cart->pfInitCartesianVar = InitCartesianVar;
	m_cart->pfCartSynCmdWithFd = CartSynCmdWithFd;

	return 0;
}
//-------------------------------------------------------------------------------------
int16 InitCartesianVar(cartesian_module * m_cart, robot_config_module * m_cfg, Uint8 id)
{
	Uint8 i, j;
	for (i = 0; i< m_cfg->cart_dim; ++i)
	{
		m_cart->cart_pos_cmd[i] = 0;
		m_cart->cart_pos_fd[i] = 0;
		m_cart->cart_vel_cmd[i] = 0;
		m_cart->cart_vel_fd[i] = 0;
	}
	for (i = 0; i < CART_CMD_BUFFER_NUM; ++i)
	{
		m_cart->cartCmdBuf[i].move_type = 0;

		for (j = 0; j < 6; ++j)
		{
			m_cart->cartCmdBuf[i].pose[j] = 0;
		}
	}

	return 0;
}
//-------------------------------------------------------------------------------------

Uint32 CartSynCmdWithFd(cartesian_module * m_cart, robot_config_module * m_cfg)
{
	Uint8 i;
	for (i = 0; i< m_cfg->cart_dim; ++i)
	{
		m_cart->cart_pos_cmd[i] = m_cart->cart_pos_fd[i];
	}

	return 0;
}
