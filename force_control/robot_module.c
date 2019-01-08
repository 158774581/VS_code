/*
* robot_module.c
*
*  Created on: Mar 21, 2018
*      Author: root
*/

#include "type_def.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "matrix.h"
#include "robot_ctl_utility.h"
#include "robot_config.h"

#include "robot_module.h"

#include "robot_app.h"
//---------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
int16 InitRobotModule(robot_module *m_robot_module, robot_config_module* cfg, Uint8 id)
{
	Uint8 index;

	//-------------------------------------------------------------

	//-------------------------------------------------------------
	m_robot_module->pfInitRobotModule = InitRobotModule;
	//m_robot_module->pfAsynCalcMotionProfile = AsynCalcMotionProfile;
	//m_robot_module->pfRtnCycleUpdata = RtnCycleUpdata;
	//m_robot_module->pfRobotModuleCtlSchedule = RobotModuleCtlSchedule;
	//-------------------------------------------------------------

	//---------------------------------------------------------
	//init robot kinematics
	InitKinematics(&(m_robot_module->kinematics), cfg, id);

	//----------------------------------------------------------------
	//config kinematics,it has been initial
	//m_robot_module->kinematics.pfConfigKinematics(&(m_robot_module->kinematics),cfg,id);
	configKinematics(&(m_robot_module->kinematics), cfg, id);
	//----------------------------------------------------------------
	//initial Cartesian module

	InitCartesianModule(&(m_robot_module->cart), cfg, id);

	//----------------------------------------------------------------
	//init joint module
	for (index = 0; index < cfg->joint_dim; index++)
	{
		InitJointModule(&(m_robot_module->joint[index]), id);
	}
	m_robot_module->joint[2].prm.joint_hard_positive_limit = 1000;  //1 m
	m_robot_module->joint[2].prm.joint_hard_negative_limit = -1000;  //1 m
	m_robot_module->joint[2].prm.joint_soft_positive_limit = 1000;  //1 m
	m_robot_module->joint[2].prm.joint_soft_negative_limit = -1000;  //1 m
	//----------------------------------------------------------------
	//init trajectory module
	InitTrajectoryModule(&m_robot_module->trajectory, cfg, id);

	//init robot coordinate system
	InitRobotCoordinateSystem(&m_robot_module->coordinate);
	return 0;
}
//sync robot module internal's cmd with fd
int16 SyncRobotModule(robot_module * m_robot_module, robot_config_module* cfg, Uint8 id)
{
	Uint8 index;
	double joint[6];
	double cart[6];
	double jtmp[6];
	double jtmp1[6];
	//----------------------------------------------------------------
	//map
	for (index = 0; index < cfg->joint_dim; index++)
	{
		// map real time network pos to joint servo pos feedback of joint,get data to and from rtn
		(m_robot_module->joint[index]).pfMapJ2S((joint_module*)(&m_robot_module->joint[index]));
		jtmp[index] = m_robot_module->joint[index].servo_pos_fb;

		(m_robot_module->joint[index]).pfMapJ2S((joint_module*)(&m_robot_module->joint[index]));
		jtmp1[index] = m_robot_module->joint[index].servo_pos_fb;

		if (fabs(jtmp[index] - jtmp1[index]) > 10)
		{
			printf("sync position of axis %d  error,please restart the machine or call sync function! \n", index);
		}
	}
	//-------------------------------------------------------------------
	// update the feedback of joint according to the joint servo pos and gear ratio
	m_robot_module->kinematics.pfM2J(&m_robot_module->cart, m_robot_module->joint, &m_robot_module->kinematics, cfg);

	//-------------------------------------------------------------------
	for (index = 0; index < cfg->joint_dim; index++)
	{
		// synchronize joint pos command using joint pos feedback
		(m_robot_module->joint[index]).pfJointSynCmdWithFd((joint_module*)(&m_robot_module->joint[index]));

		//write the joint pos feedback to joint array ,use for cartesian
		joint[index] = (m_robot_module->joint[index]).joint_pos_fd;

		//gRobot.robot_dev[id].mode.rmacro.joint_end[index] = joint[index]; //add_20181108
	}

	//-------------------------------------------------------------------
	//get Cartesian from joint pos feedback
	m_robot_module->kinematics.pfFK(&m_robot_module->cart, m_robot_module->joint, &m_robot_module->kinematics, cfg, cart, joint);

	//-------------------------------------------------------------------
	// get Cartesian fd
	for (index = 0; index < cfg->cart_dim; index++)
	{
		//get cartesian pos fd  from cart array
		m_robot_module->cart.cart_pos_fd[index] = cart[index];

		//gRobot.robot_dev[id].mode.rmacro.pose_end[index] = cart[index]; //add_20181108
	}
	//synchronize cartesian pos command using cartesian pos feedback
	m_robot_module->cart.pfCartSynCmdWithFd((cartesian_module*)(&m_robot_module->cart), cfg);

	// synchronize motion block states with feedback states
	//m_robot_module->trajectory.pfTrajSynCmdWithFd(&m_robot_module->trajectory, cfg, &m_robot_module->cart, &m_robot_module->joint[0]);

	m_robot_module->trajectory.current_time = 0;  //add_hqi

	return 0;

}

/*
//-------------------------------------------------------------------------------------------------------
// robot module control schedule routinue ,use for switch mode
//-------------------------------------------------------------------------------------------------------
int16 RobotModuleCtlSchedule(robot_module* m_robot_module, robot_config_module* cfg, Uint8 id)
{
	RobotModeSwtich(&gRobot.robot_dev[id].module, &m_robot_module->trajectory, cfg, id);
	return 0;
}
//-------------------------------------------------------------------------------------------------------
//async task which use for calculate motion profile
//-------------------------------------------------------------------------------------------------------
int16 AsynCalcMotionProfile(robot_module *m_robot_module, robot_config_module* m_cfg)
{
	int16 rtn = 0;
	//rtn = m_robot_module->trajectory.pfCalcMotionProfile(&m_robot_module->trajectory, m_cfg);

	return rtn;
}


//-------------------------------------------------------------------------------------------------------
// cycle update data to and from real time network
//-------------------------------------------------------------------------------------------------------
int16 RtnCycleUpdata(robot_module *m_robot_module, robot_config_module* m_cfg, Uint8 station_id, Uint8 id)
{
	Uint8  index = 0;
	//----------------------------------------------------------------
	double joint[6];
	double cart[6];
	//----------------------------------------------------------------

	// send and receive Real-time network data
	robot_module_network_interface(m_robot_module, m_cfg, station_id);

	//----------------------------------------------------------------
	//get pos feedback of joint and cartesian coordinate
	//----------------------------------------------------------------

	// using Mechanical decoupling matrix to change between Joins and Motors
	m_robot_module->kinematics.pfM2J(&m_robot_module->cart, &m_robot_module->joint[0], &m_robot_module->kinematics, m_cfg); 														// calc joint feedback

																																													//assign joint array to use for cartesian calc
	for (index = 0; index < m_cfg->joint_dim; index++)
	{
		joint[index] = (m_robot_module->joint[index]).joint_pos_fd;
	}
	//----------------------------------------------------------------
	//Transform Joint motion to Cartesian motion
	m_robot_module->kinematics.pfFK(&m_robot_module->cart, &m_robot_module->joint[0], &m_robot_module->kinematics, m_cfg, cart, joint);

	//assign cartesian pos feedback
	for (index = 0; index < m_cfg->cart_dim; index++)
	{
		m_robot_module->cart.cart_pos_fd[index] = cart[index];
	}
	return 0;
}
*/
/*
//------------------------------------------------------------------------------------------------------
// real time network interface which use for data send and receive   (command and feedback)
//-------------------------------------------------------------------------------------------------------
void robot_module_network_interface(robot_module *m_robot_module, robot_config_module* m_cfg, Uint8 station_id)
{
	Uint8 index = 0;
#if (SINGLE_TEST_ROBOT == 1)
	//this two value should come from or to real time network,now we use the network ,so ignore the define
	double command = 0;
	double feedback = 0;
#endif
	//-------------------------------------------------------------------------------------------------------

	if (station_id == 0)
	{
		/* Real-time network data 
		for (index = 0; index < m_cfg->joint_dim; index++)
		{
			//this part need consider the robot type .now we just focus 4 axis scara

			if ((m_robot_module->joint[index]).prm.mapJ2S < 2)
			{
				/* Servo on Master node. 
#if (SINGLE_TEST_ROBOT == 1)
				//receive the feedback
				(m_robot_module->joint[index]).servo_pos_fb = feedback;
				//send the command
				command = (m_robot_module->joint[index]).servo_pos_cmd;
#endif
				//communicate with rtn,use station_id
				//tbd
				if (gSys_cfg.mode_type == NETWORK_MODE)
				{
					get_rtn_recv_data(0, m_robot_module->joint[index].prm.mapJ2S + 1, &(m_robot_module->joint[index]).servo_pos_fb);
					set_rtn_send_data(0, m_robot_module->joint[index].prm.mapJ2S, (m_robot_module->joint[index]).servo_pos_cmd);
					get_rtn_recv_data(0, m_robot_module->joint[index].prm.mapJ2S + 3, &m_robot_module->joint[index].status_word);
					set_rtn_send_data(0, m_robot_module->joint[index].prm.mapJ2S + 2, m_robot_module->joint[index].ctrl_word);
					set_rtn_send_data(0, m_robot_module->joint[index].prm.mapJ2S + 4, gRobot.robot_dev[0].module.trajectory.abs_time);
				}
			}
			else
			{/* Servo on Slave node. 
#if (SINGLE_TEST_ROBOT == 1)
			 //receive the feedback
				(m_robot_module->joint[index]).servo_pos_fb = feedback;
				//send the command
				command = (m_robot_module->joint[index]).servo_pos_cmd;
#endif
				//communicate with rtn,use station_id
				//tbd
				if (gSys_cfg.mode_type == NETWORK_MODE)
				{
					get_rtn_recv_data(1, m_robot_module->joint[index].prm.mapJ2S - 1, &(m_robot_module->joint[index]).servo_pos_fb);
					set_rtn_send_data(1, m_robot_module->joint[index].prm.mapJ2S - 2, (m_robot_module->joint[index]).servo_pos_cmd);
					get_rtn_recv_data(1, m_robot_module->joint[index].prm.mapJ2S + 1, &m_robot_module->joint[index].status_word);
					set_rtn_send_data(1, m_robot_module->joint[index].prm.mapJ2S + 0, m_robot_module->joint[index].ctrl_word);
					set_rtn_send_data(1, m_robot_module->joint[index].prm.mapJ2S + 2, gRobot.robot_dev[0].module.trajectory.abs_time);
				}
			}
		}
	}
	else
	{

	}


	return;
}

*/
