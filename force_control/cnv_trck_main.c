
//#include "motion_planning_3rd.h"
#include "stdio.h"
//#include "trajectory_generator.h"
#include "matrix.h"
//----------------------------------------------------------
#include "robot_ctl_utility.h"
#include "robot_config.h"
//----------------------------------------------------------

#include "joint_module.h"
#include "cartesian_module.h"

#include "kinematics.h"

//----------------------------------------------------------

#include "robot_module.h"
//----------------------------------------------------------

//----------------------------------------------------------

#include "robot_mode.h"
//----------------------------------------------------------
#include "robot_device.h"
//----------------------------------------------------------

#include "robot_app.h"

#define SAMPLE_TIME		0.001

#define test_var       1
FILE* cnv_fp1;
FILE* cnv_fp2;

int16 cnv_trck_mode()
{



	return 0;
}
void main_cnv1()
{
	Uint8 id = 0;
	InitRobot(&gRobot);

	trajectory_module* p_trajectory = &gRobot.robot_dev[0].module.trajectory;
	cartesian_module* p_cart = &gRobot.robot_dev[0].module.cart;
	joint_module* p_joint = &gRobot.robot_dev[0].module.joint;
	robot_config_module* p_config = &gRobot.robot_dev[0].cfg;

	p_trajectory->motion_block_new_insert->pose.arc_lenth_trans = 10;
	p_trajectory->motion_block_new_insert->pose.pose_start[0] = 10;
	p_trajectory->motion_block_new_insert->pose.pose_start[1] = 20;
	p_trajectory->motion_block_new_insert->pose.pose_start[2] = 30;
	p_trajectory->motion_block_new_insert->pose.pose_start[3] = 40;
	p_trajectory->motion_block_new_insert->pose.pose_start[4] = 50;
	p_trajectory->motion_block_new_insert->pose.pose_start[5] = 60;
	p_trajectory->motion_block_new_insert->pose.pose_end[0] = 0;
	p_trajectory->motion_block_new_insert->pose.pose_end[1] = 1;
	p_trajectory->motion_block_new_insert->pose.pose_end[2] = 2;
	p_trajectory->motion_block_new_insert->pose.pose_end[3] = 3;
	p_trajectory->motion_block_new_insert->pose.pose_end[4] = 4;
	p_trajectory->motion_block_new_insert->pose.pose_end[5] = 5;

	p_trajectory->motion_block_new_insert->pose.vel_start[0] = 10;
	p_trajectory->motion_block_new_insert->pose.vel_start[1] = 20;
	p_trajectory->motion_block_new_insert->pose.vel_start[2] = 30;
	p_trajectory->motion_block_new_insert->pose.vel_start[3] = 40;
	p_trajectory->motion_block_new_insert->pose.vel_end[0]   = 0;
	p_trajectory->motion_block_new_insert->pose.vel_end[1]   = 1;
	p_trajectory->motion_block_new_insert->pose.vel_end[2]   = 2;
	p_trajectory->motion_block_new_insert->pose.vel_end[3]   = 3;

	p_trajectory->motion_block_new_insert->motion_type = TRAJECTORY_MODULE_MOTION_TYPE_LIN;

	//motion_block.pose.vel_start[0] = 0;
	//motion_block.pose.vel_start[1] = 0;
	//motion_block.pose.vel_start[2] = 0;
	//motion_block.pose.vel_start[3] = 0;
	//motion_block.pose.vel_end[0] = 0;
	//motion_block.pose.vel_end[1] = 0;
	//motion_block.pose.vel_end[2] = 0;
	//motion_block.pose.vel_end[3] = 0;

	p_trajectory->motion_block_new_insert->profile.profile_sample_time = SAMPLE_TIME;
	p_trajectory->period_time										   = SAMPLE_TIME;

	cnv_fp1	= fopen("C://Users//hqi//Desktop//debugging_data//cnv_rbt_cart_pos_cmd", "w+");
	cnv_fp2 = fopen("C://Users//hqi//Desktop//debugging_data//cnv_rbt_cart_vel_cmd", "w+");

	calculate_motion_profile_of_cnv_rbt(p_trajectory->motion_block_calculate, COORD_TYPE_CART_TRANS, 0);

	while (p_trajectory->motion_block_execute->state == TRAJECTORY_MODULE_MOTION_BLOCK_STATE_RUNNING)
	{
		cnv_rbt_traj_gen(p_trajectory, p_cart, p_joint, p_config, NULL, id);
	}
	fclose(cnv_fp1);
	return;
}
