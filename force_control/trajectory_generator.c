/*
* trajectory_generator.c
*
*  Created on: mar 15, 2018
*      Author: root
*/

//#include "robot_config.h"
//#include "cartesian_module.h"
//#include "joint_module.h"
//#include "trajectory_generator.h"
#include "motion_planning_3rd.h"
#include "robot_app.h"
#include "windows.h"
#include <stdio.h>
#include <stdlib.h>
//----------------------------------------------------------------------------------------
//input parameter 1 : point to buffer start
//input parameter 1 : point to buffer of current pos
motion_block* get_next_motion_block(const motion_block* buffer, const motion_block* current)
{
	motion_block* next = (motion_block*)current;
	Uint8 counter = 0;
	//----------------------------------------------
	//get current id
	counter = current->motion_block_id;
	counter++;
	//----------------------------------------------
	if (counter >= MOTION_MODULE_DEFAULT_MOTION_BLOCK_SIZE)
	{
		//back to the first motion block
		next = (motion_block*)buffer;
	}
	else
	{
		//point to the next motion block
		next = (motion_block*)(current + 1);
	}
	return next;
}
//----------------------------------------------------------------------------------------
motion_block* get_last_motion_block(const motion_block* buffer, const motion_block* current)
{
	motion_block* last = (motion_block*)current;
	Uint8 counter = 0;
	//----------------------------------------------
	counter = current->motion_block_id;
	counter--;
	//----------------------------------------------
	if (counter < 0)
	{
		//back to the buffer motion block end
		counter = MOTION_MODULE_DEFAULT_MOTION_BLOCK_SIZE - 1;
		last = (motion_block*)(buffer + counter);
	}
	else
	{
		//point to the last motion block
		last = (motion_block*)(current - 1);
	}
	return last;
}
//----------------------------------------------------------------------------------
int16 InitTrajectoryModule(trajectory_module *m_traj, robot_config_module *p_config, Uint8 id)
{
	Uint8 counter = 0;

	//---------------------------------------------------------------------
	m_traj->current_time = 0;
	//m_traj->abs_time = 0;
	m_traj->motion_profile[0] = 100;
	m_traj->prf_cart_rot[0] = 3.0 / 3.1415926 * 180.0;
	m_traj->motion_profile[1] = 25;
	m_traj->prf_cart_rot[1] = 1.0 / 3.1415926 * 180.0;
	m_traj->motion_profile[3] = 25;
	m_traj->prf_cart_rot[3] = 1.0 / 3.1415926 * 180.0;
	m_traj->motion_profile[2] = 0.1;
	m_traj->prf_cart_rot[2] = 0.1;
	m_traj->motion_profile[4] = 0.1;
	m_traj->prf_cart_rot[4] = 0.1;
	m_traj->motion_profile[5] = 0.001;
	m_traj->prf_cart_rot[5] = 0.001;
	//---------------------------------------------------------------------
	m_traj->pfInitTrajectoryModule = InitTrajectoryModule;
	m_traj->pfTrajSynCmdWithFd = TrajSynCmdWithFd;
	m_traj->pfCalcMotionProfile = CalcMotionProfile;
	m_traj->pfInsertNewMotionBlock = InsertNewMotionBlock;

	//---------------------------------------------------------------------
	m_traj->motion_block_execute = (motion_block*)&(m_traj->motion_block_buffer[0]);
	m_traj->motion_block_new_insert = (motion_block*)&(m_traj->motion_block_buffer[0]);
	m_traj->motion_block_calculate = (motion_block*)&(m_traj->motion_block_buffer[0]);

	//---------------------------------------------------------------------

	for (counter = 0; counter < MOTION_MODULE_DEFAULT_MOTION_BLOCK_SIZE; counter++)
	{
		//---------------------------------------------------------------------
		// initial pose
		//---------------------------------------------------------------------
		matxx_malloc(&((m_traj->motion_block_buffer[counter]).pose.joint_start_vector), p_config->joint_dim, 1);
		matxx_malloc(&((m_traj->motion_block_buffer[counter]).pose.joint_end_vector), p_config->joint_dim, 1);
		matxx_malloc(&((m_traj->motion_block_buffer[counter]).pose.joint_unit_vector), p_config->joint_dim, 1);
		matxx_malloc(&((m_traj->motion_block_buffer[counter]).pose.joint_pos), p_config->joint_dim, 1);
		matxx_malloc(&((m_traj->motion_block_buffer[counter]).pose.joint_temp_vector), p_config->joint_dim, 1);
		//---------------------------------------------------------------------
		/* rotation matrix */
		VECTOR3(&((m_traj->motion_block_buffer[counter]).pose.axis_vector));
		VECTOR3(&((m_traj->motion_block_buffer[counter]).pose.unit_vector));
		VECTOR3(&((m_traj->motion_block_buffer[counter]).pose.center_vector));
		SQUARE3(&((m_traj->motion_block_buffer[counter]).pose.rotation_matxx));
		SQUARE3(&((m_traj->motion_block_buffer[counter]).pose.d_rotation_matxx));
		SQUARE3(&((m_traj->motion_block_buffer[counter]).pose.rot_start));
		SQUARE3(&((m_traj->motion_block_buffer[counter]).pose.rot_end));
		SQUARE3(&((m_traj->motion_block_buffer[counter]).pose.rot_tmp));
		SQUARE3(&((m_traj->motion_block_buffer[counter]).pose.rot_tmp1));
		VECTOR3(&m_traj->motion_block_buffer[counter].pose.cartesian_xyz_vel);
		VECTOR3(&m_traj->motion_block_buffer[counter].pose.tmp31);
		VECTOR3(&m_traj->motion_block_buffer[counter].pose.P0);
		VECTOR3(&m_traj->motion_block_buffer[counter].pose.P0P1);
		VECTOR3(&m_traj->motion_block_buffer[counter].pose.P0P2);
		VECTOR3(&m_traj->motion_block_buffer[counter].pose.u);
		VECTOR3(&m_traj->motion_block_buffer[counter].pose.v);
		VECTOR3(&m_traj->motion_block_buffer[counter].pose.w);

		//---------------------------------------------------------------------
		/* Cartesian Commands */
		VECTOR3(&((m_traj->motion_block_buffer[counter]).pose.cartesian_xyz));
		//---------------------------------------------------------------------
		// motion block ID
		(m_traj->motion_block_buffer[counter]).motion_block_id = counter;
		//---------------------------------------------------------------------
		(m_traj->motion_block_buffer[counter]).state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_IDL;
		(m_traj->motion_block_buffer[counter]).motion_type = TRAJECTORY_MODULE_MOTION_TYPE_P2P;
		(m_traj->motion_block_buffer[counter]).delay_enable = 0;
		(m_traj->motion_block_buffer[counter]).delay_time = 0;
	}
	// mallock motion block tmp
	matxx_malloc(&m_traj->motion_block_tmp.pose.cartesian_xyz, 3, 1);
	matxx_malloc(&(m_traj->motion_block_tmp.pose.joint_start_vector), p_config->joint_dim, 1);
	matxx_malloc(&(m_traj->motion_block_tmp.pose.joint_end_vector), p_config->joint_dim, 1);
	matxx_malloc(&(m_traj->motion_block_tmp.pose.joint_unit_vector), p_config->joint_dim, 1);
	matxx_malloc(&(m_traj->motion_block_tmp.pose.joint_pos), p_config->joint_dim, 1);
	matxx_malloc(&(m_traj->motion_block_tmp.pose.joint_temp_vector), p_config->joint_dim, 1);
	//---------------------------------------------------------------------
	/* rotation matrix */
	VECTOR3(&(m_traj->motion_block_tmp.pose.axis_vector));
	VECTOR3(&(m_traj->motion_block_tmp.pose.unit_vector));
	VECTOR3(&(m_traj->motion_block_tmp.pose.center_vector));
	SQUARE3(&(m_traj->motion_block_tmp.pose.rotation_matxx));
	SQUARE3(&(m_traj->motion_block_tmp.pose.d_rotation_matxx));
	SQUARE3(&(m_traj->motion_block_tmp.pose.rot_start));
	SQUARE3(&(m_traj->motion_block_tmp.pose.rot_end));
	SQUARE3(&(m_traj->motion_block_tmp.pose.rot_tmp));
	SQUARE3(&(m_traj->motion_block_tmp.pose.rot_tmp1));
	VECTOR3(&m_traj->motion_block_tmp.pose.cartesian_xyz_vel);
	VECTOR3(&m_traj->motion_block_tmp.pose.tmp31);
	VECTOR3(&m_traj->motion_block_tmp.pose.P0);
	VECTOR3(&m_traj->motion_block_tmp.pose.P0P1);
	VECTOR3(&m_traj->motion_block_tmp.pose.P0P2);
	VECTOR3(&m_traj->motion_block_tmp.pose.u);
	VECTOR3(&m_traj->motion_block_tmp.pose.v);
	VECTOR3(&m_traj->motion_block_tmp.pose.w);

	return 0;
}
//----------------------------------------------------------------------------------
int16 TrajSynCmdWithFd(trajectory_module * m_traj, robot_config_module *p_config, cartesian_module* p_cart, joint_module* p_joint)
{
	Uint8 			i;
	Uint8 			index = 0;
	motion_block* 	p_motion = m_traj->motion_block_buffer;
	//------------------------------------------------------------------
	for (i = 0; i< MOTION_MODULE_DEFAULT_MOTION_BLOCK_SIZE; ++i)
	{
		for (index = 0; index < p_config->cart_dim; index++)
		{
			p_motion[i].pose.pose_start[index] = (p_cart)->cart_pos_fd[index];
			p_motion[i].pose.pose_end[index] = (p_cart)->cart_pos_fd[index];
		}
		//---------------------------------------------------
		for (index = 0; index < p_config->joint_dim; index++)
		{
			matxx_set_element(&p_motion[i].pose.joint_start_vector, index, 0, &(p_joint[index]).joint_pos_fd);
		}

		matxx_copy(&p_motion[i].pose.joint_start_vector, &p_motion[i].pose.joint_end_vector);

		p_motion[i].delay_enable = 0;
		p_motion[i].delay_time = 0;
		p_motion[i].state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_IDL;
	}
	//------------------------------------------------------------------
	/* Synchronize motion block */
	m_traj->motion_block_execute = m_traj->motion_block_new_insert;
	m_traj->motion_block_calculate = m_traj->motion_block_new_insert;

	return 0;
}
//------------------------------------------------------------------
void calculate_acceleration_from_profile(double t,
	double jerk,
	double acc_init,
	double* acc_t)
{
	*acc_t = acc_init;
	*acc_t += jerk * t;
	return;
}
//------------------------------------------------------------------
void calculate_velocity_from_profile(double t,
	double jerk,
	double acc_init,
	double vel_init,
	double* vel_t)
{
	/*
	* v =  v0 + a0 * t + 0.5 * jerk * t * t
	*   = v0 + (a0  + 0.5 * jerk * t )* t
	* */
	*vel_t = acc_init;
	*vel_t += jerk * t * 0.5;
	*vel_t *= t;
	*vel_t += vel_init;
	return;
}
//------------------------------------------------------------------
void calculate_position_from_profile(double t,
	double jerk,
	double acc_init,
	double vel_init,
	double pos_init,
	double* pos_t)
{
	/*
	* s
	* =  s0+ v0*t + a0*t*t*1/2  + jerk*t*t*t*1/6
	* =  s0+ (v0 + a0*t*1/2  + jerk*t*t*1/6)*t
	* =  s0+ (v0 + (a0  + jerk*t*1/3)*t*1/2 )*t
	* */
	*pos_t = acc_init;
	*pos_t += jerk * t * 0.333333333333333;
	*pos_t *= (t*0.5);
	*pos_t += vel_init;
	*pos_t *= t;
	*pos_t += pos_init;
	return;
}
//------------------------------------------------------------------
void calculate_motion_profile_with_limited_jerk(motion_block* p_motion, Uint8 type, Uint8 joint_id)
{
	MOTION_PLANNING_3RD profile_memory;
	MOTION_PLANNING_PRM mp_rpm;

	motion_trajectory* p_traj_cart_trans = &p_motion->trajectory;
	motion_trajectory* p_traj_cart_rot = &p_motion->traj_cart_rot;
	motion_trajectory* p_traj_joint = &p_motion->traj_joint[joint_id];

	double time_interval = 0.0;
	Uint8  counter = 0;

	//------------------------------------------------------------
	if (fabs(p_motion->pose.arc_length) < MOTION_MODULE_CONSTANT_MIN_POSITIVE)
	{
		profile_memory.Jacc = 0.0;
		profile_memory.Jdec = 0.0;
		profile_memory.t_all = 0.0;
		profile_memory.t_acc_ceil = 0.0;
		profile_memory.t_dec_ceil = 0.0;
		profile_memory.t1_ceil = 0.0;
		profile_memory.t2_ceil = 0.0;
		profile_memory.t3_ceil = 0.0;

		//----------------------------------------------
		if (type == COORD_TYPE_CART_TRANS)
		{
			//----------------------------------------------
			p_traj_cart_trans->jerk_acc = profile_memory.Jacc;
			p_traj_cart_trans->jerk_dec = profile_memory.Jdec;
			p_traj_cart_trans->total_interval = profile_memory.t_all; /* Total interval  */
																	  //----------------------------------------------
			for (counter = 0; counter < 8; ++counter)
			{
				p_traj_cart_trans->time_series[counter] = 0.0;
				p_traj_cart_trans->jerk_series[counter] = 0.0;
				p_traj_cart_trans->acc_series[counter] = 0.0;
				p_traj_cart_trans->vel_series[counter] = 0.0;
				p_traj_cart_trans->pos_series[counter] = 0.0;
			}
			//----------------------------------------------
		}
		else if (type == COORD_TYPE_CART_ROT)
		{
			//----------------------------------------------
			p_traj_cart_rot->jerk_acc = profile_memory.Jacc;
			p_traj_cart_rot->jerk_dec = profile_memory.Jdec;
			p_traj_cart_rot->total_interval = profile_memory.t_all; /* Total interval  */
																	//----------------------------------------------
			for (counter = 0; counter < 8; ++counter)
			{
				p_traj_cart_rot->time_series[counter] = 0.0;
				p_traj_cart_rot->jerk_series[counter] = 0.0;
				p_traj_cart_rot->acc_series[counter] = 0.0;
				p_traj_cart_rot->vel_series[counter] = 0.0;
				p_traj_cart_rot->pos_series[counter] = 0.0;
			}
			//----------------------------------------------
		}
		else if (type == COORD_TYPE_JOINT)
		{
			//----------------------------------------------
			p_traj_joint->jerk_acc = profile_memory.Jacc;
			p_traj_joint->jerk_dec = profile_memory.Jdec;
			p_traj_joint->total_interval = profile_memory.t_all; /* Total interval  */
																 //----------------------------------------------
			for (counter = 0; counter < 8; ++counter)
			{
				p_traj_joint->time_series[counter] = 0.0;
				p_traj_joint->jerk_series[counter] = 0.0;
				p_traj_joint->acc_series[counter] = 0.0;
				p_traj_joint->vel_series[counter] = 0.0;
				p_traj_joint->pos_series[counter] = 0.0;
			}
			//----------------------------------------------
		}
		else
		{

		}
		return;
	}
	//------------------------------------------------------------
	//cartesian coordinate motion profile
	if (type == COORD_TYPE_CART_TRANS)
	{
		//------------------------------------------------------------
		mp_rpm.pos = p_motion->pose.arc_length;
		mp_rpm.spd_max = p_motion->profile.profile_vel;
		mp_rpm.acc_max = p_motion->profile.profile_acc;
		mp_rpm.t_acc = p_motion->profile.profile_tacc;
		mp_rpm.dec_max = p_motion->profile.profile_dec;
		mp_rpm.t_dec = p_motion->profile.profile_tdec;

		motion_planning_3rd(&mp_rpm, &profile_memory, p_motion->profile.profile_sample_time);

		//------------------------------------------------------------

		p_traj_cart_trans->jerk_acc = profile_memory.Jacc;
		p_traj_cart_trans->jerk_dec = profile_memory.Jdec;
		p_traj_cart_trans->total_interval = profile_memory.t_all_d; /* Total interval  */
																	//------------------------------------------------------------
																	//calculate time
																	/*
																	* ACC:[0 Tacc)
																	* -ACC:[Tacc+T1 Tacc+T1+Tacc)
																	* -DEC:[Tacc+T1+Tacc+T3 Tacc+T1+Tacc+T3+Tdec)
																	* DEC:[Tacc+T1+Tacc+T3+Tdec+T2 Tacc+T1+Tacc+T3+Tdec+T2+Tdec)
																	* */
		p_traj_cart_trans->time_series[0] = 0;
		p_traj_cart_trans->time_series[1] = (Uint32)profile_memory.t_acc_ceil;
		p_traj_cart_trans->time_series[2] = (Uint32)(profile_memory.t_acc_ceil + profile_memory.t1_ceil);
		p_traj_cart_trans->time_series[3] = p_traj_cart_trans->time_series[2] + (Uint32)(profile_memory.t_acc_ceil);
		p_traj_cart_trans->time_series[4] = p_traj_cart_trans->time_series[3] + (Uint32)(profile_memory.t3_ceil);
		p_traj_cart_trans->time_series[5] = p_traj_cart_trans->time_series[4] + (Uint32)(profile_memory.t_dec_ceil);
		p_traj_cart_trans->time_series[6] = p_traj_cart_trans->time_series[5] + (Uint32)(profile_memory.t2_ceil);
		p_traj_cart_trans->time_series[7] = p_traj_cart_trans->time_series[6] + (Uint32)(profile_memory.t_dec_ceil);
		//------------------------------------------------------------

		// compute initial states
		// Jerk States corresponding to the different time
		p_traj_cart_trans->jerk_series[0] = p_traj_cart_trans->jerk_acc;
		p_traj_cart_trans->jerk_series[1] = 0.0;
		p_traj_cart_trans->jerk_series[2] = -p_traj_cart_trans->jerk_acc;
		p_traj_cart_trans->jerk_series[3] = 0.0;
		p_traj_cart_trans->jerk_series[4] = -p_traj_cart_trans->jerk_dec;
		p_traj_cart_trans->jerk_series[5] = 0.0;
		p_traj_cart_trans->jerk_series[6] = p_traj_cart_trans->jerk_dec;
		p_traj_cart_trans->jerk_series[7] = 0.0;


		// initial states for acc, dec, vel, and pos
		p_traj_cart_trans->acc_series[0] = 0.0;
		p_traj_cart_trans->vel_series[0] = 0.0;
		p_traj_cart_trans->pos_series[0] = 0.0;

		for (counter = 1; counter < 8; ++counter)
		{
			//get time n and mutiply the sample time to get the real time
			time_interval = p_traj_cart_trans->time_series[counter] - p_traj_cart_trans->time_series[counter - 1];
			time_interval *= p_motion->profile.profile_sample_time;

			//calculate acc
			calculate_acceleration_from_profile(time_interval,
				p_traj_cart_trans->jerk_series[counter - 1],
				p_traj_cart_trans->acc_series[counter - 1],
				&p_traj_cart_trans->acc_series[counter]);
			//calculate vel
			calculate_velocity_from_profile(time_interval,
				p_traj_cart_trans->jerk_series[counter - 1],
				p_traj_cart_trans->acc_series[counter - 1],
				p_traj_cart_trans->vel_series[counter - 1],
				&p_traj_cart_trans->vel_series[counter]);
			//calculate pos
			calculate_position_from_profile(time_interval,
				p_traj_cart_trans->jerk_series[counter - 1],
				p_traj_cart_trans->acc_series[counter - 1],
				p_traj_cart_trans->vel_series[counter - 1],
				p_traj_cart_trans->pos_series[counter - 1],
				&p_traj_cart_trans->pos_series[counter]);
		}
	}
	else if (type == COORD_TYPE_CART_ROT)
	{
		//------------------------------------------------------------
		mp_rpm.pos = p_motion->pose.arc_length;
		mp_rpm.spd_max = p_motion->prf_cart_rot.profile_vel;
		mp_rpm.acc_max = p_motion->prf_cart_rot.profile_acc;
		mp_rpm.t_acc = p_motion->prf_cart_rot.profile_tacc;
		mp_rpm.dec_max = p_motion->prf_cart_rot.profile_dec;
		mp_rpm.t_dec = p_motion->prf_cart_rot.profile_tdec;

		motion_planning_3rd(&mp_rpm, &profile_memory, p_motion->prf_cart_rot.profile_sample_time);

		//------------------------------------------------------------

		p_traj_cart_rot->jerk_acc = profile_memory.Jacc;
		p_traj_cart_rot->jerk_dec = profile_memory.Jdec;
		p_traj_cart_rot->total_interval = profile_memory.t_all_d; /* Total interval  */
																  //------------------------------------------------------------
																  //calculate time
																  /*
																  * ACC:[0 Tacc)
																  * -ACC:[Tacc+T1 Tacc+T1+Tacc)
																  * -DEC:[Tacc+T1+Tacc+T3 Tacc+T1+Tacc+T3+Tdec)
																  * DEC:[Tacc+T1+Tacc+T3+Tdec+T2 Tacc+T1+Tacc+T3+Tdec+T2+Tdec)
																  * */
		p_traj_cart_rot->time_series[0] = 0;
		p_traj_cart_rot->time_series[1] = (Uint32)profile_memory.t_acc_ceil;
		p_traj_cart_rot->time_series[2] = (Uint32)(profile_memory.t_acc_ceil + profile_memory.t1_ceil);
		p_traj_cart_rot->time_series[3] = p_traj_cart_rot->time_series[2] + (Uint32)(profile_memory.t_acc_ceil);
		p_traj_cart_rot->time_series[4] = p_traj_cart_rot->time_series[3] + (Uint32)(profile_memory.t3_ceil);
		p_traj_cart_rot->time_series[5] = p_traj_cart_rot->time_series[4] + (Uint32)(profile_memory.t_dec_ceil);
		p_traj_cart_rot->time_series[6] = p_traj_cart_rot->time_series[5] + (Uint32)(profile_memory.t2_ceil);
		p_traj_cart_rot->time_series[7] = p_traj_cart_rot->time_series[6] + (Uint32)(profile_memory.t_dec_ceil);
		//------------------------------------------------------------

		// compute initial states
		// Jerk States corresponding to the different time
		p_traj_cart_rot->jerk_series[0] = p_traj_cart_rot->jerk_acc;
		p_traj_cart_rot->jerk_series[1] = 0.0;
		p_traj_cart_rot->jerk_series[2] = -p_traj_cart_rot->jerk_acc;
		p_traj_cart_rot->jerk_series[3] = 0.0;
		p_traj_cart_rot->jerk_series[4] = -p_traj_cart_rot->jerk_dec;
		p_traj_cart_rot->jerk_series[5] = 0.0;
		p_traj_cart_rot->jerk_series[6] = p_traj_cart_rot->jerk_dec;
		p_traj_cart_rot->jerk_series[7] = 0.0;


		// initial states for acc, dec, vel, and pos
		p_traj_cart_rot->acc_series[0] = 0.0;
		p_traj_cart_rot->vel_series[0] = 0.0;
		p_traj_cart_rot->pos_series[0] = 0.0;

		for (counter = 1; counter < 8; ++counter)
		{
			//get time n and mutiply the sample time to get the real time
			time_interval = p_traj_cart_rot->time_series[counter] - p_traj_cart_rot->time_series[counter - 1];
			time_interval *= p_motion->prf_cart_rot.profile_sample_time;

			//calculate acc
			calculate_acceleration_from_profile(time_interval,
				p_traj_cart_rot->jerk_series[counter - 1],
				p_traj_cart_rot->acc_series[counter - 1],
				&p_traj_cart_rot->acc_series[counter]);
			//calculate vel
			calculate_velocity_from_profile(time_interval,
				p_traj_cart_rot->jerk_series[counter - 1],
				p_traj_cart_rot->acc_series[counter - 1],
				p_traj_cart_rot->vel_series[counter - 1],
				&p_traj_cart_rot->vel_series[counter]);
			//calculate pos
			calculate_position_from_profile(time_interval,
				p_traj_cart_rot->jerk_series[counter - 1],
				p_traj_cart_rot->acc_series[counter - 1],
				p_traj_cart_rot->vel_series[counter - 1],
				p_traj_cart_rot->pos_series[counter - 1],
				&p_traj_cart_rot->pos_series[counter]);
		}
	}
	else if (type == COORD_TYPE_JOINT)
	{
		//------------------------------------------------------------
		mp_rpm.pos = p_motion->pose.arc_length;
		mp_rpm.spd_max = p_motion->prf_joint[joint_id].profile_vel;
		mp_rpm.acc_max = p_motion->prf_joint[joint_id].profile_acc;
		mp_rpm.t_acc = p_motion->prf_joint[joint_id].profile_tacc;
		mp_rpm.dec_max = p_motion->prf_joint[joint_id].profile_dec;
		mp_rpm.t_dec = p_motion->prf_joint[joint_id].profile_tdec;

		motion_planning_3rd(&mp_rpm, &profile_memory, p_motion->prf_joint[joint_id].profile_sample_time);

		//------------------------------------------------------------

		p_traj_joint->jerk_acc = profile_memory.Jacc;
		p_traj_joint->jerk_dec = profile_memory.Jdec;
		p_traj_joint->total_interval = profile_memory.t_all_d; /* Total interval  */
															   //------------------------------------------------------------
															   //calculate time
															   /*
															   * ACC:[0 Tacc)
															   * -ACC:[Tacc+T1 Tacc+T1+Tacc)
															   * -DEC:[Tacc+T1+Tacc+T3 Tacc+T1+Tacc+T3+Tdec)
															   * DEC:[Tacc+T1+Tacc+T3+Tdec+T2 Tacc+T1+Tacc+T3+Tdec+T2+Tdec)
															   * */
		p_traj_joint->time_series[0] = 0;
		p_traj_joint->time_series[1] = (Uint32)profile_memory.t_acc_ceil;
		p_traj_joint->time_series[2] = (Uint32)(profile_memory.t_acc_ceil + profile_memory.t1_ceil);
		p_traj_joint->time_series[3] = p_traj_joint->time_series[2] + (Uint32)(profile_memory.t_acc_ceil);
		p_traj_joint->time_series[4] = p_traj_joint->time_series[3] + (Uint32)(profile_memory.t3_ceil);
		p_traj_joint->time_series[5] = p_traj_joint->time_series[4] + (Uint32)(profile_memory.t_dec_ceil);
		p_traj_joint->time_series[6] = p_traj_joint->time_series[5] + (Uint32)(profile_memory.t2_ceil);
		p_traj_joint->time_series[7] = p_traj_joint->time_series[6] + (Uint32)(profile_memory.t_dec_ceil);
		//------------------------------------------------------------

		// compute initial states
		// Jerk States corresponding to the different time
		p_traj_joint->jerk_series[0] = p_traj_joint->jerk_acc;
		p_traj_joint->jerk_series[1] = 0.0;
		p_traj_joint->jerk_series[2] = -p_traj_joint->jerk_acc;
		p_traj_joint->jerk_series[3] = 0.0;
		p_traj_joint->jerk_series[4] = -p_traj_joint->jerk_dec;
		p_traj_joint->jerk_series[5] = 0.0;
		p_traj_joint->jerk_series[6] = p_traj_joint->jerk_dec;
		p_traj_joint->jerk_series[7] = 0.0;


		// initial states for acc, dec, vel, and pos
		p_traj_joint->acc_series[0] = 0.0;
		p_traj_joint->vel_series[0] = 0.0;
		p_traj_joint->pos_series[0] = 0.0;

		for (counter = 1; counter < 8; ++counter)
		{
			//get time n and mutiply the sample time to get the real time
			time_interval = p_traj_joint->time_series[counter] - p_traj_joint->time_series[counter - 1];
			time_interval *= p_motion->prf_joint[joint_id].profile_sample_time;

			//calculate acc
			calculate_acceleration_from_profile(time_interval,
				p_traj_joint->jerk_series[counter - 1],
				p_traj_joint->acc_series[counter - 1],
				&p_traj_joint->acc_series[counter]);
			//calculate vel
			calculate_velocity_from_profile(time_interval,
				p_traj_joint->jerk_series[counter - 1],
				p_traj_joint->acc_series[counter - 1],
				p_traj_joint->vel_series[counter - 1],
				&p_traj_joint->vel_series[counter]);
			//calculate pos
			calculate_position_from_profile(time_interval,
				p_traj_joint->jerk_series[counter - 1],
				p_traj_joint->acc_series[counter - 1],
				p_traj_joint->vel_series[counter - 1],
				p_traj_joint->pos_series[counter - 1],
				&p_traj_joint->pos_series[counter]);
		}
	}
	else
	{

	}
	return;
}

//------------------------------------------------------------------------
// 由于笛卡尔空间的初速度方向与位移方向不相等，所以必须分别在x,y,z方向进行运动规划。最后为了与0边界的梯形规划统一，将x,y,z方向的规划合成。
void calculate_motion_profile_of_cnv_rbt(motion_block* p_motion, Uint8 type, Uint8 sync_method)
{
	MOTION_PLANNING_3RD profile_memory;
	MOTION_PLANNING_NON_ZERO_PRM input;
	MOTION_PLANNING_PRM input_0;
	Uint8 counter = 0;
	double time_interval = 0;
	Uint8 trans_dim = 3;
	Uint8 dim = 4;
	Uint8 i = 0;
	double Tmax = 0;
	Uint8  Tmax_id = 0;
	double k = 0;
	double k2 = 0;
	double k3 = 0;
	double vmax_tmp = 0;
	double j1_tmp = 0;
	double j2_tmp = 0;
	Uint8 non_zero_boundary_flag = 0;
	Uint32* p_db = NULL;
	double theta_tmp	= 0;
	double pos = 0;
	double vs = 0;
	double ve = 0;

	motion_trajectory* p_traj = &p_motion->traj_cart_xyz_poseture[0];
	matxx_malloc(&p_motion->pose.rot_start, 3, 3);
	matxx_malloc(&p_motion->pose.rot_end, 3, 3);
	matxx_malloc(&p_motion->pose.rot_tmp, 3, 3);
	matxx_malloc(&p_motion->pose.rotation_matxx, 3, 3);
	matxx_malloc(&p_motion->pose.axis_vector, 3, 1);
	
	if (type != COORD_TYPE_CART_TRANS)
	{
		printf("传送带跟踪应该为笛卡尔空间直线运动\n");
		return;
	}
	input.spd_max = 100;
	input.acc_max = 25;
	input.t_acc = 0.1;
	input.max_itera_time = 30;
	input.scale = 1.0;
	input.secant_eps = 0.001;

	input_0.spd_max = 100;
	input_0.dec_max = 25;
	input_0.acc_max = 25;
	input_0.t_acc = 0.1;
	input_0.t_dec = 0.1;

	//===============================================================
	//在x,y,z方向分别进行规划
	//对始末姿态采用轴角方式规划
	xyz2r(&p_motion->pose.pose_start[3], &p_motion->pose.rot_start, 0);
	xyz2r(&p_motion->pose.pose_end[3], &p_motion->pose.rot_end, 0);
	//get the rotate matrix from  pose start to pose end  PB = R*PA --->R = PB*PA^-1
	//PA^-1 = PA^T
	matxx_transpose(&p_motion->pose.rot_start, &p_motion->pose.rot_tmp);
	matxx_multiply(&p_motion->pose.rot_end, &p_motion->pose.rot_tmp, &p_motion->pose.rotation_matxx);
	//-------------------------------------------------------------------
	//calculate the axis angle
	//-------------------------------------------------------------------
	p_motion->pose.angle_lenth = acos(0.5*((*(p_motion->pose.rotation_matxx.point + 0))[0] + \
		(*(p_motion->pose.rotation_matxx.point + 1))[1] + \
		(*(p_motion->pose.rotation_matxx.point + 2))[2] - 1));
	for (i = 0; i < dim; i++)
	{
		if (i < trans_dim)
		{
			input_0.pos = input.pos = p_motion->pose.pose_end[i] - p_motion->pose.pose_start[i];
			input.vs = p_motion->pose.vel_start[i];
			input.ve = p_motion->pose.vel_end[i];
		}
		else
		{
			//姿态的初末速度暂时设置为0
			input_0.pos = input.pos = p_motion->pose.angle_lenth;
			input.vs = 0;
			input.ve = 0;

			//-------------------------------------------------------------------
			//计算姿态轴角的轴。
			//-------------------------------------------------------------------
			if (sin(p_motion->pose.angle_lenth)< MOTION_MODULE_CONSTANT_MIN_POSITIVE)
			{
				//因为角度太小，轴不存在。
				p_motion->pose.axis_angle_flag = 0;
			}
			else
			{
				p_motion->pose.axis_angle_flag = 1;

				//common part ,1/(2×sin(angle))
				theta_tmp = 1.0 / (2 * sin(p_motion->pose.angle_lenth));

				//r32-r23
				(*(p_motion->pose.axis_vector.point + 0))[0] = theta_tmp*((*(p_motion->pose.rotation_matxx.point + 1))[2] - \
					(*(p_motion->pose.rotation_matxx.point + 2))[1]);
				//r13-r31
				(*(p_motion->pose.axis_vector.point + 0))[1] = theta_tmp*((*(p_motion->pose.rotation_matxx.point + 2))[0] - \
					(*(p_motion->pose.rotation_matxx.point + 0))[2]);
				//r21 - r12
				(*(p_motion->pose.axis_vector.point + 0))[2] = theta_tmp*((*(p_motion->pose.rotation_matxx.point + 0))[1] - \
					(*(p_motion->pose.rotation_matxx.point + 1))[0]);
			}

		}
		if (fabs(input.vs) < 1E-10&&fabs(input.ve) < 1E-10)
		{
			motion_planning_3rd(&input_0, &profile_memory, p_motion->profile.profile_sample_time);
		}
		else
		{
			// 在始末速度不为0的梯形规划中，必须用时间缩放。时间同步方法会导致错误
			non_zero_boundary_flag = 1;
			sync_method = CARTLINE_TYPE_SYNC_TIME_SCALE;

			if (motion_planning_nonzero(&input, &profile_memory, p_motion->profile.profile_sample_time) == -1)
			{
				printf("有始末速度的梯形规划失败，使用无始末速度的梯形规划\n");
				motion_planning_3rd(&input_0, &profile_memory, p_motion->profile.profile_sample_time);
			}

		}
		//find the id of max_T
		if (Tmax < profile_memory.t_all_d)
		{
			//get the time max joint id
			Tmax = profile_memory.t_all_d;
			Tmax_id = i;
		}

		p_traj[i].jerk_acc = profile_memory.Jacc;
		p_traj[i].jerk_dec = profile_memory.Jdec;
		p_traj[i].total_interval = profile_memory.t_all_d; /* Total interval  */
														   //------------------------------------------------------------
														   //calculate time
														   /*
														   * ACC:[0 Tacc)
														   * -ACC:[Tacc+T1 Tacc+T1+Tacc)
														   * -DEC:[Tacc+T1+Tacc+T3 Tacc+T1+Tacc+T3+Tdec)
														   * DEC:[Tacc+T1+Tacc+T3+Tdec+T2 Tacc+T1+Tacc+T3+Tdec+T2+Tdec)
														   * */
		p_traj[i].time_series[0] = 0;
		p_traj[i].time_series[1] = (Uint32)profile_memory.t_acc_ceil;
		p_traj[i].time_series[2] = (Uint32)(profile_memory.t_acc_ceil + profile_memory.t1_ceil);
		p_traj[i].time_series[3] = p_traj[i].time_series[2] + (Uint32)(profile_memory.t_acc_ceil);
		p_traj[i].time_series[4] = p_traj[i].time_series[3] + (Uint32)(profile_memory.t3_ceil);
		p_traj[i].time_series[5] = p_traj[i].time_series[4] + (Uint32)(profile_memory.t_dec_ceil);
		p_traj[i].time_series[6] = p_traj[i].time_series[5] + (Uint32)(profile_memory.t2_ceil);
		p_traj[i].time_series[7] = p_traj[i].time_series[6] + (Uint32)(profile_memory.t_dec_ceil);
		//------------------------------------------------------------

		// compute initial states
		// Jerk States corresponding to the different time
		p_traj[i].jerk_series[0] = p_traj[i].jerk_acc;
		p_traj[i].jerk_series[1] = 0.0;
		p_traj[i].jerk_series[2] = -p_traj[i].jerk_acc;
		p_traj[i].jerk_series[3] = 0.0;
		p_traj[i].jerk_series[4] = -p_traj[i].jerk_dec;
		p_traj[i].jerk_series[5] = 0.0;
		p_traj[i].jerk_series[6] = p_traj[i].jerk_dec;
		p_traj[i].jerk_series[7] = 0.0;


		// initial states for acc, dec, vel, and pos
		p_traj[i].acc_series[0] = 0.0;
		p_traj[i].vel_series[0] = input.vs;
		if (i < trans_dim)
		{
			p_traj[i].pos_series[0] = p_motion->pose.pose_start[i];
		}
		else
		{
			p_traj[i].pos_series[0] = 0;
		}

		for (counter = 1; counter < 8; ++counter)
		{
			//get time n and mutiply the sample time to get the real time
			time_interval = p_traj[i].time_series[counter] - p_traj[i].time_series[counter - 1];
			time_interval *= p_motion->profile.profile_sample_time;

			//calculate acc
			calculate_acceleration_from_profile(time_interval,
				p_traj[i].jerk_series[counter - 1],
				p_traj[i].acc_series[counter - 1],
				&p_traj[i].acc_series[counter]);
			//calculate vel
			calculate_velocity_from_profile(time_interval,
				p_traj[i].jerk_series[counter - 1],
				p_traj[i].acc_series[counter - 1],
				p_traj[i].vel_series[counter - 1],
				&p_traj[i].vel_series[counter]);
			//calculate pos
			calculate_position_from_profile(time_interval,
				p_traj[i].jerk_series[counter - 1],
				p_traj[i].acc_series[counter - 1],
				p_traj[i].vel_series[counter - 1],
				p_traj[i].pos_series[counter - 1],
				&p_traj[i].pos_series[counter]);
		}
	}

	//===============================================================
	// 通过缩放等方法使得关节轴之间或笛卡尔轴之间运动时间相同。
	//---------------------------------------------------------------------------------------
	//now we have know that which joint the max time ,so we use this time intervals to calculate other joint
	//we have two methods, one is scale the time ,do not let every section time is the same as the most longest time joint
	//---------------------------------------------------------------------------------------
	if (sync_method == CARTLINE_TYPE_SYNC_TIME_SCALE)
	{
		//calculate the every joint's parameter for fitting the max T;
		for (i = 0; i < dim; i++)
		{
			//假如是最大的关节，不需要再次计算
			if (i == Tmax_id)
			{
				continue;
			}
			//假如关节不运动，获取最长执行的时间那个关节的时间
			if (fabs(p_traj[i].total_interval) < MOTION_MODULE_CONSTANT_MIN_POSITIVE)
			{
				for (counter = 0; counter < 8; ++counter)
				{
					//save time series
					p_traj[i].time_series[counter] = p_traj[Tmax_id].time_series[counter];
				}
				continue;
			}
			//get the scale k
			k = p_traj[Tmax_id].total_interval / p_traj[i].total_interval;
			k2 = k*k;
			k3 = k2*k;
			// 有始末速度的梯形规划的缩放方法不同
			if (non_zero_boundary_flag == 1)
			{
				// 时间缩放，时间取整
				for (counter = 0; counter < 8; ++counter)
				{
					p_traj[i].time_series[counter] = floor(k*p_traj[i].time_series[counter]); //floor 与cell导致很大区别。
				}
				p_db = &(p_traj[i].time_series[0]);  //取地址，便于书写
													 // 时间取整后的补偿
				if (p_traj[i].total_interval < 1E-10)
				{
					return -1;
				}
				if (i < trans_dim)
				{
					pos = p_motion->pose.pose_end[i] - p_motion->pose.pose_start[i];
					vs = p_motion->pose.vel_start[i];
					ve = p_motion->pose.vel_end[i];
				}
				else
				{
					pos = p_motion->pose.angle_lenth;
					vs = 0;
					ve = 0;
				}
				vmax_tmp = (pos - ve * (0.5*(p_db[5] + p_db[6]) - p_db[4])*\
							p_motion->profile.profile_sample_time - vs * 0.5*(p_db[2] + p_db[1])*\
							p_motion->profile.profile_sample_time) / (0.5*(p_db[1] + p_db[2] + p_db[5] + p_db[6]) - p_db[3]) \
							/ p_motion->profile.profile_sample_time;
				if (p_db[1] < 1E-10)
				{
					j1_tmp = 0;
				}
				else
				{
					j1_tmp = (vmax_tmp - vs) / (p_db[1] * p_db[2] * p_motion->profile.profile_sample_time*p_motion->profile.profile_sample_time);
				}
				if ((p_db[5] - p_db[4])*p_motion->profile.profile_sample_time < 1E-10)
				{
					j2_tmp = 0;
				}
				else
				{
					j2_tmp = (vmax_tmp - ve) / ((p_db[6] - p_db[4]) * (p_db[5] - p_db[4])*p_motion->profile.profile_sample_time*p_motion->profile.profile_sample_time);
				}

				p_traj[i].jerk_series[0] = j1_tmp;
				p_traj[i].jerk_series[1] = 0;
				p_traj[i].jerk_series[2] = -j1_tmp;
				p_traj[i].jerk_series[3] = 0;
				p_traj[i].jerk_series[4] = -j2_tmp;
				p_traj[i].jerk_series[5] = 0;
				p_traj[i].jerk_series[6] = j2_tmp;
				p_traj[i].jerk_series[7] = 0;

				p_traj[i].acc_series[0] = 0;

				if (i < trans_dim)
				{
					p_traj[i].vel_series[0] = p_motion->pose.vel_start[i];
					p_traj[i].pos_series[0] = p_motion->pose.pose_start[i];
				}
				else
				{
					p_traj[i].pos_series[0] = 0;
					p_traj[i].vel_series[0] = 0;
				}


				for (counter = 1; counter < 8; counter++)
				{
					time_interval = p_traj[i].time_series[counter] - p_traj[i].time_series[counter - 1];
					time_interval *= p_motion->profile.profile_sample_time;
					// acc
					calculate_acceleration_from_profile_(time_interval, \
						p_traj[i].jerk_series[counter - 1], \
						p_traj[i].acc_series[counter - 1], \
						&p_traj[i].acc_series[counter]);
					// vel
					calculate_velocity_from_profile_(time_interval, \
						p_traj[i].jerk_series[counter - 1], \
						p_traj[i].acc_series[counter - 1], \
						p_traj[i].vel_series[counter - 1], \
						&p_traj[i].vel_series[counter]);
					// pos
					calculate_position_from_profile_(time_interval, \
						p_traj[i].jerk_series[counter - 1], \
						p_traj[i].acc_series[counter - 1], \
						p_traj[i].vel_series[counter - 1], \
						p_traj[i].pos_series[counter - 1], \
						&p_traj[i].pos_series[counter]);
				}

			}
			else
			{
				//according to the scale k to update other joint profiles,do not calculate again,just update
				for (counter = 0; counter < 8; ++counter)
				{
					p_traj[i].time_series[counter] = ceil(k*p_traj[i].time_series[counter]);
					p_traj[i].jerk_series[counter] /= (k3);
					p_traj[i].acc_series[counter] /= (k2);
					p_traj[i].vel_series[counter] /= (k);
					//pos do not change
				}
			}
			p_traj[i].total_interval = p_traj[Tmax_id].total_interval;
			//now counter is 8,强制最后一段时间和时间最长的那个关节一致。
			p_traj[i].time_series[counter - 1] = p_traj[Tmax_id].time_series[counter - 1];
		}
	}
	else if (sync_method == CARTLINE_TYPE_SYNC_TIME_SAME)
	{
		//calculate the every joint's parameter for fitting the max T;
		for (i = 0; i < dim; i++)
		{
			if (i == Tmax_id)
			{
				continue;
			}
			if (fabs(p_traj[i].total_interval) < MOTION_MODULE_CONSTANT_MIN_POSITIVE)
			{
				for (counter = 0; counter < 8; ++counter)
				{
					//save time series
					p_traj[i].time_series[counter] = p_traj[Tmax_id].time_series[counter];
				}
				continue;
			}
			// t1 	= t(2) - t(1);
			// t11  = t(3) - t(2) = t(1)- t(0);
			// t3 	= t(4) - t(3);
			// t2 	= t(6) - t(5);
			// t22  = t(7) - t(6) = t(5)- t(4);

			// Vmax = pos / (t11 + 1/2 * t1 + t22 + 1/2 * t2 + t3)*sampletime
			if (i < trans_dim)
			{
				pos = p_motion->pose.pose_end[i] - p_motion->pose.pose_start[i];
			}
			else
			{
				pos = p_motion->pose.angle_lenth;
			}
			vmax_tmp = pos / ((p_traj[Tmax_id].time_series[1] + \
				p_traj[Tmax_id].time_series[5] - p_traj[Tmax_id].time_series[4] + \
				0.5*(p_traj[Tmax_id].time_series[2] - p_traj[Tmax_id].time_series[1] + \
					p_traj[Tmax_id].time_series[6] - p_traj[Tmax_id].time_series[5]) + \
				p_traj[Tmax_id].time_series[4] - p_traj[Tmax_id].time_series[3])* \
				p_motion->profile.profile_sample_time);
			//-------------------------------------------------------------------------------------------
			// J1 = Vmax / ((t11^2 + t1 * t11)*Ts^2);
			// J2 = Vmax / ((t22^2 + t2 * t22)*Ts^2);
			//calculate jerk series
			p_traj[i].jerk_series[0] = vmax_tmp / (p_traj[Tmax_id].time_series[1] * \
				p_traj[Tmax_id].time_series[2] * \
				p_motion->profile.profile_sample_time* \
				p_motion->profile.profile_sample_time);

			p_traj[i].jerk_series[1] = 0.0;

			p_traj[i].jerk_series[2] = -p_traj[i].jerk_series[0];

			p_traj[i].jerk_series[3] = 0.0;

			p_traj[i].jerk_series[4] = -vmax_tmp / ((p_traj[Tmax_id].time_series[5] - \
				p_traj[Tmax_id].time_series[4])* \
				(p_traj[Tmax_id].time_series[6] - \
					p_traj[Tmax_id].time_series[4])* \
				p_motion->profile.profile_sample_time* \
				p_motion->profile.profile_sample_time);

			p_traj[i].jerk_series[5] = 0.0;

			p_traj[i].jerk_series[6] = -p_traj[i].jerk_series[4];

			p_traj[i].jerk_series[7] = 0.0;


			// initial states for acc, dec, vel, and pos
			p_traj[i].acc_series[0] = 0.0;
			p_traj[i].vel_series[0] = 0.0;
			if (i < trans_dim)
			{
				p_traj[i].pos_series[0] = p_motion->pose.pose_start[i];
			}
			else
			{
				p_traj[i].pos_series[0] = 0;
			}

			for (counter = 1; counter < 8; ++counter)
			{
				//save time series
				p_traj[i].time_series[counter - 1] = p_traj[Tmax_id].time_series[counter - 1];
				//get time interval
				time_interval = p_traj[Tmax_id].time_series[counter] - p_traj[Tmax_id].time_series[counter - 1];
				time_interval *= p_motion->profile.profile_sample_time;

				//calculate acc series
				calculate_acceleration_from_profile(time_interval,
					p_traj[i].jerk_series[counter - 1],
					p_traj[i].acc_series[counter - 1],
					&p_traj[i].acc_series[counter]);
				//calculate vel series
				calculate_velocity_from_profile(time_interval,
					p_traj[i].jerk_series[counter - 1],
					p_traj[i].acc_series[counter - 1],
					p_traj[i].vel_series[counter - 1],
					&p_traj[i].vel_series[counter]);
				//caculate pose series
				calculate_position_from_profile(time_interval,
					p_traj[i].jerk_series[counter - 1],
					p_traj[i].acc_series[counter - 1],
					p_traj[i].vel_series[counter - 1],
					p_traj[i].pos_series[counter - 1],
					&p_traj[i].pos_series[counter]);
			}
			//now counter is 8
			p_traj[i].time_series[counter - 1] = p_traj[Tmax_id].time_series[counter - 1];
			p_traj[i].total_interval = p_traj[Tmax_id].total_interval;
		}
	}
	else
	{
		//@@@@ 后续处理
		return -1;
	}

	p_motion->state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_RUNNING;
	return;
}
//----------------------------------------------------------------------------------------
void calculate_motion_parameters(robot_config_module* m_cfg, motion_block* p_motion)
{
	Uint8 cartesian_id = 0;
	Uint8 joint_id = 0;
	Uint8 Tmax_id = 0;
	Uint8 counter = 0;
	double theta_tmp;
	double Tmax = 0.0;
	double vmax_tmp;
	double k;
	double k2;
	double k3;
	//double comp;
	double time_interval = 0;

	//used in circle
	double tmp = 0.0;
	double tmp1 = 0.0;
	double tmp2 = 0.0;
	matxx P0, P0P1, P0P2, P1P2;
	matxx u, v, w; /* normal vector */
	double P1_u, P2_u, P2_v;
	double C_u, C_v;


	switch (p_motion->motion_type)
	{
	case TRAJECTORY_MODULE_MOTION_TYPE_P2P: //point to point motion in joint
	{
		// this method need to change
		// because in joint space. we can not calculate the norm like cartesian
		// we need calculate every axis profile separately,and find the longest time used,and inverse to use this time to
		// recalculate the profile again.(we can scale use the time ratio)
		// for example
		// if t--> k *t and s--> s(constant)
		// then,v-->v/k ,acc--->acc/k^2 ,jerk--->jerk/k^3

		// use the scale method ,we can not need calculate again


		// * point to point motion in joint ,give joint angle,for scara,you need give four axis angle
		// *
		// * calculate the arc length and motion direction vector: J0J1
		// *
		// * J0J1 = J1-J0
		// *

		//set profile coordinate type ,according to the type to do the blending
		p_motion->prfcoord_type = COORD_TYPE_JOINT;
		//do copy
		matxx_copy(&p_motion->pose.joint_end_vector, &p_motion->pose.joint_unit_vector);

		// y = y -x ,  y = end -start ,joint_unit_voctor=joint_end_vector-joint_start_vector
		matxx_k_mac(-1.0, &p_motion->pose.joint_start_vector, &p_motion->pose.joint_unit_vector);

		for (joint_id = 0; joint_id < m_cfg->joint_dim; joint_id++)
		{
			//get every joint arc lenth
			p_motion->pose.arc_length = (*p_motion->pose.joint_unit_vector.point)[joint_id];

			//if this joint need to move,then calculate the motion profile ,if not ,do not calculate internal
			calculate_motion_profile_with_limited_jerk(p_motion, COORD_TYPE_JOINT, joint_id);

			//find the id of max_T
			if (Tmax < p_motion->traj_joint[joint_id].total_interval)
			{
				//get the time max joint id
				Tmax = p_motion->traj_joint[joint_id].total_interval;
				Tmax_id = joint_id;
			}
		}
		//now we have know that which joint the max time ,so we use this time intervals to calculate other joint
		//we have two methods, one is scale the time ,do not let every section time is the same as the most longest time joint
		if (p_motion->jointp2p_sync == JOINTP2P_TYPE_TIME_SCALE)
		{
			//calculate the every joint's parameter for fitting the max T;
			for (joint_id = 0; joint_id < m_cfg->joint_dim; joint_id++)
			{
				if (joint_id == Tmax_id)
				{
					continue;
				}
				if (fabs(p_motion->traj_joint[joint_id].total_interval) < MOTION_MODULE_CONSTANT_MIN_POSITIVE)
				{
					for (counter = 0; counter < 8; ++counter)
					{
						//save time series
						p_motion->traj_joint[joint_id].time_series[counter] = p_motion->traj_joint[Tmax_id].time_series[counter];
					}
					continue;
				}
				//get the scale k
				k = p_motion->traj_joint[Tmax_id].total_interval / p_motion->traj_joint[joint_id].total_interval;
				k2 = k*k;
				k3 = k2*k;
				//according to the scale k to update other joint profiles,do not calculate again,just update
				for (counter = 0; counter < 8; ++counter)
				{
					p_motion->traj_joint[joint_id].time_series[counter] = ceil(k*p_motion->traj_joint[joint_id].time_series[counter]);
					p_motion->traj_joint[joint_id].jerk_series[counter] /= (k3);
					p_motion->traj_joint[joint_id].acc_series[counter] /= (k2);
					p_motion->traj_joint[joint_id].vel_series[counter] /= (k);
					//pos do not change
				}
				p_motion->traj_joint[joint_id].total_interval = p_motion->traj_joint[Tmax_id].total_interval;
				p_motion->traj_joint[joint_id].time_series[counter - 1] = p_motion->traj_joint[Tmax_id].time_series[counter - 1]; //now counter is 8

			}
		}
		else if (p_motion->jointp2p_sync == JOINTP2P_TYPE_TIME_SAME)
		{
			//calculate the every joint's parameter for fitting the max T;
			for (joint_id = 0; joint_id < m_cfg->joint_dim; joint_id++)
			{
				if (joint_id == Tmax_id)
				{
					continue;
				}
				if (fabs(p_motion->traj_joint[joint_id].total_interval) < MOTION_MODULE_CONSTANT_MIN_POSITIVE)
				{
					for (counter = 0; counter < 8; ++counter)
					{
						//save time series
						p_motion->traj_joint[joint_id].time_series[counter] = p_motion->traj_joint[Tmax_id].time_series[counter];
					}
					continue;
				}
				// t1 	= t(2) - t(1);
				// t11  = t(3) - t(2) = t(1)- t(0);
				// t3 	= t(4) - t(3);
				// t2 	= t(6) - t(5);
				// t22  = t(7) - t(6) = t(5)- t(4);

				// Vmax = pos / (t11 + 1/2 * t1 + t22 + 1/2 * t2 + t3)*sampletime

				vmax_tmp = (*p_motion->pose.joint_unit_vector.point)[joint_id] / ((p_motion->traj_joint[Tmax_id].time_series[1] + \
					p_motion->traj_joint[Tmax_id].time_series[5] - p_motion->traj_joint[Tmax_id].time_series[4] + \
					0.5*(p_motion->traj_joint[Tmax_id].time_series[2] - p_motion->traj_joint[Tmax_id].time_series[1] + \
						p_motion->traj_joint[Tmax_id].time_series[6] - p_motion->traj_joint[Tmax_id].time_series[5]) + \
					p_motion->traj_joint[Tmax_id].time_series[4] - p_motion->traj_joint[Tmax_id].time_series[3])* \
					p_motion->prf_joint[joint_id].profile_sample_time);
				//-------------------------------------------------------------------------------------------
				// J1 = Vmax / ((t11^2 + t1 * t11)*Ts^2);
				// J2 = Vmax / ((t22^2 + t2 * t22)*Ts^2);
				//calculate jerk series
				p_motion->traj_joint[joint_id].jerk_series[0] = vmax_tmp / (p_motion->traj_joint[Tmax_id].time_series[1] * \
					p_motion->traj_joint[Tmax_id].time_series[2] * \
					p_motion->prf_joint[joint_id].profile_sample_time* \
					p_motion->prf_joint[joint_id].profile_sample_time);

				p_motion->traj_joint[joint_id].jerk_series[1] = 0.0;

				p_motion->traj_joint[joint_id].jerk_series[2] = -p_motion->traj_joint[joint_id].jerk_series[0];

				p_motion->traj_joint[joint_id].jerk_series[3] = 0.0;

				p_motion->traj_joint[joint_id].jerk_series[4] = -vmax_tmp / ((p_motion->traj_joint[Tmax_id].time_series[5] - \
					p_motion->traj_joint[Tmax_id].time_series[4])* \
					(p_motion->traj_joint[Tmax_id].time_series[6] - \
						p_motion->traj_joint[Tmax_id].time_series[4])* \
					p_motion->prf_joint[joint_id].profile_sample_time* \
					p_motion->prf_joint[joint_id].profile_sample_time);

				p_motion->traj_joint[joint_id].jerk_series[5] = 0.0;

				p_motion->traj_joint[joint_id].jerk_series[6] = -p_motion->traj_joint[joint_id].jerk_series[4];

				p_motion->traj_joint[joint_id].jerk_series[7] = 0.0;


				// initial states for acc, dec, vel, and pos
				p_motion->traj_joint[joint_id].acc_series[0] = 0.0;
				p_motion->traj_joint[joint_id].vel_series[0] = 0.0;
				p_motion->traj_joint[joint_id].pos_series[0] = 0.0;

				for (counter = 1; counter < 8; ++counter)
				{
					//save time series
					p_motion->traj_joint[joint_id].time_series[counter - 1] = p_motion->traj_joint[Tmax_id].time_series[counter - 1];
					//get time interval
					time_interval = p_motion->traj_joint[Tmax_id].time_series[counter] - p_motion->traj_joint[Tmax_id].time_series[counter - 1];
					time_interval *= p_motion->prf_joint[joint_id].profile_sample_time;

					//calculate acc series
					calculate_acceleration_from_profile(time_interval,
						p_motion->traj_joint[joint_id].jerk_series[counter - 1],
						p_motion->traj_joint[joint_id].acc_series[counter - 1],
						&p_motion->traj_joint[joint_id].acc_series[counter]);
					//calculate vel series
					calculate_velocity_from_profile(time_interval,
						p_motion->traj_joint[joint_id].jerk_series[counter - 1],
						p_motion->traj_joint[joint_id].acc_series[counter - 1],
						p_motion->traj_joint[joint_id].vel_series[counter - 1],
						&p_motion->traj_joint[joint_id].vel_series[counter]);
					//caculate pose series
					calculate_position_from_profile(time_interval,
						p_motion->traj_joint[joint_id].jerk_series[counter - 1],
						p_motion->traj_joint[joint_id].acc_series[counter - 1],
						p_motion->traj_joint[joint_id].vel_series[counter - 1],
						p_motion->traj_joint[joint_id].pos_series[counter - 1],
						&p_motion->traj_joint[joint_id].pos_series[counter]);
				}
				//now counter is 8
				p_motion->traj_joint[joint_id].time_series[counter - 1] = p_motion->traj_joint[Tmax_id].time_series[counter - 1];
				p_motion->traj_joint[joint_id].total_interval = p_motion->traj_joint[Tmax_id].total_interval;
			}
		}
		else
		{

		}


		break;
	}
	//-----------------------------------------------------------------------------------------------------------------------
	case TRAJECTORY_MODULE_MOTION_TYPE_LIN: // line motion in Cartesian
	{
		//matxx rot_start,rot_end;
		//matxx rot_tmp;

		//set profile coordinate type ,according to the type to do the blending
		p_motion->prfcoord_type = (COORD_TYPE_CART);
		//------------------------------------------------------------------
		// 1. compute the distance
		p_motion->pose.arc_length = 0.0;

		for (cartesian_id = 0; cartesian_id < TRAJECTORY_MODULE_DIMENSION_OF_XYZ_FRAME; ++cartesian_id)
		{
			//end - start
			p_motion->pose.pose_unit_vector[cartesian_id] = p_motion->pose.pose_end[cartesian_id] - p_motion->pose.pose_start[cartesian_id];
			p_motion->pose.arc_length += p_motion->pose.pose_unit_vector[cartesian_id] * p_motion->pose.pose_unit_vector[cartesian_id];
		}
		//calculate the norm
		p_motion->pose.arc_length = sqrt(p_motion->pose.arc_length);

		//record this lenth for trans
		p_motion->pose.arc_lenth_trans = p_motion->pose.arc_length;
		//------------------------------------------------------------------
		// 2. pos do motion planning in the Cartesian coordinate
		calculate_motion_profile_with_limited_jerk(p_motion, COORD_TYPE_CART_TRANS, 0);

		//------------------------------------------------------------------
		// 3. normalization vector
		for (cartesian_id = 0; cartesian_id < TRAJECTORY_MODULE_DIMENSION_OF_XYZ_FRAME; cartesian_id++)
		{
			// generate the unit vector of the motion profile
			if (fabs(p_motion->pose.arc_length) < MOTION_MODULE_CONSTANT_MIN_POSITIVE)
			{
				// distance = 0
				p_motion->pose.pose_unit_vector[cartesian_id] = 0;
			}
			else
			{
				// get unit calculation
				p_motion->pose.pose_unit_vector[cartesian_id] = p_motion->pose.pose_unit_vector[cartesian_id] / p_motion->pose.arc_length;
			}
		}
		//----------------------------------------------------------------------------------
		//posture do motion planning in the Cartesian coordinate
		if (p_motion->cartline_method == CARTLINE_TYPE_ROT_MATRIX)
		{
			//
		}
		else if (p_motion->cartline_method == CARTLINE_TYPE_AXIS_ANGLE)
		{
			//-------------------------------------------------------------------
			// 4. pose,input is fix angle,so change them to rotate matrix ,and then change the rotate matrix to axis angle
			//create start pose and end pose rotate matrix
			//SQUARE3(&rot_start);
			//SQUARE3(&rot_end);
			//SQUARE3(&rot_tmp);

			//change_10_29
			/*
			//calculate the angle sin and cos used for rotate matrix of start pose
			sr = sin(p_motion->pose.pose_start[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);  //rotate x
			sb = sin(p_motion->pose.pose_start[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME+1] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD); //rotate y
			sa = sin(p_motion->pose.pose_start[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME+2] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD); //rotate z
			cr = cos(p_motion->pose.pose_start[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			cb = cos(p_motion->pose.pose_start[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME+1] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			ca = cos(p_motion->pose.pose_start[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME+2] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);

			//calculate the rotate matrix of start pose
			(*(p_motion->pose.rot_start.point + 0))[0] 	= ca*cb;
			(*(p_motion->pose.rot_start.point + 0))[1]  = sa*cb;
			(*(p_motion->pose.rot_start.point + 0))[2]  = -sb;

			(*(p_motion->pose.rot_start.point + 1))[0] 	= ca*sb*sr - sa*cr;
			(*(p_motion->pose.rot_start.point + 1))[1]  = sa*sb*sr + ca*cr;
			(*(p_motion->pose.rot_start.point + 1))[2]  = cb*sr;

			(*(p_motion->pose.rot_start.point + 2))[0] 	= ca*sb*cr + sa*sr;
			(*(p_motion->pose.rot_start.point + 2))[1]  = sa*sb*cr - ca*sr;
			(*(p_motion->pose.rot_start.point + 2))[2]  = cb*cr;
			*/

			xyz2r(&p_motion->pose.pose_start[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME], &p_motion->pose.rot_start, 0);

			/*
			//calculate the angle sin and cos used for rotate matrix of end pose
			sr = sin(p_motion->pose.pose_end[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			sb = sin(p_motion->pose.pose_end[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME+1] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			sa = sin(p_motion->pose.pose_end[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME+2] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			cr = cos(p_motion->pose.pose_end[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			cb = cos(p_motion->pose.pose_end[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME+1] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			ca = cos(p_motion->pose.pose_end[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME+2] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);

			//calculate the rotate matrix of end pose
			(*(p_motion->pose.rot_end.point + 0))[0] 	= ca*cb;
			(*(p_motion->pose.rot_end.point + 0))[1]  	= sa*cb;
			(*(p_motion->pose.rot_end.point + 0))[2]  	= -sb;

			(*(p_motion->pose.rot_end.point + 1))[0] 	= ca*sb*sr - sa*cr;
			(*(p_motion->pose.rot_end.point + 1))[1]  	= sa*sb*sr + ca*cr;
			(*(p_motion->pose.rot_end.point + 1))[2]  	= cb*sr;

			(*(p_motion->pose.rot_end.point + 2))[0] 	= ca*sb*cr + sa*sr;
			(*(p_motion->pose.rot_end.point + 2))[1]  	= sa*sb*cr - ca*sr;
			(*(p_motion->pose.rot_end.point + 2))[2]  	= cb*cr;
			*/

			xyz2r(&p_motion->pose.pose_end[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME], &p_motion->pose.rot_end, 0);

			//get the rotate matrix from  pose start to pose end  PB = R*PA --->R = PB*PA^-1
			//PA^-1 = PA^T
			matxx_transpose(&p_motion->pose.rot_start, &p_motion->pose.rot_tmp);
			matxx_multiply(&p_motion->pose.rot_end, &p_motion->pose.rot_tmp, &p_motion->pose.rotation_matxx);
			//matxx_printf(&p_motion->pose.rot_tmp);
			/*
			//calculate the axis angle
			p_motion->pose.angle_lenth = acos(0.5*((*(p_motion->pose.rotation_matxx.point + 0))[0] + \
			(*(p_motion->pose.rotation_matxx.point + 1))[1] + \
			(*(p_motion->pose.rotation_matxx.point + 2))[2] - 1));
			p_motion->pose.arc_length =  p_motion->pose.angle_lenth;

			//check if the angle is too small.which can not calculate the axis
			if (sin(p_motion->pose.angle_lenth)< MOTION_MODULE_CONSTANT_MIN_POSITIVE)
			{
			//printf("the line move pose angle is too small\n");
			//can not calculate axis
			//return;
			p_motion->pose.axis_angle_flag = false;
			}
			else
			{
			p_motion->pose.axis_angle_flag = true;

			//common part 1/2/sin(angle)
			theta_tmp = 1.0/2/sin(p_motion->pose.angle_lenth);

			//r32-r23
			(*(p_motion->pose.axis_vector.point + 0))[0] = theta_tmp*((*(p_motion->pose.rotation_matxx.point + 1))[2]- \
			(*(p_motion->pose.rotation_matxx.point + 2))[1]);
			//r13-r31
			(*(p_motion->pose.axis_vector.point + 0))[1] = theta_tmp*((*(p_motion->pose.rotation_matxx.point + 2))[0]- \
			(*(p_motion->pose.rotation_matxx.point + 0))[2]);
			//r21 - r12
			(*(p_motion->pose.axis_vector.point + 0))[2] = theta_tmp*((*(p_motion->pose.rotation_matxx.point + 0))[1]- \
			(*(p_motion->pose.rotation_matxx.point + 1))[0]);
			}
			*/
		//	matxx_printf(&p_motion->pose.rotation_matxx);
			if (r2AxisAngle(&p_motion->pose.rotation_matxx, &p_motion->pose.axis_vector, &p_motion->pose.angle_lenth) == 0)
			{
				p_motion->pose.axis_angle_flag = 1;
			}
			else
			{
				p_motion->pose.axis_angle_flag = 0;
			}
			p_motion->pose.arc_length = p_motion->pose.angle_lenth;

			//calculate the angle motion profile
			calculate_motion_profile_with_limited_jerk(p_motion, COORD_TYPE_CART_ROT, 0);

			if (p_motion->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SAME)
			{
				//check which part time is longer, if translate time longer than rotate time,then let rotate move as translate.
				if (p_motion->trajectory.total_interval >= p_motion->traj_cart_rot.total_interval)
				{
					if (fabs(p_motion->traj_cart_rot.total_interval) > MOTION_MODULE_CONSTANT_MIN_POSITIVE)
					{
						// t1 	= t(2) - t(1);
						// t11  = t(3) - t(2) = t(1)- t(0);
						// t3 	= t(4) - t(3);
						// t2 	= t(6) - t(5);
						// t22  = t(7) - t(6) = t(5)- t(4);

						// Vmax = pos / (t11 + 1/2 * t1 + t22 + 1/2 * t2 + t3)*sampletime

						vmax_tmp = (p_motion->pose.angle_lenth / ((p_motion->trajectory.time_series[1] + \
							p_motion->trajectory.time_series[5] - p_motion->trajectory.time_series[4] + \
							0.5*(p_motion->trajectory.time_series[2] - p_motion->trajectory.time_series[1] + \
								p_motion->trajectory.time_series[6] - p_motion->trajectory.time_series[5]) + \
							p_motion->trajectory.time_series[4] - p_motion->trajectory.time_series[3])* \
							p_motion->profile.profile_sample_time));
						//-------------------------------------------------------------------------------------------
						// J1 = Vmax / ((t11^2 + t1 * t11)*Ts^2);
						// J2 = Vmax / ((t22^2 + t2 * t22)*Ts^2);
						//calculate jerk series
						p_motion->traj_cart_rot.jerk_series[0] = vmax_tmp / (p_motion->trajectory.time_series[1] * \
							p_motion->trajectory.time_series[2] * \
							p_motion->profile.profile_sample_time* \
							p_motion->profile.profile_sample_time);

						p_motion->traj_cart_rot.jerk_series[1] = 0.0;

						p_motion->traj_cart_rot.jerk_series[2] = -p_motion->traj_cart_rot.jerk_series[0];

						p_motion->traj_cart_rot.jerk_series[3] = 0.0;

						p_motion->traj_cart_rot.jerk_series[4] = -vmax_tmp / ((p_motion->trajectory.time_series[5] - \
							p_motion->trajectory.time_series[4])* \
							(p_motion->trajectory.time_series[6] - \
								p_motion->trajectory.time_series[4])* \
							p_motion->profile.profile_sample_time* \
							p_motion->profile.profile_sample_time);

						p_motion->traj_cart_rot.jerk_series[5] = 0.0;

						p_motion->traj_cart_rot.jerk_series[6] = -p_motion->traj_cart_rot.jerk_series[4];

						p_motion->traj_cart_rot.jerk_series[7] = 0.0;


						// initial states for acc, dec, vel, and pos
						p_motion->traj_cart_rot.acc_series[0] = 0.0;
						p_motion->traj_cart_rot.vel_series[0] = 0.0;
						p_motion->traj_cart_rot.pos_series[0] = 0.0;

						for (counter = 1; counter < 8; ++counter)
						{
							//save time series
							p_motion->traj_cart_rot.time_series[counter - 1] = p_motion->trajectory.time_series[counter - 1];
							//get time interval
							time_interval = p_motion->trajectory.time_series[counter] - p_motion->trajectory.time_series[counter - 1];
							time_interval *= p_motion->profile.profile_sample_time;

							//calculate acc series
							calculate_acceleration_from_profile(time_interval,
								p_motion->traj_cart_rot.jerk_series[counter - 1],
								p_motion->traj_cart_rot.acc_series[counter - 1],
								&p_motion->traj_cart_rot.acc_series[counter]);
							//calculate vel series
							calculate_velocity_from_profile(time_interval,
								p_motion->traj_cart_rot.jerk_series[counter - 1],
								p_motion->traj_cart_rot.acc_series[counter - 1],
								p_motion->traj_cart_rot.vel_series[counter - 1],
								&p_motion->traj_cart_rot.vel_series[counter]);
							//caculate pose series
							calculate_position_from_profile(time_interval,
								p_motion->traj_cart_rot.jerk_series[counter - 1],
								p_motion->traj_cart_rot.acc_series[counter - 1],
								p_motion->traj_cart_rot.vel_series[counter - 1],
								p_motion->traj_cart_rot.pos_series[counter - 1],
								&p_motion->traj_cart_rot.pos_series[counter]);
						}
						//now counter is 8
						p_motion->traj_cart_rot.time_series[counter - 1] = p_motion->trajectory.time_series[counter - 1];
						p_motion->traj_cart_rot.total_interval = p_motion->trajectory.total_interval;
					}
					else
					{
						for (counter = 0; counter < 8; ++counter)
						{
							p_motion->traj_cart_rot.time_series[counter] = p_motion->trajectory.time_series[counter];
						}
						p_motion->traj_cart_rot.total_interval = p_motion->trajectory.total_interval;
					}

				}
				else
				{
					if (p_motion->trajectory.total_interval > MOTION_MODULE_CONSTANT_MIN_POSITIVE)
					{
						// t1 	= t(2) - t(1);
						// t11  = t(3) - t(2) = t(1)- t(0);
						// t3 	= t(4) - t(3);
						// t2 	= t(6) - t(5);
						// t22  = t(7) - t(6) = t(5)- t(4);

						// Vmax = pos / (t11 + 1/2 * t1 + t22 + 1/2 * t2 + t3)*sampletime

						vmax_tmp = (p_motion->pose.arc_lenth_trans / ((p_motion->traj_cart_rot.time_series[1] + \
							p_motion->traj_cart_rot.time_series[5] - p_motion->traj_cart_rot.time_series[4] + \
							0.5*(p_motion->traj_cart_rot.time_series[2] - p_motion->traj_cart_rot.time_series[1] + \
								p_motion->traj_cart_rot.time_series[6] - p_motion->traj_cart_rot.time_series[5]) + \
							p_motion->traj_cart_rot.time_series[4] - p_motion->traj_cart_rot.time_series[3])* \
							p_motion->profile.profile_sample_time));
						//-------------------------------------------------------------------------------------------
						// J1 = Vmax / ((t11^2 + t1 * t11)*Ts^2);
						// J2 = Vmax / ((t22^2 + t2 * t22)*Ts^2);
						//calculate jerk series
						p_motion->trajectory.jerk_series[0] = vmax_tmp / (p_motion->traj_cart_rot.time_series[1] * \
							p_motion->traj_cart_rot.time_series[2] * \
							p_motion->profile.profile_sample_time* \
							p_motion->profile.profile_sample_time);

						p_motion->trajectory.jerk_series[1] = 0.0;

						p_motion->trajectory.jerk_series[2] = -p_motion->trajectory.jerk_series[0];

						p_motion->trajectory.jerk_series[3] = 0.0;

						p_motion->trajectory.jerk_series[4] = -vmax_tmp / ((p_motion->traj_cart_rot.time_series[5] - \
							p_motion->traj_cart_rot.time_series[4])* \
							(p_motion->traj_cart_rot.time_series[6] - \
								p_motion->traj_cart_rot.time_series[4])* \
							p_motion->profile.profile_sample_time* \
							p_motion->profile.profile_sample_time);

						p_motion->trajectory.jerk_series[5] = 0.0;

						p_motion->trajectory.jerk_series[6] = -p_motion->trajectory.jerk_series[4];

						p_motion->trajectory.jerk_series[7] = 0.0;


						// initial states for acc, dec, vel, and pos
						p_motion->trajectory.acc_series[0] = 0.0;
						p_motion->trajectory.vel_series[0] = 0.0;
						p_motion->trajectory.pos_series[0] = 0.0;

						for (counter = 1; counter < 8; ++counter)
						{
							//save time series
							p_motion->trajectory.time_series[counter - 1] = p_motion->traj_cart_rot.time_series[counter - 1];
							//get time interval
							time_interval = p_motion->traj_cart_rot.time_series[counter] - p_motion->traj_cart_rot.time_series[counter - 1];
							time_interval *= p_motion->profile.profile_sample_time;

							//calculate acc series
							calculate_acceleration_from_profile(time_interval,
								p_motion->trajectory.jerk_series[counter - 1],
								p_motion->trajectory.acc_series[counter - 1],
								&p_motion->trajectory.acc_series[counter]);
							//calculate vel series
							calculate_velocity_from_profile(time_interval,
								p_motion->trajectory.jerk_series[counter - 1],
								p_motion->trajectory.acc_series[counter - 1],
								p_motion->trajectory.vel_series[counter - 1],
								&p_motion->trajectory.vel_series[counter]);
							//caculate pose series
							calculate_position_from_profile(time_interval,
								p_motion->trajectory.jerk_series[counter - 1],
								p_motion->trajectory.acc_series[counter - 1],
								p_motion->trajectory.vel_series[counter - 1],
								p_motion->trajectory.pos_series[counter - 1],
								&p_motion->trajectory.pos_series[counter]);
						}
						//now counter is 8
						p_motion->trajectory.time_series[counter - 1] = p_motion->traj_cart_rot.time_series[counter - 1];
						p_motion->trajectory.total_interval = p_motion->traj_cart_rot.total_interval;
					}
					else
					{
						// if total_interval=0,let  time_series = time_series of the longer
						// and the jerk ,acc ,vel serise=0
						for (counter = 0; counter < 8; ++counter)
						{
							p_motion->trajectory.time_series[counter] = p_motion->traj_cart_rot.time_series[counter];
						}
						p_motion->trajectory.total_interval = p_motion->traj_cart_rot.total_interval;

					}
				}

			}
			//time scale method
			else
			{
				//check which part time is longer, if translate time longer than rotate time,then let rotate move as translate.
				if (p_motion->trajectory.total_interval >= p_motion->traj_cart_rot.total_interval)
				{
					//now we just use scaled method for cartesian line
					if (fabs(p_motion->traj_cart_rot.total_interval) > MOTION_MODULE_CONSTANT_MIN_POSITIVE)
					{

						//get the scale k
						k = p_motion->trajectory.total_interval / p_motion->traj_cart_rot.total_interval;
						k2 = k*k;
						k3 = k2*k;

						//according to the scale k to update other joint profiles,do not calculate again,just update
						for (counter = 0; counter < 8; ++counter)
						{
							/*		if(p_motion->traj_cart_rot.time_series[counter]< MOTION_MODULE_CONSTANT_MIN_POSITIVE)
							{
							comp = 1;
							}
							else
							{
							comp = (k*p_motion->traj_cart_rot.time_series[counter])/ceil(k*p_motion->traj_cart_rot.time_series[counter]);
							}*/

							//p_motion->traj_cart_rot.time_series[counter] = floor(k*p_motion->traj_cart_rot.time_series[counter]);
							p_motion->traj_cart_rot.time_series[counter] = ceil(k*p_motion->traj_cart_rot.time_series[counter]);
							p_motion->traj_cart_rot.jerk_series[counter] /= (k3);
							p_motion->traj_cart_rot.acc_series[counter] /= (k2);
							p_motion->traj_cart_rot.vel_series[counter] /= (k);

							/*p_motion->traj_cart_rot.jerk_series[counter] *=(comp*comp*comp);
							p_motion->traj_cart_rot.acc_series[counter]  *=(comp*comp);
							p_motion->traj_cart_rot.vel_series[counter]  *=(comp);*/
							//pos do not change
						}
						p_motion->traj_cart_rot.total_interval = p_motion->trajectory.total_interval;
						p_motion->traj_cart_rot.time_series[counter - 1] = p_motion->trajectory.time_series[counter - 1];
					}
					else
					{
						for (counter = 0; counter < 8; ++counter)
						{
							p_motion->traj_cart_rot.time_series[counter] = p_motion->trajectory.time_series[counter];
						}
						p_motion->traj_cart_rot.total_interval = p_motion->trajectory.total_interval;
					}
				}
				else
				{
					if (p_motion->trajectory.total_interval > MOTION_MODULE_CONSTANT_MIN_POSITIVE)
					{
						k = p_motion->traj_cart_rot.total_interval / p_motion->trajectory.total_interval;
						k2 = k*k;
						k3 = k2*k;
						for (counter = 0; counter < 8; ++counter)
						{
							/*if(p_motion->trajectory.time_series[counter]< MOTION_MODULE_CONSTANT_MIN_POSITIVE)
							{
							comp = 1;
							}
							else
							{
							comp = (k*p_motion->trajectory.time_series[counter])/ceil(k*p_motion->trajectory.time_series[counter]);
							}*/
							//p_motion->trajectory.time_series[counter] = floor(k*p_motion->trajectory.time_series[counter]);
							p_motion->trajectory.time_series[counter] = ceil(k*p_motion->trajectory.time_series[counter]);
							p_motion->trajectory.jerk_series[counter] /= (k3);
							p_motion->trajectory.acc_series[counter] /= (k2);
							p_motion->trajectory.vel_series[counter] /= (k);

							/*p_motion->trajectory.jerk_series[counter] *=(comp*comp*comp);
							p_motion->trajectory.acc_series[counter]  *=(comp*comp);
							p_motion->trajectory.vel_series[counter]  *=(comp);*/
						}
						p_motion->trajectory.total_interval = p_motion->traj_cart_rot.total_interval;
						p_motion->trajectory.time_series[7] = p_motion->traj_cart_rot.time_series[7];
					}
					else
					{
						// if total_interval=0,let  time_series = time_series of the longer
						// and the jerk ,acc ,vel serise=0
						for (counter = 0; counter < 8; ++counter)
						{
							p_motion->trajectory.time_series[counter] = p_motion->traj_cart_rot.time_series[counter];
						}
						p_motion->trajectory.total_interval = p_motion->traj_cart_rot.total_interval;

					}
				}
			}

			//------------------------------------
			//matxx_delete(&rot_start);
			//matxx_delete(&rot_end);
			//matxx_delete(&rot_tmp);
		}
		break;
	}
	//-----------------------------------------------------------------------------------------------------------------------
	case TRAJECTORY_MODULE_MOTION_TYPE_CIRCL:
	{
		//matxx rot_start,rot_end;
		//matxx rot_tmp;


		//double temp = 0.0;

		//set profile coordinate type ,according to the type to do the blending
		p_motion->prfcoord_type = (COORD_TYPE_CART);
		//------------------------------------------------------------------------
		// create vectors (u,v,w) for the frame of CIRCL motion plane

		//create three vector which in cartesian frame
		VECTOR3I(&P0, p_motion->pose.pose_start);
		VECTOR3I(&P0P1, p_motion->pose.pose_aux1);
		VECTOR3I(&P0P2, p_motion->pose.pose_aux2);
		VECTOR3I(&P1P2, p_motion->pose.pose_aux2);

		//------------------------------------------------------------------------
		//P0P1 and P0P2 can produce a plane  ,the two vector in cartesian frame
		matxx_k_mac(-1.0, &P0, &P0P1);		//Vector minus   (P0P1- P0)
		matxx_k_mac(-1.0, &P0, &P0P2);		//Vector minus   (P0P2- P0)
		matxx_k_mac(-1.0, &P0P1, &P1P2);		//Vector minus   (P1P2- P0P1)

												//create u v w and let U = P0P1  ,V = P0P2
		VECTOR3(&u);
		matxx_copy(&P0P1, &u);

		VECTOR3(&v);
		matxx_copy(&P0P2, &v);

		VECTOR3(&w);

		// normalization u ,
		P1_u = matxx_EuclideanNorm2(&u);
		matxx_k_mult(1.0 / P1_u, &u);

		// u and v cross to get w and normalization w
		matxx_cross(&u, &v, &w);
		tmp = matxx_EuclideanNorm2(&w);
		matxx_k_mult(1.0 / tmp, &w);

		//w X u = v so we can get the frame because w and u are normalized,so v is normalized
		matxx_cross(&w, &u, &v);
		//------------------------------------------------------------------------
		//now we have the frame P0P1 is u ,conform right hand  rules
		/* P0 is the original of the frame (u,v,w)
		* p1 in the frame: (P1_u, 0,   0)
		* p2 in the frame: (P2_u, P2_v,0)
		* Center of CIRCL motion in the frame: (C_u, C_v,0)
		* C_u = P1_u/2  because the distance of center of circle to P0 and P1 is same ,so u axis coordinate is P1_u/2, p0 is original is (0,0,0)
		* C_v = ((P2_u - C_u)^2 + P2_v^2 - C_u^2)/2/P2_v
		* */
		P2_u = matxx_dot(&P0P2, &u);
		P2_v = matxx_dot(&P0P2, &v);

		//now we get the original in uvw frame
		C_u = P1_u * 0.5;
		C_v = ((P2_u - P1_u) * P2_u / P2_v + P2_v)* 0.5;
		//------------------------------------------------------------------------

		/*
		* from Circle original to P0 = -C_u * u - C_v * v
		* rotation direction = w
		* */
		matxx_copy(&w, &(p_motion->pose.unit_vector));  				//w is unit vector
		matxx_copy(&u, &(p_motion->pose.center_vector));
		matxx_k_mult(-C_u, &(p_motion->pose.center_vector));
		matxx_k_mac(-C_v, &v, &(p_motion->pose.center_vector));		// now center_vector is translation

																	//------------------------------------------------------------------------
																	// get the radius
		tmp = C_u * C_u + C_v * C_v;
		tmp = sqrt(tmp);
		p_motion->pose.circle_radius = tmp;
		p_motion->pose.circle_radius_inv = 1.0 / tmp;

		//------------------------------------------------------------------------
		/* get the arc length
		* s = 2*asin(|P0P2|/2/R)*R   arc = theta * R
		* */
		tmp = 0.5*matxx_EuclideanNorm2(&P0P1);
		tmp *= p_motion->pose.circle_radius_inv;
		tmp = 2.0*asin(tmp);
		tmp1 = 0.5*matxx_EuclideanNorm2(&P1P2);
		tmp1 *= p_motion->pose.circle_radius_inv;
		tmp1 = 2.0*asin(tmp1);
		tmp2 = 0.5*matxx_EuclideanNorm2(&P0P2);
		tmp2 *= p_motion->pose.circle_radius_inv;
		tmp2 = 2.0*asin(tmp2);

		if (fabs(tmp + tmp1 - tmp2) < MOTION_MODULE_CONSTANT_MIN_POSITIVE)
		{
			p_motion->pose.arc_length = tmp2*p_motion->pose.circle_radius;
		}
		else
		{
			p_motion->pose.arc_length = (2 * MOTION_MODULE_CONSTANT_PI - tmp2)*p_motion->pose.circle_radius;
		}

		// if need a toatl circle
		if (p_motion->arc_flag == 1)
		{
			p_motion->pose.arc_length = 2 * MOTION_MODULE_CONSTANT_PI;
		}


		// do motion planning in the Cartesian coordinate
		calculate_motion_profile_with_limited_jerk(p_motion, COORD_TYPE_CART_TRANS, 0);

		/* free memory */
		matxx_delete(&u);
		matxx_delete(&v);
		matxx_delete(&w);
		matxx_delete(&P0);
		matxx_delete(&P0P1);
		matxx_delete(&P0P2);
		matxx_delete(&P1P2);


		if (p_motion->cartcircle_method == CARTCIRCLE_TYPE_ROT_MATRIX)
		{
			//
		}
		else if (p_motion->cartcircle_method == CARTCIRCLE_TYPE_AXIS_ANGLE)
		{
			//-------------------------------------------------------------------
			//this part is same as line.
			// 4. pose,input is fix angle,so change them to rotate matrix ,and then change the rotate matrix to axis angle
			//create start pose and end pose rotate matrix
			//SQUARE3(&rot_start);
			//SQUARE3(&rot_end);
			//SQUARE3(&rot_tmp);

			// change_10_29
			/*
			//calculate the angle sin and cos used for rotate matrix of start pose
			sa = sin(p_motion->pose.pose_start[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME]*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			sb = sin(p_motion->pose.pose_start[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME+1]*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			sr = sin(p_motion->pose.pose_start[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME+2]*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			ca = cos(p_motion->pose.pose_start[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME]*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			cb = cos(p_motion->pose.pose_start[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME+1]*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			cr = cos(p_motion->pose.pose_start[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME+2]*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);

			//calculate the rotate matrix of start pose
			(*(p_motion->pose.rot_start.point + 0))[0] = ca*cb;
			(*(p_motion->pose.rot_start.point + 0))[1] = sa*cb;
			(*(p_motion->pose.rot_start.point + 0))[2] = -sb;

			(*(p_motion->pose.rot_start.point + 1))[0] = ca*sb*sr - sa*cr;
			(*(p_motion->pose.rot_start.point + 1))[1] = sa*sb*sr + ca*cr;
			(*(p_motion->pose.rot_start.point + 1))[2] = cb*sr;

			(*(p_motion->pose.rot_start.point + 2))[0] = ca*sb*cr + sa*sr;
			(*(p_motion->pose.rot_start.point + 2))[1] = sa*sb*cr - ca*sr;
			(*(p_motion->pose.rot_start.point + 2))[2] = cb*cr;
			*/

			xyz2r(&p_motion->pose.pose_start[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME], &p_motion->pose.rot_start, 0);

			/*
			//calculate the angle sin and cos used for rotate matrix of end pose
			sa = sin(p_motion->pose.pose_end[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME]*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			sb = sin(p_motion->pose.pose_end[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME+1]*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			sr = sin(p_motion->pose.pose_end[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME+2]*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			ca = cos(p_motion->pose.pose_end[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME]*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			cb = cos(p_motion->pose.pose_end[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME+1]*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);
			cr = cos(p_motion->pose.pose_end[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME+2]*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD);

			//calculate the rotate matrix of end pose
			(*(p_motion->pose.rot_end.point + 0))[0] = ca*cb;
			(*(p_motion->pose.rot_end.point + 0))[1] = sa*cb;
			(*(p_motion->pose.rot_end.point + 0))[2] = -sb;

			(*(p_motion->pose.rot_end.point + 1))[0] = ca*sb*sr - sa*cr;
			(*(p_motion->pose.rot_end.point + 1))[1] = sa*sb*sr + ca*cr;
			(*(p_motion->pose.rot_end.point + 1))[2] = cb*sr;

			(*(p_motion->pose.rot_end.point + 2))[0] = ca*sb*cr + sa*sr;
			(*(p_motion->pose.rot_end.point + 2))[1] = sa*sb*cr - ca*sr;
			(*(p_motion->pose.rot_end.point + 2))[2] = cb*cr;
			*/

			xyz2r(&p_motion->pose.pose_end[TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME], &p_motion->pose.rot_end, 0);


			//get the rotate matrix from  pose start to pose end  PB = R*PA --->R = PB*PA^-1
			//PA^-1 = PA^T
			matxx_transpose(&p_motion->pose.rot_start, &p_motion->pose.rot_tmp);
			matxx_multiply(&p_motion->pose.rot_end, &p_motion->pose.rot_tmp, &p_motion->pose.rotation_matxx);

			/*
			//calculate the axis angle
			p_motion->pose.angle_lenth = acos(0.5*((*(p_motion->pose.rotation_matxx.point + 0))[0] + \
			(*(p_motion->pose.rotation_matxx.point + 1))[1] + \
			(*(p_motion->pose.rotation_matxx.point + 2))[2] - 1));
			p_motion->pose.arc_length =  p_motion->pose.angle_lenth;


			if (sin(p_motion->pose.angle_lenth)< MOTION_MODULE_CONSTANT_MIN_POSITIVE)
			{
			//printf("the line move pose angle is too small\n");
			//can not calculate axis
			//return;
			p_motion->pose.axis_angle_flag = false;
			}
			else
			{
			p_motion->pose.axis_angle_flag = true;

			//comon part 1/2.0/sin(angle)
			theta_tmp = 1.0/2.0/sin(p_motion->pose.angle_lenth);

			//r32-r23
			(*(p_motion->pose.axis_vector.point + 0))[0] = theta_tmp*((*(p_motion->pose.rotation_matxx.point + 1))[2]- \
			(*(p_motion->pose.rotation_matxx.point + 2))[1]);
			//r13-r31
			(*(p_motion->pose.axis_vector.point + 0))[1] = theta_tmp*((*(p_motion->pose.rotation_matxx.point + 0))[1]- \
			(*(p_motion->pose.rotation_matxx.point + 0))[2]);
			//r21 - r12
			(*(p_motion->pose.axis_vector.point + 0))[2] = theta_tmp*((*(p_motion->pose.rotation_matxx.point + 2))[0]- \
			(*(p_motion->pose.rotation_matxx.point + 1))[0]);
			}
			*/
			if (r2AxisAngle(&p_motion->pose.rotation_matxx, &p_motion->pose.axis_vector, &p_motion->pose.angle_lenth) == 0)
			{
				p_motion->pose.axis_angle_flag = 1;
			}
			else
			{
				p_motion->pose.axis_angle_flag = 0;
			}
			p_motion->pose.arc_length = p_motion->pose.angle_lenth;

			//calculate the angle motion profile
			calculate_motion_profile_with_limited_jerk(p_motion, COORD_TYPE_CART_ROT, 0);

			//check which part time is longer, if translate time longer than rotate time,then let rotate move as translate.
			//if translate time < rotate time.then rotate do not follow the translate.maybe translate has finished and rotate is still moving

			if (p_motion->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SAME)
			{
				//check which part time is longer, if translate time longer than rotate time,then let rotate move as translate.
				if (p_motion->trajectory.total_interval >= p_motion->traj_cart_rot.total_interval)
				{
					if (fabs(p_motion->traj_cart_rot.total_interval) > MOTION_MODULE_CONSTANT_MIN_POSITIVE)
					{
						// t1 	= t(2) - t(1);
						// t11  = t(3) - t(2) = t(1)- t(0);
						// t3 	= t(4) - t(3);
						// t2 	= t(6) - t(5);
						// t22  = t(7) - t(6) = t(5)- t(4);

						// Vmax = pos / (t11 + 1/2 * t1 + t22 + 1/2 * t2 + t3)*sampletime

						vmax_tmp = (p_motion->pose.angle_lenth / ((p_motion->trajectory.time_series[1] + \
							p_motion->trajectory.time_series[5] - p_motion->trajectory.time_series[4] + \
							0.5*(p_motion->trajectory.time_series[2] - p_motion->trajectory.time_series[1] + \
								p_motion->trajectory.time_series[6] - p_motion->trajectory.time_series[5]) + \
							p_motion->trajectory.time_series[4] - p_motion->trajectory.time_series[3])* \
							p_motion->profile.profile_sample_time));
						//-------------------------------------------------------------------------------------------
						// J1 = Vmax / ((t11^2 + t1 * t11)*Ts^2);
						// J2 = Vmax / ((t22^2 + t2 * t22)*Ts^2);
						//calculate jerk series
						p_motion->traj_cart_rot.jerk_series[0] = vmax_tmp / (p_motion->trajectory.time_series[1] * \
							p_motion->trajectory.time_series[2] * \
							p_motion->profile.profile_sample_time* \
							p_motion->profile.profile_sample_time);

						p_motion->traj_cart_rot.jerk_series[1] = 0.0;

						p_motion->traj_cart_rot.jerk_series[2] = -p_motion->traj_cart_rot.jerk_series[0];

						p_motion->traj_cart_rot.jerk_series[3] = 0.0;

						p_motion->traj_cart_rot.jerk_series[4] = -vmax_tmp / ((p_motion->trajectory.time_series[5] - \
							p_motion->trajectory.time_series[4])* \
							(p_motion->trajectory.time_series[6] - \
								p_motion->trajectory.time_series[4])* \
							p_motion->profile.profile_sample_time* \
							p_motion->profile.profile_sample_time);

						p_motion->traj_cart_rot.jerk_series[5] = 0.0;

						p_motion->traj_cart_rot.jerk_series[6] = -p_motion->traj_cart_rot.jerk_series[4];

						p_motion->traj_cart_rot.jerk_series[7] = 0.0;


						// initial states for acc, dec, vel, and pos
						p_motion->traj_cart_rot.acc_series[0] = 0.0;
						p_motion->traj_cart_rot.vel_series[0] = 0.0;
						p_motion->traj_cart_rot.pos_series[0] = 0.0;

						for (counter = 1; counter < 8; ++counter)
						{
							//save time series
							p_motion->traj_cart_rot.time_series[counter - 1] = p_motion->trajectory.time_series[counter - 1];
							//get time interval
							time_interval = p_motion->trajectory.time_series[counter] - p_motion->trajectory.time_series[counter - 1];
							time_interval *= p_motion->profile.profile_sample_time;

							//calculate acc series
							calculate_acceleration_from_profile(time_interval,
								p_motion->traj_cart_rot.jerk_series[counter - 1],
								p_motion->traj_cart_rot.acc_series[counter - 1],
								&p_motion->traj_cart_rot.acc_series[counter]);
							//calculate vel series
							calculate_velocity_from_profile(time_interval,
								p_motion->traj_cart_rot.jerk_series[counter - 1],
								p_motion->traj_cart_rot.acc_series[counter - 1],
								p_motion->traj_cart_rot.vel_series[counter - 1],
								&p_motion->traj_cart_rot.vel_series[counter]);
							//caculate pose series
							calculate_position_from_profile(time_interval,
								p_motion->traj_cart_rot.jerk_series[counter - 1],
								p_motion->traj_cart_rot.acc_series[counter - 1],
								p_motion->traj_cart_rot.vel_series[counter - 1],
								p_motion->traj_cart_rot.pos_series[counter - 1],
								&p_motion->traj_cart_rot.pos_series[counter]);
						}
						//now counter is 8
						p_motion->traj_cart_rot.time_series[counter - 1] = p_motion->trajectory.time_series[counter - 1];
						p_motion->traj_cart_rot.total_interval = p_motion->trajectory.total_interval;
					}
					else
					{
						for (counter = 0; counter < 8; ++counter)
						{
							p_motion->traj_cart_rot.time_series[counter] = p_motion->trajectory.time_series[counter];
						}
						p_motion->traj_cart_rot.total_interval = p_motion->trajectory.total_interval;
					}

				}
				else
				{
					if (p_motion->trajectory.total_interval > MOTION_MODULE_CONSTANT_MIN_POSITIVE)
					{
						// t1 	= t(2) - t(1);
						// t11  = t(3) - t(2) = t(1)- t(0);
						// t3 	= t(4) - t(3);
						// t2 	= t(6) - t(5);
						// t22  = t(7) - t(6) = t(5)- t(4);

						// Vmax = pos / (t11 + 1/2 * t1 + t22 + 1/2 * t2 + t3)*sampletime

						vmax_tmp = (p_motion->pose.arc_lenth_trans / ((p_motion->traj_cart_rot.time_series[1] + \
							p_motion->traj_cart_rot.time_series[5] - p_motion->traj_cart_rot.time_series[4] + \
							0.5*(p_motion->traj_cart_rot.time_series[2] - p_motion->traj_cart_rot.time_series[1] + \
								p_motion->traj_cart_rot.time_series[6] - p_motion->traj_cart_rot.time_series[5]) + \
							p_motion->traj_cart_rot.time_series[4] - p_motion->traj_cart_rot.time_series[3])* \
							p_motion->profile.profile_sample_time));
						//-------------------------------------------------------------------------------------------
						// J1 = Vmax / ((t11^2 + t1 * t11)*Ts^2);
						// J2 = Vmax / ((t22^2 + t2 * t22)*Ts^2);
						//calculate jerk series
						p_motion->trajectory.jerk_series[0] = vmax_tmp / (p_motion->traj_cart_rot.time_series[1] * \
							p_motion->traj_cart_rot.time_series[2] * \
							p_motion->profile.profile_sample_time* \
							p_motion->profile.profile_sample_time);

						p_motion->trajectory.jerk_series[1] = 0.0;

						p_motion->trajectory.jerk_series[2] = -p_motion->trajectory.jerk_series[0];

						p_motion->trajectory.jerk_series[3] = 0.0;

						p_motion->trajectory.jerk_series[4] = -vmax_tmp / ((p_motion->traj_cart_rot.time_series[5] - \
							p_motion->traj_cart_rot.time_series[4])* \
							(p_motion->traj_cart_rot.time_series[6] - \
								p_motion->traj_cart_rot.time_series[4])* \
							p_motion->profile.profile_sample_time* \
							p_motion->profile.profile_sample_time);

						p_motion->trajectory.jerk_series[5] = 0.0;

						p_motion->trajectory.jerk_series[6] = -p_motion->trajectory.jerk_series[4];

						p_motion->trajectory.jerk_series[7] = 0.0;


						// initial states for acc, dec, vel, and pos
						p_motion->trajectory.acc_series[0] = 0.0;
						p_motion->trajectory.vel_series[0] = 0.0;
						p_motion->trajectory.pos_series[0] = 0.0;

						for (counter = 1; counter < 8; ++counter)
						{
							//save time series
							p_motion->trajectory.time_series[counter - 1] = p_motion->traj_cart_rot.time_series[counter - 1];
							//get time interval
							time_interval = p_motion->traj_cart_rot.time_series[counter] - p_motion->traj_cart_rot.time_series[counter - 1];
							time_interval *= p_motion->profile.profile_sample_time;

							//calculate acc series
							calculate_acceleration_from_profile(time_interval,
								p_motion->trajectory.jerk_series[counter - 1],
								p_motion->trajectory.acc_series[counter - 1],
								&p_motion->trajectory.acc_series[counter]);
							//calculate vel series
							calculate_velocity_from_profile(time_interval,
								p_motion->trajectory.jerk_series[counter - 1],
								p_motion->trajectory.acc_series[counter - 1],
								p_motion->trajectory.vel_series[counter - 1],
								&p_motion->trajectory.vel_series[counter]);
							//caculate pose series
							calculate_position_from_profile(time_interval,
								p_motion->trajectory.jerk_series[counter - 1],
								p_motion->trajectory.acc_series[counter - 1],
								p_motion->trajectory.vel_series[counter - 1],
								p_motion->trajectory.pos_series[counter - 1],
								&p_motion->trajectory.pos_series[counter]);
						}
						//now counter is 8
						p_motion->trajectory.time_series[counter - 1] = p_motion->traj_cart_rot.time_series[counter - 1];
						p_motion->trajectory.total_interval = p_motion->traj_cart_rot.total_interval;
					}
					else
					{
						// if total_interval=0,let  time_series = time_series of the longer
						// and the jerk ,acc ,vel serise=0
						for (counter = 0; counter < 8; ++counter)
						{
							p_motion->trajectory.time_series[counter] = p_motion->traj_cart_rot.time_series[counter];
						}
						p_motion->trajectory.total_interval = p_motion->traj_cart_rot.total_interval;

					}
				}

			}
			//time scale method
			else
			{
				//check which part time is longer, if translate time longer than rotate time,then let rotate move as translate.
				if (p_motion->trajectory.total_interval >= p_motion->traj_cart_rot.total_interval)
				{
					//now we just use scaled method for cartesian line
					if (fabs(p_motion->traj_cart_rot.total_interval) > MOTION_MODULE_CONSTANT_MIN_POSITIVE)
					{

						//get the scale k
						k = p_motion->trajectory.total_interval / p_motion->traj_cart_rot.total_interval;
						k2 = k*k;
						k3 = k2*k;

						//according to the scale k to update other joint profiles,do not calculate again,just update
						for (counter = 0; counter < 8; ++counter)
						{
							/*if(p_motion->traj_cart_rot.time_series[counter]< MOTION_MODULE_CONSTANT_MIN_POSITIVE)
							{
							comp = 1;
							}
							else
							{
							comp = (k*p_motion->traj_cart_rot.time_series[counter])/ceil(k*p_motion->traj_cart_rot.time_series[counter]);
							}*/

							//p_motion->traj_cart_rot.time_series[counter] = floor(k*p_motion->traj_cart_rot.time_series[counter]);
							p_motion->traj_cart_rot.time_series[counter] = ceil(k*p_motion->traj_cart_rot.time_series[counter]);
							p_motion->traj_cart_rot.jerk_series[counter] /= (k3);
							p_motion->traj_cart_rot.acc_series[counter] /= (k2);
							p_motion->traj_cart_rot.vel_series[counter] /= (k);

							/*p_motion->traj_cart_rot.jerk_series[counter] *=(comp*comp*comp);
							p_motion->traj_cart_rot.acc_series[counter]  *=(comp*comp);
							p_motion->traj_cart_rot.vel_series[counter]  *=(comp);
							//pos do not change*/
						}
						p_motion->traj_cart_rot.total_interval = p_motion->trajectory.total_interval;
						p_motion->traj_cart_rot.time_series[7] = p_motion->trajectory.time_series[7];
					}
					else
					{
						for (counter = 0; counter < 8; ++counter)
						{
							p_motion->traj_cart_rot.time_series[counter] = p_motion->trajectory.time_series[counter];
						}
						p_motion->traj_cart_rot.total_interval = p_motion->trajectory.total_interval;
					}
				}
				else
				{
					if (p_motion->trajectory.total_interval > MOTION_MODULE_CONSTANT_MIN_POSITIVE)
					{
						k = p_motion->traj_cart_rot.total_interval / p_motion->trajectory.total_interval;
						k2 = k*k;
						k3 = k2*k;
						for (counter = 0; counter < 8; ++counter)
						{
							/*if(p_motion->trajectory.time_series[counter]< MOTION_MODULE_CONSTANT_MIN_POSITIVE)
							{
							comp = 1;
							}
							else
							{
							comp = (k*p_motion->trajectory.time_series[counter])/ceil(k*p_motion->trajectory.time_series[counter]);
							}*/
							//p_motion->trajectory.time_series[counter] = floor(k*p_motion->trajectory.time_series[counter]);
							p_motion->trajectory.time_series[counter] = ceil(k*p_motion->trajectory.time_series[counter]);
							p_motion->trajectory.jerk_series[counter] /= (k3);
							p_motion->trajectory.acc_series[counter] /= (k2);
							p_motion->trajectory.vel_series[counter] /= (k);

							/*p_motion->trajectory.jerk_series[counter] *=(comp*comp*comp);
							p_motion->trajectory.acc_series[counter]  *=(comp*comp);
							p_motion->trajectory.vel_series[counter]  *=(comp);*/
						}
						p_motion->trajectory.total_interval = p_motion->traj_cart_rot.total_interval;
						p_motion->trajectory.time_series[7] = p_motion->traj_cart_rot.time_series[7];
					}
					else
					{
						// if total_interval=0,let  time_series = time_series of the longer
						// and the jerk ,acc ,vel serise=0
						for (counter = 0; counter < 8; ++counter)
						{
							p_motion->trajectory.time_series[counter] = p_motion->traj_cart_rot.time_series[counter];
						}
						p_motion->trajectory.total_interval = p_motion->traj_cart_rot.total_interval;

					}
				}
			}
		}

		break;
	}

	case TRAJECTORY_MODULE_MOTION_TYPE_NONE:
		break;
	default:
		break;
	}

	return;
}
//----------------------------------------------------------------------------------------
void calculate_blending_parameters(motion_block* current, motion_block* last, robot_config_module* m_cfg)
{
	// now blending just for pos ,not rotate. because rotate we use the angle-axis method,we just do the angle motion planning
	// we can not do the axis blending ,at first , i think just do angle blending is ok,
	// but, this is wrong idea,because the axis is different for two motion block .
	// so now for blending we just for pos.
	if ((current->blending.is_need_blending) && (last->state == TRAJECTORY_MODULE_MOTION_BLOCK_STATE_RUNNING))
	{
		//Need Blending and is the last motion ready to run?
		current->blending.is_ready_blending = 0;

		// Calculate Blending parameters
		switch (current->blending.type)
		{
		case TRAJECTORY_MODULE_BLENDING_PERCENTAGE:

			//according the coordinate type
			if ((last->prfcoord_type == COORD_TYPE_JOINT) && (last->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_P2P))
			{
				if (last->jointp2p_sync == JOINTP2P_TYPE_TIME_SCALE)
				{
					//this method ,every joint of the (acc part,even part, dec part) are not same,so do not use blending now
					current->blending.is_need_blending = 0;
					break;
				}
				else if (last->jointp2p_sync == JOINTP2P_TYPE_TIME_SAME)
				{
					//because every joint time is same ,so blending parameter is same

					//1. calculate the length of blending interval
					// time interval is from the end of last blending ,so this left time can blending
					current->blending.blending_time_length = last->traj_joint[0].total_interval - last->blending.blending_time_length;

					if (current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_P2P)
					{
						//if the current total time less than the current blending time lenth ,then you smaller time part
						if (current->traj_joint[0].total_interval < current->blending.blending_time_length)
						{
							if (current->jointp2p_sync == JOINTP2P_TYPE_TIME_SCALE)
							{
								//this method ,every joint of the (acc part,even part, dec part) are not same,so do not use blending now
								//is_ready_blending is 0;
								current->blending.is_need_blending = 0;
								break;
							}
							else if (current->jointp2p_sync == JOINTP2P_TYPE_TIME_SAME)
							{
								// Percentage of whole motion time interval
								current->blending.blending_time_length = (Uint32)(current->traj_joint[0].total_interval * current->blending.parameters);
							}
							else
							{
								current->blending.is_need_blending = 0;
								break;
							}
						}
						else
						{
							//use smaller time part
							// Percentage of whole motion time interval
							current->blending.blending_time_length = (Uint32)(current->blending.blending_time_length * current->blending.parameters);
						}
					}
					else if ((current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_LIN))
					{
						//if the current total time less than the current blending time lenth ,then you smaller time part
						if (current->trajectory.total_interval < current->blending.blending_time_length)
						{
							if (current->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SCALE)
							{
								//this method ,pos and posture (acc part,even part, dec part) time are not same,so do not use blending now
								current->blending.is_need_blending = 0;
								break;
							}
							else if (current->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SAME)
							{
								// Percentage of whole motion time interval
								current->blending.blending_time_length = (Uint32)(current->trajectory.total_interval * current->blending.parameters);
							}
							else
							{
								current->blending.is_need_blending = 0;
								break;
							}
						}
						else
						{
							//use smaller time part
							// Percentage of whole motion time interval
							current->blending.blending_time_length = (Uint32)(current->blending.blending_time_length * current->blending.parameters);
						}
					}
					else if ((current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_CIRCL))
					{
						//if the current total time less than the current blending time lenth ,then you smaller time part
						if (current->trajectory.total_interval < current->blending.blending_time_length)
						{
							if (current->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SCALE)
							{
								//this method ,pos and posture (acc part,even part, dec part) time are not same,so do not use blending now
								current->blending.is_need_blending = 0;
								break;
							}
							else if (current->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SAME)
							{
								// Percentage of whole motion time interval
								current->blending.blending_time_length = (Uint32)(current->trajectory.total_interval * current->blending.parameters);
							}
							else
							{
								current->blending.is_need_blending = 0;
								break;
							}
						}
						else
						{
							//use smaller time part
							// Percentage of whole motion time interval
							current->blending.blending_time_length = (Uint32)(current->blending.blending_time_length * current->blending.parameters);
						}

					}
					else
					{
						current->blending.is_need_blending = 0;
						break;
					}


					// 2. inverse of the blending interval time length
					if (current->blending.blending_time_length > 0)
					{
						current->blending.blending_time_length_inv = 1.0 / ((double)current->blending.blending_time_length);
					}
					else
					{
						current->blending.blending_time_length_inv = 0.0;
					}
					// 3. get the start blending time
					current->blending.blending_start_time = last->traj_joint[0].time_series[7] - current->blending.blending_time_length;
				}
			}
			else if ((last->prfcoord_type == COORD_TYPE_CART) && ((last->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_LIN)))
			{
				if (last->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SCALE)
				{
					//this method ,every joint of the (acc part,even part, dec part) are not same,so do not use blending now
					current->blending.is_need_blending = 0;
					break;
				}
				else if (last->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SAME)
				{
					//1. calculate the length of blending interval
					// time interval is from the end of last blending ,so this left time can blending
					current->blending.blending_time_length = last->trajectory.total_interval - last->blending.blending_time_length;

					if (current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_P2P)
					{
						if (current->traj_joint[0].total_interval < current->blending.blending_time_length)
						{
							if (current->jointp2p_sync == JOINTP2P_TYPE_TIME_SCALE)
							{
								//this method ,every joint of the (acc part,even part, dec part) are not same,so do not use blending now
								//is_ready_blending is 0;
								current->blending.is_need_blending = 0;
								break;
							}
							else if (current->jointp2p_sync == JOINTP2P_TYPE_TIME_SAME)
							{
								// Percentage of whole motion time interval
								current->blending.blending_time_length = (Uint32)(current->traj_joint[0].total_interval * current->blending.parameters);
							}
							else
							{
								current->blending.is_need_blending = 0;
								break;
							}
						}
						else
						{
							//use smaller time part
							// Percentage of whole motion time interval
							current->blending.blending_time_length = (Uint32)(current->blending.blending_time_length * current->blending.parameters);
						}

					}
					else if ((current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_LIN))
					{
						//if the current total time less than the current blending time lenth ,then you smaller time part
						if (current->trajectory.total_interval < current->blending.blending_time_length)
						{
							if (current->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SCALE)
							{
								//this method ,pos and posture (acc part,even part, dec part) time are not same,so do not use blending now
								current->blending.is_need_blending = 0;
								break;
							}
							else if (current->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SAME)
							{
								// Percentage of whole motion time interval
								current->blending.blending_time_length = (Uint32)(current->trajectory.total_interval * current->blending.parameters);
							}
							else
							{
								current->blending.is_need_blending = 0;
								break;
							}
						}
						else
						{
							//use smaller time part
							// Percentage of whole motion time interval
							current->blending.blending_time_length = (Uint32)(current->blending.blending_time_length * current->blending.parameters);
						}
					}
					else if ((current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_CIRCL))
					{
						//if the current total time less than the current blending time lenth ,then you smaller time part
						if (current->trajectory.total_interval < current->blending.blending_time_length)
						{
							if (current->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SCALE)
							{
								//this method ,pos and posture (acc part,even part, dec part) time are not same,so do not use blending now
								current->blending.is_need_blending = 0;
								break;
							}
							else if (current->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SAME)
							{
								// Percentage of whole motion time interval
								current->blending.blending_time_length = (Uint32)(current->trajectory.total_interval * current->blending.parameters);
							}
							else
							{
								current->blending.is_need_blending = 0;
								break;
							}
						}
						else
						{
							//use smaller time part
							// Percentage of whole motion time interval
							current->blending.blending_time_length = (Uint32)(current->blending.blending_time_length * current->blending.parameters);
						}
					}
					else
					{
						current->blending.is_need_blending = 0;
						break;
					}


					// 2. inverse of the blending interval time length
					if (current->blending.blending_time_length > 0)
					{
						current->blending.blending_time_length_inv = 1.0 / ((double)current->blending.blending_time_length);
					}
					else
					{
						current->blending.blending_time_length_inv = 0.0;
					}
					// 3. get the start blending time
					current->blending.blending_start_time = last->trajectory.time_series[7] - current->blending.blending_time_length;
				}

			}
			else if ((last->prfcoord_type == COORD_TYPE_CART) && ((last->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_CIRCL)))
			{
				if (last->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SCALE)
				{
					//this method ,every joint of the (acc part,even part, dec part) are not same,so do not use blending now
					current->blending.is_need_blending = 0;
					break;
				}
				else if (last->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SAME)
				{
					//1. calculate the length of blending interval
					// time interval is from the end of last blending ,so this left time can blending
					current->blending.blending_time_length = last->trajectory.total_interval - last->blending.blending_time_length;

					if (current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_P2P)
					{
						if (current->traj_joint[0].total_interval < current->blending.blending_time_length)
						{
							if (current->jointp2p_sync == JOINTP2P_TYPE_TIME_SCALE)
							{
								//this method ,every joint of the (acc part,even part, dec part) are not same,so do not use blending now
								//is_ready_blending is 0;
								current->blending.is_need_blending = 0;
								break;
							}
							else if (current->jointp2p_sync == JOINTP2P_TYPE_TIME_SAME)
							{
								// Percentage of whole motion time interval
								current->blending.blending_time_length = (Uint32)(current->traj_joint[0].total_interval * current->blending.parameters);
							}
							else
							{
								current->blending.is_need_blending = 0;
								break;
							}
						}
						else
						{
							//use smaller time part
							// Percentage of whole motion time interval
							current->blending.blending_time_length = (Uint32)(current->blending.blending_time_length * current->blending.parameters);
						}

					}
					else if ((current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_LIN))
					{
						//if the current total time less than the current blending time lenth ,then you smaller time part
						if (current->trajectory.total_interval < current->blending.blending_time_length)
						{
							if (current->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SCALE)
							{
								//this method ,pos and posture (acc part,even part, dec part) time are not same,so do not use blending now
								current->blending.is_need_blending = 0;
								break;
							}
							else if (current->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SAME)
							{
								// Percentage of whole motion time interval
								current->blending.blending_time_length = (Uint32)(current->trajectory.total_interval * current->blending.parameters);
							}
							else
							{
								current->blending.is_need_blending = 0;
								break;
							}
						}
						else
						{
							//use smaller time part
							// Percentage of whole motion time interval
							current->blending.blending_time_length = (Uint32)(current->blending.blending_time_length * current->blending.parameters);
						}
					}
					else if ((current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_CIRCL))
					{
						//if the current total time less than the current blending time lenth ,then you smaller time part
						if (current->trajectory.total_interval < current->blending.blending_time_length)
						{
							if (current->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SCALE)
							{
								//this method ,pos and posture (acc part,even part, dec part) time are not same,so do not use blending now
								current->blending.is_need_blending = 0;
								break;
							}
							else if (current->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SAME)
							{
								// Percentage of whole motion time interval
								current->blending.blending_time_length = (Uint32)(current->trajectory.total_interval * current->blending.parameters);
							}
							else
							{
								current->blending.is_need_blending = 0;
								break;
							}
						}
						else
						{
							//use smaller time part
							// Percentage of whole motion time interval
							current->blending.blending_time_length = (Uint32)(current->blending.blending_time_length * current->blending.parameters);
						}
					}
					else
					{
						current->blending.is_need_blending = 0;
						break;
					}


					// 2. inverse of the blending interval time length
					if (current->blending.blending_time_length > 0)
					{
						current->blending.blending_time_length_inv = 1.0 / ((double)current->blending.blending_time_length);
					}
					else
					{
						current->blending.blending_time_length_inv = 0.0;
					}
					// 3. get the start blending time
					current->blending.blending_start_time = last->trajectory.time_series[7] - current->blending.blending_time_length;
				}
			}
			else
			{
				current->blending.is_need_blending = 0;
				break;
			}

			// Blending ready flag
			current->blending.is_ready_blending = 1;
			break;
		case TRAJECTORY_MODULE_BLENDING_AUTO:

			//according the coordinate type
			if ((last->prfcoord_type == COORD_TYPE_JOINT) && (last->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_P2P))
			{
				if (last->jointp2p_sync == JOINTP2P_TYPE_TIME_SCALE)
				{
					//this method ,every joint of the (acc part,even part, dec part) are not same,so do not use blending now
					//is_ready_blending is 0;
					current->blending.is_need_blending = 0;
					break;
				}
				else if (last->jointp2p_sync == JOINTP2P_TYPE_TIME_SAME)
				{
					//1. calculate the length of blending interval
					// get the length of decelerate time of last motion
					current->blending.blending_time_length = last->traj_joint[0].time_series[7] - last->traj_joint[0].time_series[4];

					//get the accelerate time of the current motion,according to the type
					if (current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_P2P)
					{
						if (current->jointp2p_sync == JOINTP2P_TYPE_TIME_SCALE)
						{
							//this method ,every joint of the (acc part,even part, dec part) are not same,so do not use blending now
							//is_ready_blending is 0;
							current->blending.is_need_blending = 0;
							break;
						}
						else if (current->jointp2p_sync == JOINTP2P_TYPE_TIME_SAME)
						{
							current->blending.blending_start_time = current->traj_joint[0].time_series[3] - current->traj_joint[0].time_series[0];
						}
						else
						{
							current->blending.is_need_blending = 0;
							break;
						}
					}
					else if ((current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_LIN))
					{
						if (current->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SCALE)
						{
							//this method ,pos and posture (acc part,even part, dec part) time are not same,so do not use blending now
							current->blending.is_need_blending = 0;
							break;
						}
						else if (current->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SAME)
						{
							current->blending.blending_start_time = current->trajectory.time_series[3] - current->trajectory.time_series[0];
						}
						else
						{
							current->blending.is_need_blending = 0;
							break;
						}
					}
					else if ((current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_CIRCL))
					{
						if (current->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SCALE)
						{
							//this method ,pos and posture (acc part,even part, dec part) time are not same,so do not use blending now
							current->blending.is_need_blending = 0;
							break;
						}
						else if (current->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SAME)
						{
							current->blending.blending_start_time = current->trajectory.time_series[3] - current->trajectory.time_series[0];
						}
						else
						{
							current->blending.is_need_blending = 0;
							break;
						}
					}
					else
					{
						current->blending.is_need_blending = 0;
						break;
					}

					//if the current decelerate time bigger than the accelerate time
					if (current->blending.blending_time_length > current->blending.blending_start_time)
					{
						// lenth set to accelerate time
						current->blending.blending_time_length = current->blending.blending_start_time;
					}

					// 2. inverse of the blending interval time length
					if (current->blending.blending_time_length > 0)
					{
						current->blending.blending_time_length_inv = 1.0 / ((double)current->blending.blending_time_length);
					}
					else
					{
						current->blending.blending_time_length_inv = 0.0;
					}
					// 3. get the start blending time
					current->blending.blending_start_time = last->traj_joint[0].time_series[7] - current->blending.blending_time_length;
				}
			}
			else if ((last->prfcoord_type == COORD_TYPE_CART) && ((last->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_LIN)))
			{
				if (last->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SCALE)
				{
					//this method ,pos and posture (acc part,even part, dec part) time are not same,so do not use blending now
					current->blending.is_need_blending = 0;
					break;
				}
				else if (last->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SAME)
				{
					//1. calculate the length of blending interval
					// get the length of decelerate time of last motion
					current->blending.blending_time_length = last->trajectory.time_series[7] - last->trajectory.time_series[4];

					//get the accelerate time of the current motion,according to the type
					if (current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_P2P)
					{
						if (current->jointp2p_sync == JOINTP2P_TYPE_TIME_SCALE)
						{
							//this method ,every joint of the (acc part,even part, dec part) are not same,so do not use blending now
							//is_ready_blending is 0;
							current->blending.is_need_blending = 0;
							break;
						}
						else if (current->jointp2p_sync == JOINTP2P_TYPE_TIME_SAME)
						{
							current->blending.blending_start_time = current->traj_joint[0].time_series[3] - current->traj_joint[0].time_series[0];
						}
						else
						{
							current->blending.is_need_blending = 0;
							break;
						}
					}
					else if ((current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_LIN))
					{
						if (current->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SCALE)
						{
							//this method ,pos and posture (acc part,even part, dec part) time are not same,so do not use blending now
							current->blending.is_need_blending = 0;
							break;
						}
						else if (current->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SAME)
						{
							current->blending.blending_start_time = current->trajectory.time_series[3] - current->trajectory.time_series[0];
						}
						else
						{
							current->blending.is_need_blending = 0;
							break;
						}
					}
					else if ((current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_CIRCL))
					{
						if (current->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SCALE)
						{
							//this method ,pos and posture (acc part,even part, dec part) time are not same,so do not use blending now
							current->blending.is_need_blending = 0;
							break;
						}
						else if (current->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SAME)
						{
							current->blending.blending_start_time = current->trajectory.time_series[3] - current->trajectory.time_series[0];
						}
						else
						{
							current->blending.is_need_blending = 0;
							break;
						}
					}
					else
					{
						current->blending.is_need_blending = 0;
						break;
					}

					//if the current decelerate time bigger than the accelerate time
					if (current->blending.blending_time_length > current->blending.blending_start_time)
					{
						// lenth set to accelerate time
						current->blending.blending_time_length = current->blending.blending_start_time;
					}

					// 2. inverse of the blending interval time length
					if (current->blending.blending_time_length > 0)
					{
						current->blending.blending_time_length_inv = 1.0 / ((double)current->blending.blending_time_length);
					}
					else
					{
						current->blending.blending_time_length_inv = 0.0;
					}
					// 3. get the start blending time
					current->blending.blending_start_time = last->trajectory.time_series[7] - current->blending.blending_time_length;
				}
			}
			else if ((last->prfcoord_type == COORD_TYPE_CART) && ((last->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_CIRCL)))
			{
				if (last->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SCALE)
				{
					//this method ,pos and posture (acc part,even part, dec part) time are not same,so do not use blending now
					current->blending.is_need_blending = 0;
					break;
				}
				else if (last->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SAME)
				{
					//1. calculate the length of blending interval
					// get the length of decelerate time of last motion
					current->blending.blending_time_length = last->trajectory.time_series[7] - last->trajectory.time_series[4];

					//get the accelerate time of the current motion,according to the type
					if (current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_P2P)
					{
						if (current->jointp2p_sync == JOINTP2P_TYPE_TIME_SCALE)
						{
							//this method ,every joint of the (acc part,even part, dec part) are not same,so do not use blending now
							//is_ready_blending is 0;
							current->blending.is_need_blending = 0;
							break;
						}
						else if (current->jointp2p_sync == JOINTP2P_TYPE_TIME_SAME)
						{
							current->blending.blending_start_time = current->traj_joint[0].time_series[3] - current->traj_joint[0].time_series[0];
						}
						else
						{
							current->blending.is_need_blending = 0;
							break;
						}
					}
					else if ((current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_LIN))
					{
						if (current->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SCALE)
						{
							//this method ,pos and posture (acc part,even part, dec part) time are not same,so do not use blending now
							current->blending.is_need_blending = 0;
							break;
						}
						else if (current->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SAME)
						{
							current->blending.blending_start_time = current->trajectory.time_series[3] - current->trajectory.time_series[0];
						}
						else
						{
							current->blending.is_need_blending = 0;
							break;
						}
					}
					else if ((current->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_CIRCL))
					{
						if (current->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SCALE)
						{
							//this method ,pos and posture (acc part,even part, dec part) time are not same,so do not use blending now
							current->blending.is_need_blending = 0;
							break;
						}
						else if (current->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SAME)
						{
							current->blending.blending_start_time = current->trajectory.time_series[3] - current->trajectory.time_series[0];
						}
						else
						{
							current->blending.is_need_blending = 0;
							break;
						}
					}
					else
					{
						current->blending.is_need_blending = 0;
						break;
					}

					//if the current decelerate time bigger than the accelerate time
					if (current->blending.blending_time_length > current->blending.blending_start_time)
					{
						// lenth set to accelerate time
						current->blending.blending_time_length = current->blending.blending_start_time;
					}

					// 2. inverse of the blending interval time length
					if (current->blending.blending_time_length > 0)
					{
						current->blending.blending_time_length_inv = 1.0 / ((double)current->blending.blending_time_length);
					}
					else
					{
						current->blending.blending_time_length_inv = 0.0;
					}
					// 3. get the start blending time
					current->blending.blending_start_time = last->trajectory.time_series[7] - current->blending.blending_time_length;
				}
			}
			else
			{
				current->blending.is_need_blending = 0;
				break;
			}
			// Blending ready flag
			current->blending.is_ready_blending = 1;
			break;
		case TRAJECTORY_MODULE_BLENDING_DISTANCE: /* Do not support*/
		case TRAJECTORY_MODULE_BLENDING_NONE:
		default:
			current->blending.is_need_blending = 0;
			current->blending.is_ready_blending = 0;
			current->blending.blending_time_length = 0;
			current->blending.blending_start_time = 0;
			current->blending.blending_time_length_inv = 0;
			break;
		}
	}
	else
	{
		/* Blending ready flag */
		current->blending.is_need_blending = 0;
		current->blending.is_ready_blending = 0;
		current->blending.blending_time_length = 0;
		current->blending.blending_start_time = 0;
		current->blending.blending_time_length_inv = 0;
	}
	return;
}
//----------------------------------------------------------------------------------------
int16 CalcMotionProfile(trajectory_module * m_traj, robot_config_module *m_cfg)
{
	Uint8 id = 0;
	//AsynCalcJogProfile(&gRobot.robot_dev[id].mode.rjog, &gRobot.robot_dev[id].module.joint[0], \
//		&gRobot.robot_dev[id].module.cart, &gRobot.robot_dev[id].cfg);

	motion_block* 	p_motion = m_traj->motion_block_calculate;
	motion_block* 	p_last_motion = m_traj->motion_block_calculate;			//p_last_motion point to the same motion block as p_motion
																			//------------------------------------------------------------------------
	if (p_motion->state != TRAJECTORY_MODULE_MOTION_BLOCK_STATE_USED)
	{
		return -1;
	}
	//------------------------------------------------------------------------
	// Determine motion in the coordinate
	calculate_motion_parameters(m_cfg, p_motion);

	//------------------------------------------------------------------------
	// calculate blending parameters,first get last motion block.
	p_last_motion = get_last_motion_block(m_traj->motion_block_buffer, p_last_motion);
	calculate_blending_parameters(p_motion, p_last_motion, m_cfg);

	//------------------------------------------------------------------------
	// finish the calculate,then the motion block is ready to run
	//m_traj->current_time 	= 0;
	p_motion->state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_RUNNING;

	//------------------------------------------------------------------------
	// calculate motion block pointer point to next motion block
	m_traj->motion_block_calculate = get_next_motion_block(m_traj->motion_block_buffer, m_traj->motion_block_calculate);

	return 0;
}
//----------------------------------------------------------------------------------------
int16 InsertNewMotionBlock(trajectory_module *m_traj, robot_config_module* 	p_config, double* pose_start, double* pose_end, double* pose_aux, double* profile, double* BlendDelayArc, Uint8 motion_type, Uint8 id)
{
	motion_block* new = m_traj->motion_block_new_insert;
	motion_block* last = NULL;
	Uint8 counter = 0;
	Uint8 i;

	//if stop the script flag is set ,then do not insert the new block
	//if (gRobot.robot_dev[id].cfg.stop_script_flag == 1 && gRobot.robot_dev[id].mode.mode == ROBOT_MODULE_SCRIPT_MODE)
	{
	//	return -1;
	}


	/* get last motion block */
	counter = new->motion_block_id;
	if (counter > 0)
	{
		/* point to last motion block */
		last = (motion_block*)(new - 1);
	}
	else
	{
		/* get the buffer motion block */
		counter = MOTION_MODULE_DEFAULT_MOTION_BLOCK_SIZE - 1;
		last = &(m_traj->motion_block_buffer[counter]);
	}
	// motion block is running, and wait to release the motion block
	while (new->state != TRAJECTORY_MODULE_MOTION_BLOCK_STATE_IDL)
	{
		//if stop the script flag is set ,then do not wait the motion block to release
		//if (gRobot.robot_dev[id].cfg.stop_script_flag == 1 && gRobot.robot_dev[id].mode.mode == ROBOT_MODULE_SCRIPT_MODE)
		{
		//	return -1;
		}
		Sleep(10);

		//wait for semaphore
	//	rtems_semaphore_obtain(gRobot.robot_dev[id].mgr.block_finish_sem, RTEMS_WAIT, 0);
	}

	/* insert motion information into motion block */
	if (new->state == TRAJECTORY_MODULE_MOTION_BLOCK_STATE_IDL)
	{
		/* motion block is new or finished */
		new->motion_type = motion_type;

		switch (new->motion_type)
		{
		case TRAJECTORY_MODULE_MOTION_TYPE_P2P:
		{
			/*  joints commands may be not in the location configuration
			*  Change commands to be consistent with location configuration
			*  For example, left hand to right hand,because inverse
			*  kinematics consider the location configuration
			* */
			//input is degree, output is also degree
			gRobot.robot_dev[id].module.kinematics.pfFK(&(gRobot.robot_dev[id].module.cart), &(gRobot.robot_dev[id].module.joint[0]), \
				&(gRobot.robot_dev[id].module.kinematics), \
				p_config, new->pose.pose_end, pose_end);
			gRobot.robot_dev[id].module.kinematics.pfIK(&(gRobot.robot_dev[id].module.cart), &(gRobot.robot_dev[id].module.joint[0]), \
				&(gRobot.robot_dev[id].module.kinematics), \
				p_config, new->pose.pose_end, pose_end, 0);

			// assign end joint angle in degree
			matxx_assign(&new->pose.joint_end_vector, pose_end, p_config->joint_dim);

			//copy last end vector to the new start vector or set start pose
			if (pose_start == NULL)
			{
				matxx_copy(&last->pose.joint_end_vector, &new->pose.joint_start_vector);
			}
			else
			{
				memcpy(new->pose.joint_start_vector.point[0], pose_start, p_config->joint_dim * sizeof(double));
			}

			//now we need profile [0,0,0,0]----[90,80,100,20]  100---mm 90--degree
			//MOTION PARAMETER profile value is % value
			for (i = 0; i < p_config->joint_dim; ++i)
			{
				new->prf_joint[i].profile_vel = profile[0] * gRobot.robot_dev[id].module.joint[i].prm.joint_sys_vel * MOTION_MODULE_CONSTANT_PERCENTAGE;
				new->prf_joint[i].profile_acc = profile[1] * gRobot.robot_dev[id].module.joint[i].prm.joint_sys_acc * MOTION_MODULE_CONSTANT_PERCENTAGE;
				new->prf_joint[i].profile_tacc = profile[2]; //s
				new->prf_joint[i].profile_dec = profile[3] * gRobot.robot_dev[id].module.joint[i].prm.joint_sys_dec * MOTION_MODULE_CONSTANT_PERCENTAGE;
				new->prf_joint[i].profile_tdec = profile[4]; //s
				new->prf_joint[i].profile_sample_time = MOTION_MODULE_SAMPLE_TIME;
			}

			//set the p2p method,now just for test
			new->jointp2p_sync = JOINTP2P_TYPE_TIME_SAME;

			break;
		}
		case TRAJECTORY_MODULE_MOTION_TYPE_LIN:
		{
			//get end POSE ,now do not consider the pose's angle part,just position so use 3
			for (counter = 0; counter < p_config->cart_dim; counter++)
			{
				new->pose.pose_end[counter] = pose_end[counter];
				// Last motion end pose is the new motion start pose or set start pose
				if (pose_start == NULL)
				{
					new->pose.pose_start[counter] = last->pose.pose_end[counter];
				}
				else
				{
					new->pose.pose_start[counter] = pose_start[counter];
				}
			}

			//to get joint end vector
			gRobot.robot_dev[id].module.kinematics.pfIK(&(gRobot.robot_dev[id].module.cart), &(gRobot.robot_dev[id].module.joint[0]), \
				&(gRobot.robot_dev[id].module.kinematics), \
				p_config, new->pose.pose_end, *(new->pose.joint_end_vector.point), 0);

			// MOTION PARAMETER for cartesian translate move
			new->profile.profile_vel = profile[0] * gRobot.robot_dev[id].module.cart.prm.cart_trans_sys_vel * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->profile.profile_acc = profile[1] * gRobot.robot_dev[id].module.cart.prm.cart_trans_sys_acc * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->profile.profile_tacc = profile[2];  //s
			new->profile.profile_dec = profile[3] * gRobot.robot_dev[id].module.cart.prm.cart_trans_sys_dec * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->profile.profile_tdec = profile[4];  //s
			new->profile.profile_sample_time = MOTION_MODULE_SAMPLE_TIME;


			// MOTION PARAMETER for cartesian rotate move
			new->prf_cart_rot.profile_vel = profile[0] * gRobot.robot_dev[id].module.cart.prm.cart_rotate_sys_vel * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->prf_cart_rot.profile_acc = profile[1] * gRobot.robot_dev[id].module.cart.prm.cart_rotate_sys_acc * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->prf_cart_rot.profile_tacc = profile[2];
			new->prf_cart_rot.profile_dec = profile[3] * gRobot.robot_dev[id].module.cart.prm.cart_rotate_sys_dec * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->prf_cart_rot.profile_tdec = profile[4];
			new->prf_cart_rot.profile_sample_time = MOTION_MODULE_SAMPLE_TIME;

			//set the line pose method,now just for test
			new->cartline_method = CARTLINE_TYPE_AXIS_ANGLE;
			new->cartline_sync = CARTLINE_TYPE_SYNC_TIME_SAME;
			//new->cartline_sync = CARTLINE_TYPE_SYNC_TIME_SCALE;
			break;
		}
		case TRAJECTORY_MODULE_MOTION_TYPE_CIRCL:
		{
			// need a total circle
			if ((Uint8)BlendDelayArc[4] == 1)
			{
				new->arc_flag = 1;
			}
			else
			{
				new->arc_flag = 0;
			}

			// read end POSE and auxiliary POSE
			for (counter = 0; counter < p_config->cart_dim; counter++)
			{
				if (new->arc_flag == 1)
				{
					new->pose.pose_end[counter] = last->pose.pose_end[counter];
					new->pose.pose_aux2[counter] = pose_end[counter];
				}
				else
				{
					new->pose.pose_end[counter] = pose_end[counter];
					new->pose.pose_aux2[counter] = pose_end[counter];
				}
				new->pose.pose_aux1[counter] = pose_aux[counter];
				// Last motion end pose is the new motion start pose or set start pose
				if (pose_start == NULL)
				{
					new->pose.pose_start[counter] = last->pose.pose_end[counter];
				}
				else
				{
					new->pose.pose_start[counter] = pose_start[counter];
				}
			}
			//to get joint end vector
			gRobot.robot_dev[id].module.kinematics.pfIK(&(gRobot.robot_dev[id].module.cart), &(gRobot.robot_dev[id].module.joint[0]), \
				&(gRobot.robot_dev[id].module.kinematics), \
				p_config, new->pose.pose_end, *new->pose.joint_end_vector.point, 0);

			// MOTION PARAMETER for cartesian translate move
			new->profile.profile_vel = profile[0] * gRobot.robot_dev[id].module.cart.prm.cart_trans_sys_vel * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->profile.profile_acc = profile[1] * gRobot.robot_dev[id].module.cart.prm.cart_trans_sys_acc * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->profile.profile_tacc = profile[2];
			new->profile.profile_dec = profile[3] * gRobot.robot_dev[id].module.cart.prm.cart_trans_sys_dec * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->profile.profile_tdec = profile[4];
			new->profile.profile_sample_time = MOTION_MODULE_SAMPLE_TIME;


			// MOTION PARAMETER for cartesian rotate  move
			new->prf_cart_rot.profile_vel = profile[0] * gRobot.robot_dev[id].module.cart.prm.cart_rotate_sys_vel * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->prf_cart_rot.profile_acc = profile[1] * gRobot.robot_dev[id].module.cart.prm.cart_rotate_sys_acc * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->prf_cart_rot.profile_tacc = profile[2];
			new->prf_cart_rot.profile_dec = profile[3] * gRobot.robot_dev[id].module.cart.prm.cart_rotate_sys_dec * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->prf_cart_rot.profile_tdec = profile[4];
			new->prf_cart_rot.profile_sample_time = MOTION_MODULE_SAMPLE_TIME;

			//set the circle pose method,now just for test
			new->cartcircle_method = CARTCIRCLE_TYPE_AXIS_ANGLE;
			new->cartcircle_sync = CARTCIRCLE_TYPE_SYNC_TIME_SAME;
			//new->cartcircle_sync 	 = CARTCIRCLE_TYPE_SYNC_TIME_SCALE;

			break;
		}
		case TRAJECTORY_MODULE_MOTION_TYPE_NONE:
		default:
			break;
		}

		//-------------------------------------------------------------------------
		// BLEND
		new->blending.type = (Uint8)BlendDelayArc[0];
		new->delay_enable = (Uint8)BlendDelayArc[2];
		new->delay_time = (Uint32)BlendDelayArc[3];
		if (new->blending.type)
		{
			// Need Blending set Blending flag
			new->blending.is_need_blending = 1;
			new->delay_enable = 0;
		}
		else
		{
			//do not need blending
			new->blending.is_need_blending = 0;
		}
		//-------------------------------------------------------------------------
		//time percent parameter,do not used for auto mode
		new->blending.parameters = BlendDelayArc[1];
		if (new->blending.parameters > 1.0)
		{
			new->blending.parameters = 1.0; /* 100% */
		}
		else if (new->blending.parameters < 0.0)
		{
			new->blending.parameters = 0.0; /* 0% */
		}

		new->state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_USED;
		//--------------------------------------------------------------------------------
		/* point to next motion block */
		m_traj->motion_block_new_insert = get_next_motion_block(m_traj->motion_block_buffer, m_traj->motion_block_new_insert);

		//-----------------------------------------------------------------------------
		/* inform motion planning task to compute the profile of new motion*/
	//	if (gRobot.robot_dev[id].mode.mode == ROBOT_MODULE_SCRIPT_MODE || gRobot.robot_dev[id].mode.mode == ROBOT_MODULE_MACRO_PROGRAM_MODE)
		{
		//	rtems_semaphore_release(gRobot.robot_dev[id].mgr.profile_task_sem);
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

//----------------------------------------------------------------------------------------
int16 TrajectoryGenerator(trajectory_module* p_trajectory, cartesian_module* p_cart, joint_module* p_joint, robot_config_module* p_config, Uint8 id)
{
	motion_block* 			p_motion = p_trajectory->motion_block_execute;
	motion_block* 			p_motion_next = p_trajectory->motion_block_execute;
	motion_block* 			p_last_motion = p_trajectory->motion_block_execute;

	//-------------------------------------------------------------------
	double joint[8] = { 0,0,0,0,0,0,0,0 };
	double cart[6] = { 0,0,0,0,0,0 };

	double joint_next[8]= { 0,0,0,0,0,0,0,0 };
	double cart_next[6] = { 0,0,0,0,0,0 };

	double joint_vel[8]= { 0,0,0,0,0,0,0,0 };
	double cart_vel[6] = { 0,0,0,0,0,0 };

	double joint_vel_next[8]= { 0,0,0,0,0,0,0,0 };
	double cart_vel_next[6] = { 0,0,0,0,0,0 };

	//-------------------------------------------------------------------
	int32 blending_current_time = 0;
	Uint8 cart_id = 0;
	Uint8 joint_id = 0;
	Uint8 counter = 0;
	Uint8 counter1 = 0;
	Uint8 flag = 0;

	//-------------------------------------------------------------------
	double clock_time = 0.0;
	//-------------------------------------------------------------------
	double line_pos = 0.0;
	double line_vel = 0.0;
	double line_pos_next = 0.0;
	double line_vel_next = 0.0;

	//-------------------------------------------------------------------
	double circle_pos = 0.0;		//arc lenth
	double circle_vel = 0.0;		//tangent line vel
	double circle_pos_next = 0.0;		//arc lenth
	double circle_vel_next = 0.0;		//tangent line vel

	double circle_w = 0.0;		// angular velocity ---w
	double circle_angle_delta = 0.0;
	//-------------------------------------------------------------------
	//if the execute spd is faster than produce the motion block,then pos do not change ,but time increase
	//p_trajectory->abs_time ++;
	//-------------------------------------------------------------------
	if (p_motion->state < TRAJECTORY_MODULE_MOTION_BLOCK_STATE_RUNNING)
	{
		// if robot still,clear stop script flag to avoid the next block can't be inserted.
		if (p_config->stop_script_flag == 1)
		{
			p_config->stop_script_flag = 0;
		}

		// clear motion block num.
		//gRobot.robot_dev[id].mode.rmacro.block_nr = 0;
		//gRobot.robot_dev[id].status = ROBOT_DEV_STS_STOP;
		/* do not running */
		return 0;
	}

	//gRobot.robot_dev[id].status = ROBOT_DEV_STS_RUN;

	// if need delay before run
	if (/*gRobot.robot_dev[id].mode.mode != ROBOT_MODULE_SCRIPT_MODE ||*/ p_motion->blending.is_need_blending == 1)
	{
		p_motion->delay_enable = 0;
		p_motion->delay_time = 0;
	}
	if (p_motion->delay_enable == 1)
	{
		if (p_motion->delay_time > 0)
		{
			p_motion->delay_time--;
			return 0;
		}
		else
		{
			p_motion->delay_enable = 0;
		}
	}

	//location configuration changed now
	if ((p_config->locat_change_flag == 1) && (p_motion->motion_block_id == p_config->locat_change_block_id) && (p_config->locat_change_finish == 0))
	{
		p_config->prm.location_config = p_config->locat_tmp;
		p_config->locat_change_block_id = MOTION_MODULE_DEFAULT_MOTION_BLOCK_SIZE; //set this value to unreachable value
		p_config->locat_change_flag = 0;
		p_config->locat_change_finish = 0;
	}

	//-----------------------------------------------------------------
	// Running
	switch (p_motion->motion_type)
	{
	case TRAJECTORY_MODULE_MOTION_TYPE_P2P:
	{
		//----------------------------------------------------------
		//get next motion block
		p_motion_next = get_next_motion_block(p_trajectory->motion_block_buffer, p_motion_next);

		//if ((gRobot.robot_dev[id].mode.resume_flag == ROBOT_RESUME_REPEATE_MODE_PART1_BACK2START) && p_motion_next->blending.is_need_blending)
		{
		//	flag = 1;
		//	p_motion_next->blending.is_need_blending = 0;
		}

		if ((!p_trajectory->current_time) && (p_motion_next->blending.is_need_blending))
		{
			/* when start motion, check the next blending condition first */
			if (!p_motion_next->blending.is_ready_blending)
			{
				/* Wait to finish calculating the motion profile parameter and blending parameter*/
				return 0;
			}
		}
		//----------------------------------------------------------

		// update trajectory clock
		p_trajectory->current_time++;
		//p_trajectory->abs_time ++;

		//if the current need blending ,the before part has been execute at last block.
		if (p_motion->blending.is_need_blending)
		{
			/* start motion at the time of finishing blending.*/
			p_trajectory->current_time = p_motion->blending.blending_time_length + 1;
			p_motion->blending.is_need_blending = 0; /* no need now, for blending is finished! */
		}

		//-----------------------------------------------------------------
		for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
		{
			// Trajectory generated  ,every joint
			for (counter = 0; counter < 7; counter++)
			{
				if ((p_trajectory->current_time > p_motion->traj_joint[joint_id].time_series[counter]) && \
					(p_trajectory->current_time <= p_motion->traj_joint[joint_id].time_series[counter + 1]))
				{
					/* determine the time interval,and calculate the states in the time interval */
					clock_time = p_trajectory->current_time - p_motion->traj_joint[joint_id].time_series[counter];
					clock_time *= p_motion->prf_joint[joint_id].profile_sample_time;

					//calculate acc
					calculate_acceleration_from_profile(clock_time,
						p_motion->traj_joint[joint_id].jerk_series[counter],
						p_motion->traj_joint[joint_id].acc_series[counter],
						&p_motion->traj_joint[joint_id].acc);

					calculate_velocity_from_profile(clock_time,
						p_motion->traj_joint[joint_id].jerk_series[counter],
						p_motion->traj_joint[joint_id].acc_series[counter],
						p_motion->traj_joint[joint_id].vel_series[counter],
						&p_motion->traj_joint[joint_id].vel);

					calculate_position_from_profile(clock_time,
						p_motion->traj_joint[joint_id].jerk_series[counter],
						p_motion->traj_joint[joint_id].acc_series[counter],
						p_motion->traj_joint[joint_id].vel_series[counter],
						p_motion->traj_joint[joint_id].pos_series[counter],
						&p_motion->traj_joint[joint_id].pos);

					joint[joint_id] = p_motion->traj_joint[joint_id].pos;
					joint_vel[joint_id] = p_motion->traj_joint[joint_id].vel;

					// Break the FOR ,just do once every cycle
					break;
				}
			}
			//if next motion is also joint,so  blending can do in the joint directly
			if (p_motion_next->blending.is_need_blending)
			{
				//the next block need blending,so do the end blending ,which also is the next start blending

				blending_current_time = p_trajectory->current_time - p_motion_next->blending.blending_start_time;

				if ((blending_current_time > 0) && (p_motion_next->prfcoord_type == COORD_TYPE_JOINT))
				{
					for (counter = 0; counter < 7; counter++)
					{
						// if next block is type_joint ,blending is joint and joint's blending

						if ((blending_current_time > p_motion_next->traj_joint[joint_id].time_series[counter]) && \
							(blending_current_time <= p_motion_next->traj_joint[joint_id].time_series[counter + 1]))
						{
							/* determine the time interval,and calculate the states in the time interval */
							clock_time = blending_current_time - p_motion_next->traj_joint[joint_id].time_series[counter];
							clock_time *= p_motion_next->prf_joint[joint_id].profile_sample_time;

							calculate_acceleration_from_profile(clock_time,
								p_motion_next->traj_joint[joint_id].jerk_series[counter],
								p_motion_next->traj_joint[joint_id].acc_series[counter],
								&p_motion_next->traj_joint[joint_id].acc);
							calculate_velocity_from_profile(clock_time,
								p_motion_next->traj_joint[joint_id].jerk_series[counter],
								p_motion_next->traj_joint[joint_id].acc_series[counter],
								p_motion_next->traj_joint[joint_id].vel_series[counter],
								&p_motion_next->traj_joint[joint_id].vel);
							calculate_position_from_profile(clock_time,
								p_motion_next->traj_joint[joint_id].jerk_series[counter],
								p_motion_next->traj_joint[joint_id].acc_series[counter],
								p_motion_next->traj_joint[joint_id].vel_series[counter],
								p_motion_next->traj_joint[joint_id].pos_series[counter],
								&p_motion_next->traj_joint[joint_id].pos);

							//save the blending part in the joint_next and joint_vel_next
							joint_next[joint_id] = (*(p_motion_next->pose.joint_start_vector.point))[joint_id] + p_motion_next->traj_joint[joint_id].pos;
							joint_vel_next[joint_id] = p_motion_next->traj_joint[joint_id].vel;

							/* Break the FOR */
							break;
						}
					}
				}
				else
				{

				}
			}
		}
		//now we should think about the next part is cartesian type. because we can not calculate blending directly
		//we should change the cartesian type to joint to do blending
		if (p_motion_next->blending.is_need_blending)
		{
			//the next block need blending,so do the end blending ,which also is the next start blending
			blending_current_time = p_trajectory->current_time - p_motion_next->blending.blending_start_time;

			if ((blending_current_time > 0) && (p_motion_next->prfcoord_type == COORD_TYPE_CART))
			{
				//because the cartesian type include the pos(x,y,z) and posture(a,b,r).now we use fixed angle as input
				//pos trajectory
				for (counter = 0; counter < 7; counter++)
				{
					if ((blending_current_time > p_motion_next->trajectory.time_series[counter]) && \
						(blending_current_time <= p_motion_next->trajectory.time_series[counter + 1]))
					{
						clock_time = blending_current_time - p_motion_next->trajectory.time_series[counter];
						clock_time *= p_motion_next->profile.profile_sample_time;
						//-----------------position-----------------
						calculate_acceleration_from_profile(clock_time,
							p_motion_next->trajectory.jerk_series[counter],
							p_motion_next->trajectory.acc_series[counter],
							&p_motion_next->trajectory.acc);
						calculate_velocity_from_profile(clock_time,
							p_motion_next->trajectory.jerk_series[counter],
							p_motion_next->trajectory.acc_series[counter],
							p_motion_next->trajectory.vel_series[counter],
							&p_motion_next->trajectory.vel);
						calculate_position_from_profile(clock_time,
							p_motion_next->trajectory.jerk_series[counter],
							p_motion_next->trajectory.acc_series[counter],
							p_motion_next->trajectory.vel_series[counter],
							p_motion_next->trajectory.pos_series[counter],
							&p_motion_next->trajectory.pos);
						break;
					}
				}

				//posture trajectory ,this is the angle trajectory
				for (counter = 0; counter < 7; counter++)
				{
					if ((blending_current_time > p_motion_next->traj_cart_rot.time_series[counter]) && \
						(blending_current_time <= p_motion_next->traj_cart_rot.time_series[counter + 1]))
					{
						clock_time = blending_current_time - p_motion_next->traj_cart_rot.time_series[counter];
						clock_time *= p_motion_next->profile.profile_sample_time;
						//----------------posture-----------------------
						// when blending ,position and posture have the same time.
						// calculate pos
						calculate_position_from_profile(clock_time,
							p_motion_next->traj_cart_rot.jerk_series[counter],
							p_motion_next->traj_cart_rot.acc_series[counter],
							p_motion_next->traj_cart_rot.vel_series[counter],
							p_motion_next->traj_cart_rot.pos_series[counter],
							&p_motion_next->traj_cart_rot.pos);

						calculate_velocity_from_profile(clock_time,
							p_motion_next->traj_cart_rot.jerk_series[counter],
							p_motion_next->traj_cart_rot.acc_series[counter],
							p_motion_next->traj_cart_rot.vel_series[counter],
							&p_motion_next->traj_cart_rot.vel);
						break;
					}
				}

				//if the posture axis is existed
				if (p_motion_next->pose.axis_angle_flag == 1)
				{

					//angle-axis convert to rotate matrix
					matxx_axisAngl2Rotm(p_motion_next->traj_cart_rot.pos,		//angle in rad
						&p_motion_next->pose.axis_vector,						//axis
						&p_motion_next->pose.rotation_matxx);

					//---------------------------------------------------------------
					//get the rotation matrix according to the base,(A is according to the base ,R is according to the A)
					/*  B=R*A */
					matxx_multiply(&p_motion_next->pose.rotation_matxx, &p_motion_next->pose.rot_start, &p_motion_next->pose.rotation_matxx);

					//change the rotation matrix to the fix angle,because scara  can use this fix angle presentation,so these angle is the joint angle
					/* calculate   pitch, roll, yaw  -------a5,b4,r3*/

					//change_10_29
					/*
					cart_next[4] = atan2(-*(*(p_motion_next->pose.rotation_matxx.point + 0) + 2), pow(pow(*(*(p_motion_next->pose.\
					rotation_matxx.point + 0) + 0), 2) + pow(*(*(p_motion_next->pose.rotation_matxx.point + 0) + 1), 2), 0.5));

					if (fabs(cart_next[4] - MOTION_MODULE_CONSTANT_PI / 2) < 1E-10)
					{
					cart_next[5] = 0;
					cart_next[3] = atan2(*(*(p_motion_next->pose.rotation_matxx.point + 1) + 0), *(*(p_motion_next->pose.rotation_matxx.point + 1) + 1));
					}
					else if (fabs(cart_next[4] + MOTION_MODULE_CONSTANT_PI / 2) < 1E-10)
					{
					cart_next[5] = 0;
					cart_next[3] = -atan2(*(*(p_motion_next->pose.rotation_matxx.point + 1) + 0), *(*(p_motion_next->pose.rotation_matxx.point + 1) + 1));
					}
					else
					{
					cart_next[5] = atan2(*(*(p_motion_next->pose.rotation_matxx.point + 0) + 1) / cos(cart_next[4]), *(*(p_motion_next->pose.\
					rotation_matxx.point + 0) + 0) / cos(cart_next[4]));
					cart_next[3] = atan2(*(*(p_motion_next->pose.rotation_matxx.point + 1) + 2) / cos(cart_next[4]), *(*(p_motion_next->pose.\
					rotation_matxx.point + 2) + 2) / cos(cart_next[4]));
					}
					*/

					r2xyz(&p_motion_next->pose.rotation_matxx, &cart_next[3], 1);

					//---------------------------------------------------------------
					//decompose the angular velocity to base coordinate system

					//A* axis' = axis (axis' is according to the base ,axis is according to the A , A is rotation matrix of start point
					//axis' = (A)^-1 * axis , we want to use the axis which according to the base
					matxx_transpose(&p_motion_next->pose.rot_start, &p_motion_next->pose.rot_tmp);
					matxx_multiply(&p_motion_next->pose.rot_tmp, &p_motion_next->pose.axis_vector, &p_motion_next->pose.tmp31);

					// Wx  [ axis_x ]
					// Wy =[ axis_y ] * angular_vel
					// Wz  [ axis_z ]
					matxx_k_mult(p_motion_next->traj_cart_rot.vel, &p_motion_next->pose.tmp31);

					//change_10_29
					/*
					// now we can get angle speed mapping to the xyz which the axis is according to the base
					// *****************we should use this to calculate joint which mutiply inverse of jacobian********************
					// now we do not use this speed.

					w[0] = *(*(p_motion_next->pose.tmp31.point + 0) + 0);
					w[1] = *(*(p_motion_next->pose.tmp31.point + 0) + 1);
					w[2] = *(*(p_motion_next->pose.tmp31.point + 0) + 2);

					//the following maybe use to display some information about different presentation
					//we want the angle speed which is the fixed angle ,so follow the next formulation

					// d a /d t          [ 0    sr    cr    ]   [ Wx ]
					// d b /d t = 1/cb * [ 0    cscb  -srcb ] * [ Wy ]
					// d r /d t          [ cb   srsb  crsb  ]   [ Wz ]
					if (fabs(cart_next[4] - MOTION_MODULE_CONSTANT_PI / 2) < MOTION_MODULE_CONSTANT_MIN_POSITIVE)
					{
					//this function has error,we need fixed it
					cart_vel_next[4] = w[1] / cos(cart_next[3]);
					cart_vel_next[3] = -w[2] / sin(cart_next[3]);
					cart_vel_next[5] = cart_vel_next[3] - w[0];

					}
					else
					{
					cart_vel_next[5] = 1 / cos(cart_next[4])*(sin(cart_next[3])*w[1] + cos(cart_next[3])* w[2]);
					cart_vel_next[4] = 1 / cos(cart_next[4])*(cos(cart_next[3])*cos(cart_next[4])*w[1] - sin(cart_next[3])*cos(cart_next[4])*w[2]);
					cart_vel_next[3] = w[0] + sin(cart_next[3])*tan(cart_next[4])*w[1] + cos(cart_next[3])*tan(cart_next[4])*w[2];
					}

					cart_next[3] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
					cart_next[4] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
					cart_next[5] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
					cart_vel_next[3] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
					cart_vel_next[4] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
					cart_vel_next[5] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
					*/

					axisvel2dxyz(&p_motion_next->pose.tmp31, &cart_next[3], &cart_vel_next[3], 0);
				}
				else
				{
					//give the start posture
					cart_next[3] = p_motion_next->pose.pose_start[3];
					cart_next[4] = p_motion_next->pose.pose_start[4];
					cart_next[5] = p_motion_next->pose.pose_start[5];
					cart_vel_next[3] = 0;
					cart_vel_next[4] = 0;
					cart_vel_next[5] = 0;

				}
				//according to the different type to do the position and velocity calculation
				if (p_motion_next->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_LIN)
				{
					line_pos_next = p_motion_next->trajectory.pos;
					line_vel_next = p_motion_next->trajectory.vel;

					//line pos decouple, dimension is 3
					for (cart_id = 0; cart_id < p_config->cart_trans_dim; cart_id++)
					{
						cart_next[cart_id] = p_motion_next->pose.pose_start[cart_id] + line_pos_next * p_motion_next->pose.pose_unit_vector[cart_id];
						cart_vel_next[cart_id] = line_vel_next*p_motion_next->pose.pose_unit_vector[cart_id];
					}
					/* calculate the joint */
					//gRobot.robot_dev[id].module.kinematics.pfFK(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_next, joint_next);
					//gRobot.robot_dev[id].module.kinematics.pfIJ(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_vel_next, joint_vel_next);
				}
				if (p_motion_next->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_CIRCL)
				{
					circle_pos_next = p_motion_next->trajectory.pos;
					circle_vel_next = p_motion_next->trajectory.vel;
					/* get position command
					* P0 + [RotM(s,w) - I] * CP0 ,where : s is angle,w is axis
					* */
					// get arc length corresponding angle in Rad
					circle_angle_delta = circle_pos_next * p_motion_next->pose.circle_radius_inv;

					//angle-axis convert to rotate matrix
					matxx_axisAngl2Rotm(-circle_angle_delta,		//angle in rad
						&p_motion_next->pose.unit_vector,			//axis
						&p_motion_next->pose.rotation_matxx);

					// Let XYZ = P0
					matxx_assign(&p_motion_next->pose.cartesian_xyz,
						p_motion_next->pose.pose_start,
						3);
					/* RotM(s,w) - I */
					for (counter = 0; counter < 3; counter++)
					{
						(*(p_motion_next->pose.rotation_matxx.point + counter))[counter] -= 1.0;
					}

					/* do rotation */
					//cartesian_xyz = center_vector*rotation_matxx + cartesian_xyz(this is start pos)
					matxx_mat_vec_opt(&p_motion_next->pose.center_vector,
						&p_motion_next->pose.rotation_matxx,
						1.0,
						1.0,
						&p_motion_next->pose.cartesian_xyz);
					/* get velocity command
					* dP(s(t))/dt = dP(s)/ds * ds(t)/dt   ---->s(t) is angle
					* where ds(t)/dt  = w
					* dP(s)/ds = dRotM(s,w)/ds * CP0, other part is constant ,so is zero
					*
					* and vel = dRotM(s,w)/ds * CP0 * w
					* */
					//v = w*r    w = v/r
					circle_w = circle_vel_next* p_motion_next->pose.circle_radius_inv;

					matxx_axisAngle2protm(circle_angle_delta, 1,			//angle, w is mutiply in the later,so this use 1
						&p_motion_next->pose.unit_vector,				//axis
						&p_motion_next->pose.rotation_matxx); 			//rotate matrix differential

					matxx_copy(&p_motion_next->pose.rotation_matxx, &p_motion_next->pose.rot_tmp);
					matxx_transpose(&p_motion_next->pose.rot_tmp, &p_motion_next->pose.rotation_matxx);

					matxx_mat_vec_opt(&p_motion_next->pose.center_vector,	//cp0
						&p_motion_next->pose.rotation_matxx,   				//rotate matrix differential
						circle_w,						    				//w
						0.0,
						&p_motion_next->pose.cartesian_xyz_vel);

					//circle pos ,dimension is 3
					for (cart_id = 0; cart_id < p_config->cart_trans_dim; cart_id++)
					{
						cart_next[cart_id] = *(*(p_motion_next->pose.cartesian_xyz.point + 0) + cart_id);
						cart_vel_next[cart_id] = *(*(p_motion_next->pose.cartesian_xyz_vel.point + 0) + cart_id);
					}
					/* calculate the joint */
					//gRobot.robot_dev[id].module.kinematics.pfFK(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_next, joint_next);
					//gRobot.robot_dev[id].module.kinematics.pfIJ(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_vel_next, joint_vel_next);
				}

				//now we get the pos cart and posture cart part ,then we can do the inverse to get joint next
				gRobot.robot_dev[id].module.kinematics.pfIK(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_next, joint_next, 1);
			}
		}

		//calculate the current pos and vel,  this is joint condition
		for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
		{
			//Pt = P0 + delta;
			//do not have unit vector concept
			//joint_pos is used for real time pos matrix

			//calculate the joint_pos for every joint
			(*(p_motion->pose.joint_pos.point))[joint_id] = (*(p_motion->pose.joint_start_vector.point))[joint_id] + p_motion->traj_joint[joint_id].pos;
			//-----------------------------------------------------------------
			// get the command and set to the joint module
			matxx_get_element(&p_motion->pose.joint_pos, joint_id, 0, &(p_joint[joint_id].joint_pos_cmd));

			//real time vel cmd,also set to the joint module
			p_joint[joint_id].joint_vel_cmd = p_motion->traj_joint[joint_id].vel;

			//real time pos cmd, joint ,do not add blending part
			joint[joint_id] = p_joint[joint_id].joint_pos_cmd;
			joint_vel[joint_id] = p_joint[joint_id].joint_vel_cmd;
		}
		//-----------------------------------------------------------------
		//add blending part if has this part
		if (p_motion_next->blending.is_need_blending && (blending_current_time > 0))
		{
			if (p_motion_next->prfcoord_type == COORD_TYPE_CART)
			{
				for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
				{
					//joint_pos_cmd is tmp variable
					p_joint[joint_id].joint_pos_cmd = joint_next[joint_id];
				}
				//do the inverse jacobian to get the joint vel next
				gRobot.robot_dev[id].module.kinematics.pfIJ(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_vel_next, joint_vel_next);

			}

			for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
			{
				//next part minis the current end point
				p_joint[joint_id].joint_pos_cmd = joint[joint_id] + (joint_next[joint_id] - (*p_motion->pose.joint_end_vector.point)[joint_id]);
				p_joint[joint_id].joint_vel_cmd = joint_vel[joint_id] + joint_vel_next[joint_id];

				joint[joint_id] = p_joint[joint_id].joint_pos_cmd;
				joint_vel[joint_id] = p_joint[joint_id].joint_vel_cmd;
			}
		}
		//now we get the cart pos and vel after the blending
		gRobot.robot_dev[id].module.kinematics.pfFK(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart, joint);
		gRobot.robot_dev[id].module.kinematics.pfFJ(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_vel, joint_vel);

		//set the cart value to the cart module
		for (cart_id = 0; cart_id < p_config->cart_dim; cart_id++)
		{
			p_cart->cart_pos_cmd[cart_id] = cart[cart_id];
			p_cart->cart_vel_cmd[cart_id] = cart_vel[cart_id];
		}

		//if ((gRobot.robot_dev[id].mode.resume_flag == ROBOT_RESUME_REPEATE_MODE_PART1_BACK2START) && flag)
		{
		//	p_motion_next->blending.is_need_blending = 1;
		}

		//-----------------------------------------------------------------
		// finish,because every joint has same time
		if (p_trajectory->current_time >= p_motion->traj_joint[1].time_series[7])
		{
			//if ((gRobot.robot_dev[id].mode.mode == ROBOT_MODULE_SCRIPT_MODE) || gRobot.robot_dev[id].mode.mode == ROBOT_MODULE_MACRO_PROGRAM_MODE)
			//{
				/* insert new motion block at current block
				if ((gRobot.robot_dev[id].mode.resume_flag == ROBOT_RESUME_REPEATE_MODE_PART1_BACK2START))
				{
					// clear resume flag
					gRobot.robot_dev[id].mode.resume_flag = ROBOT_RESUME_REPEATE_MODE_PART2_REPEAT;

					// clear the clock time
					p_trajectory->current_time = 0;

					// resume the motion type
					p_motion->motion_type = p_motion->temp_type;


					if (p_motion->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_P2P)
					{
						matxx_copy(&p_motion->pose.joint_end_vector, &p_motion->pose.joint_start_vector);
						matxx_copy(&p_motion->pose.joint_temp_vector, &p_motion->pose.joint_end_vector);
					}

					// set states
					p_motion->state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_USED;
					p_trajectory->motion_block_calculate = p_motion;

					rtems_semaphore_release(gRobot.robot_dev[id].mgr.profile_task_sem);

				}
				else
				*/
				{
					p_motion->state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_IDL;

					// clear states
					p_trajectory->current_time = 0;
					p_trajectory->motion_block_execute = p_motion_next;


					/*if (gRobot.robot_dev[id].mode.resume_flag == ROBOT_RESUME_REPEATE_MODE_PART2_REPEAT)
					{
						//reset resume flag
						gRobot.robot_dev[id].mode.resume_flag = ROBOT_RESUME_REPEATE_MODE_PART0_FINISH;
						// change the  calculate pointer to correct block
						p_trajectory->motion_block_calculate = p_motion;

					}*/
					/* inform to create new motion block ,then you can create new motion block*/
					//rtems_semaphore_release(gRobot.robot_dev[id].mgr.script_task_sem);
					//rtems_semaphore_release(gRobot.robot_dev[id].mgr.block_finish_sem);
					//if (gRobot.robot_dev[id].mode.mode == ROBOT_MODULE_MACRO_PROGRAM_MODE) {
					//	gRobot.robot_dev[id].mode.rmacro.block_nr--;
					//}

				}
			}
			/*else if (gRobot.robot_dev[id].mode.mode == ROBOT_MODULE_JOG_MODE)
			{
				// clear states
				p_trajectory->current_time = 0;
				p_trajectory->motion_block_execute = p_motion_next;

				p_trajectory->motion_block_buffer[0].state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_IDL;
				p_trajectory->motion_block_buffer[1].state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_IDL;
				p_trajectory->motion_block_buffer[2].state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_IDL;
				gRobot.robot_dev[id].mode.rjog.block_num = 0;
				gRobot.robot_dev[id].mode.rjog.stop_over = 1;
				gRobot.robot_dev[id].mode.rjog.run_key = 0;
				gRobot.robot_dev[id].mode.rjog.jog_dir = gRobot.robot_dev[id].mode.rjog.jog_dir_tmp;

			}*/
		}

		break;
	case TRAJECTORY_MODULE_MOTION_TYPE_LIN: // line motion in Cartesian
	{
		//-----------------------------------------------------------------
		p_motion_next = get_next_motion_block(p_trajectory->motion_block_buffer, p_motion_next);

		if ((!p_trajectory->current_time) && (p_motion_next->blending.is_need_blending))
		{
			/* when start motion, check the next blending condition first */
			if (!p_motion_next->blending.is_ready_blending)
			{
				/* Wait to finish calculating the motion profile parameter and blending parameter*/
				return 0;
			}
		}
		/* update clock */
		p_trajectory->current_time++;
		//p_trajectory->abs_time ++;

		//if the current need blending ,the before part has been execute at last block.
		//so start from the blending finish clock
		if (p_motion->blending.is_need_blending)
		{
			/* start motion at the time of finishing blending.*/
			p_trajectory->current_time = p_motion->blending.blending_time_length + 1;
			p_motion->blending.is_need_blending = 0; /* no need now, for blending is finished! */
		}

		// Trajectory generated by a polynomial.
		//this part is do not need blending ,which in the center of the block ,maybe start and end all need blending

		if (p_motion->cartline_sync == CARTLINE_TYPE_SYNC_TIME_SCALE)
		{
			//if time scale mode ,pos and posture every sector time is different,but the arrived time is same .so we need seperate them
			//pos
			for (counter = 0; counter < 7; counter++)
			{
				if ((p_trajectory->current_time > p_motion->trajectory.time_series[counter]) && \
					(p_trajectory->current_time <= p_motion->trajectory.time_series[counter + 1]))
				{
					/* determine the time interval,and calculate the states in the time interval */
					clock_time = p_trajectory->current_time - p_motion->trajectory.time_series[counter];
					clock_time *= p_motion->profile.profile_sample_time;

					calculate_acceleration_from_profile(clock_time,
						p_motion->trajectory.jerk_series[counter],
						p_motion->trajectory.acc_series[counter],
						&p_motion->trajectory.acc);
					calculate_velocity_from_profile(clock_time,
						p_motion->trajectory.jerk_series[counter],
						p_motion->trajectory.acc_series[counter],
						p_motion->trajectory.vel_series[counter],
						&p_motion->trajectory.vel);
					calculate_position_from_profile(clock_time,
						p_motion->trajectory.jerk_series[counter],
						p_motion->trajectory.acc_series[counter],
						p_motion->trajectory.vel_series[counter],
						p_motion->trajectory.pos_series[counter],
						&p_motion->trajectory.pos);
					line_pos = p_motion->trajectory.pos;
					line_vel = p_motion->trajectory.vel;
					break;
				}
			}
			//posture
			for (counter1 = 0; counter1 < 7; counter1++)
			{
				if ((p_trajectory->current_time > p_motion->traj_cart_rot.time_series[counter1]) && \
					(p_trajectory->current_time <= p_motion->traj_cart_rot.time_series[counter1 + 1]))
				{
					/* determine the time interval,and calculate the states in the time interval */
					clock_time = p_trajectory->current_time - p_motion->traj_cart_rot.time_series[counter1];
					clock_time *= p_motion->prf_cart_rot.profile_sample_time;

					if (p_motion->pose.axis_angle_flag == 1)
					{
						//calculate rotate angle pos
						calculate_position_from_profile(clock_time,
							p_motion->traj_cart_rot.jerk_series[counter1],
							p_motion->traj_cart_rot.acc_series[counter1],
							p_motion->traj_cart_rot.vel_series[counter1],
							p_motion->traj_cart_rot.pos_series[counter1],
							&p_motion->traj_cart_rot.pos);
						//calculate rotate angle spd
						calculate_velocity_from_profile(clock_time,
							p_motion->traj_cart_rot.jerk_series[counter1],
							p_motion->traj_cart_rot.acc_series[counter1],
							p_motion->traj_cart_rot.vel_series[counter1],
							&p_motion->traj_cart_rot.vel);
					}
					else
					{
					}
					break;
				}
			}
		}
		else
		{
			//if time same mode ,pos and posture every sector time is same ,so we just use same time sector to do pos's trajectory and posture's trajectory
			for (counter = 0; counter < 7; counter++)
			{
				if ((p_trajectory->current_time > p_motion->trajectory.time_series[counter]) && \
					(p_trajectory->current_time <= p_motion->trajectory.time_series[counter + 1]))
				{
					/* determine the time interval,and calculate the states in the time interval */
					clock_time = p_trajectory->current_time - p_motion->trajectory.time_series[counter];
					clock_time *= p_motion->profile.profile_sample_time;

					calculate_acceleration_from_profile(clock_time,
						p_motion->trajectory.jerk_series[counter],
						p_motion->trajectory.acc_series[counter],
						&p_motion->trajectory.acc);
					calculate_velocity_from_profile(clock_time,
						p_motion->trajectory.jerk_series[counter],
						p_motion->trajectory.acc_series[counter],
						p_motion->trajectory.vel_series[counter],
						&p_motion->trajectory.vel);
					calculate_position_from_profile(clock_time,
						p_motion->trajectory.jerk_series[counter],
						p_motion->trajectory.acc_series[counter],
						p_motion->trajectory.vel_series[counter],
						p_motion->trajectory.pos_series[counter],
						&p_motion->trajectory.pos);
					line_pos = p_motion->trajectory.pos;
					line_vel = p_motion->trajectory.vel;

					if (p_motion->pose.axis_angle_flag == 1)
					{
						//calculate rotate angle pos
						calculate_position_from_profile(clock_time,
							p_motion->traj_cart_rot.jerk_series[counter],
							p_motion->traj_cart_rot.acc_series[counter],
							p_motion->traj_cart_rot.vel_series[counter],
							p_motion->traj_cart_rot.pos_series[counter],
							&p_motion->traj_cart_rot.pos);
						//calculate rotate angle spd
						calculate_velocity_from_profile(clock_time,
							p_motion->traj_cart_rot.jerk_series[counter],
							p_motion->traj_cart_rot.acc_series[counter],
							p_motion->traj_cart_rot.vel_series[counter],
							&p_motion->traj_cart_rot.vel);
					}
					else
					{
					}

					break;
				}
			}
		}
		//current posture calculate
		if (p_motion->pose.axis_angle_flag == 1)
		{
			//angle-axis convert to rotate matrix
			matxx_axisAngl2Rotm(p_motion->traj_cart_rot.pos,		//angle in rad
				&p_motion->pose.axis_vector,						//axis
				&p_motion->pose.rotation_matxx);

			/*  B=R*A */
			matxx_multiply(&p_motion->pose.rotation_matxx, &p_motion->pose.rot_start, &p_motion->pose.rotation_matxx);

			//change_10_29
			/*
			// calculate   pitch, roll, yaw
			cart[4] = atan2(-*(*(p_motion->pose.rotation_matxx.point + 0) + 2), pow(pow(*(*(p_motion->pose.\
			rotation_matxx.point + 0) + 0), 2) + pow(*(*(p_motion->pose.rotation_matxx.point + 0) + 1), 2), 0.5));

			if (fabs(cart[4] - MOTION_MODULE_CONSTANT_PI / 2)<1E-10)
			{
			cart[5] = 0;
			cart[3] = atan2(*(*(p_motion->pose.rotation_matxx.point + 1) + 0), *(*(p_motion->pose.rotation_matxx.point + 1) + 1));
			}
			else if (fabs(cart[4] + MOTION_MODULE_CONSTANT_PI / 2)<1E-10)
			{
			cart[5] = 0;
			cart[3] = -atan2(*(*(p_motion->pose.rotation_matxx.point + 1) + 0), *(*(p_motion->pose.rotation_matxx.point + 1) + 1));
			}
			else
			{
			cart[5] = atan2(*(*(p_motion->pose.rotation_matxx.point + 0) + 1) / cos(cart[4]), *(*(p_motion->pose.\
			rotation_matxx.point + 0) + 0) / cos(cart[4]));
			cart[3] = atan2(*(*(p_motion->pose.rotation_matxx.point + 1) + 2) / cos(cart[4]), *(*(p_motion->pose.\
			rotation_matxx.point + 2) + 2) / cos(cart[4]));
			}
			*/
			r2xyz(&p_motion->pose.rotation_matxx, &cart[3], 1);
			//-------------------------------------------------------------------------------------------
			//decouple axis to xyz to calculate the vel
			matxx_transpose(&p_motion->pose.rot_start, &p_motion->pose.rot_tmp);

			//convert the axis to the base
			matxx_multiply(&p_motion->pose.rot_tmp, &p_motion->pose.axis_vector, &p_motion->pose.tmp31);

			// vel map to the axis
			matxx_k_mult(p_motion->traj_cart_rot.vel, &p_motion->pose.tmp31);

			//change_10_29
			/*

			w[0] = *(*(p_motion->pose.tmp31.point + 0) + 0);
			w[1] = *(*(p_motion->pose.tmp31.point + 0) + 1);
			w[2] = *(*(p_motion->pose.tmp31.point + 0) + 2);

			//the following maybe use to display some information about different presentation
			//we want the angle speed which is the fixed angle ,so follow the next formulation

			// d a /d t          [ 0    sr    cr    ]   [ Wx ]
			// d b /d t = 1/cb * [ 0    cscb  -srcb ] * [ Wy ]
			// d r /d t          [ cb   srsb  crsb  ]   [ Wz ]
			if (fabs(cart[4] - MOTION_MODULE_CONSTANT_PI / 2) < MOTION_MODULE_CONSTANT_MIN_POSITIVE)
			{
			//this function has error,we need fixed it
			cart_vel[4] = w[1] / cos(cart[3]);
			cart_vel[3] = -w[2] / sin(cart[3]);
			cart_vel[5] = cart_vel[3] - w[0];

			}
			else
			{
			cart_vel[5] = 1 / cos(cart[4])*(sin(cart[3])*w[1] + cos(cart[3])* w[2]);
			cart_vel[4] = 1 / cos(cart[4])*(cos(cart[3])*cos(cart[4])*w[1] - sin(cart[3])*cos(cart[4])*w[2]);
			cart_vel[3] = w[0] + sin(cart[3])*tan(cart[4])*w[1] + cos(cart[3])*tan(cart[4])*w[2];
			}

			cart[3] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
			cart[4] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
			cart[5] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
			cart_vel[3] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
			cart_vel[4] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
			cart_vel[5] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
			*/

			axisvel2dxyz(&p_motion->pose.tmp31, &cart[3], &cart_vel[3], 0);

		}
		else
		{
			cart[3] = p_motion->pose.pose_start[3];
			cart[4] = p_motion->pose.pose_start[4];
			cart[5] = p_motion->pose.pose_start[5];
			cart_vel[3] = 0;
			cart_vel[4] = 0;
			cart_vel[5] = 0;

		}

		//the next block need blending,so do the end blending ,which also is the next start blending
		if (p_motion_next->blending.is_need_blending)
		{
			blending_current_time = p_trajectory->current_time - p_motion_next->blending.blending_start_time;

			if (blending_current_time > 0)
			{
				//pos part
				//if the next motion is cart type
				if (p_motion_next->prfcoord_type == COORD_TYPE_CART)
				{
					for (counter = 0; counter < 7; counter++)
					{
						if ((blending_current_time > p_motion_next->trajectory.time_series[counter]) && \
							(blending_current_time <= p_motion_next->trajectory.time_series[counter + 1]))
						{
							/* determine the time interval,and calculate the states in the time interval */
							clock_time = blending_current_time - p_motion_next->trajectory.time_series[counter];
							clock_time *= p_motion_next->profile.profile_sample_time;

							calculate_acceleration_from_profile(clock_time,
								p_motion_next->trajectory.jerk_series[counter],
								p_motion_next->trajectory.acc_series[counter],
								&p_motion_next->trajectory.acc);

							calculate_velocity_from_profile(clock_time,
								p_motion_next->trajectory.jerk_series[counter],
								p_motion_next->trajectory.acc_series[counter],
								p_motion_next->trajectory.vel_series[counter],
								&p_motion_next->trajectory.vel);

							calculate_position_from_profile(clock_time,
								p_motion_next->trajectory.jerk_series[counter],
								p_motion_next->trajectory.acc_series[counter],
								p_motion_next->trajectory.vel_series[counter],
								p_motion_next->trajectory.pos_series[counter],
								&p_motion_next->trajectory.pos);

							if (p_motion_next->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_LIN)
							{
								line_pos_next = p_motion_next->trajectory.pos;
								line_vel_next = p_motion_next->trajectory.vel;
								for (cart_id = 0; cart_id < p_config->cart_trans_dim; cart_id++)
								{
									cart_next[cart_id] = p_motion_next->pose.pose_start[cart_id] + line_pos_next * p_motion_next->pose.pose_unit_vector[cart_id];
									cart_vel_next[cart_id] = line_vel_next * p_motion_next->pose.pose_unit_vector[cart_id];
								}
							}
							else if (p_motion_next->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_CIRCL)
							{
								circle_pos_next = p_motion_next->trajectory.pos;
								circle_vel_next = p_motion_next->trajectory.vel;
								/* get position command
								* P0 + [RotM(s,w) - I] * CP0 ,where : s is angle,w is axis
								* */
								// get arc length corresponding angle in Rad
								circle_angle_delta = circle_pos_next * p_motion_next->pose.circle_radius_inv;

								//angle-axis convert to rotate matrix
								matxx_axisAngl2Rotm(-circle_angle_delta,		//angle in rad
									&p_motion_next->pose.unit_vector,		//axis
									&p_motion_next->pose.rotation_matxx);

								// Let XYZ = P0
								matxx_assign(&p_motion_next->pose.cartesian_xyz,
									p_motion_next->pose.pose_start,
									3);
								/* RotM(s,w) - I */
								for (counter = 0; counter < 3; counter++)
								{
									(*(p_motion_next->pose.rotation_matxx.point + counter))[counter] -= 1.0;
								}

								/* do rotation */
								//cartesian_xyz = center_vector*rotation_matxx + cartesian_xyz(this is start pos)
								matxx_mat_vec_opt(&p_motion_next->pose.center_vector,
									&p_motion_next->pose.rotation_matxx,
									1.0,
									1.0,
									&p_motion_next->pose.cartesian_xyz);
								/* get velocity command
								* dP(s(t))/dt = dP(s)/ds * ds(t)/dt   ---->s(t) is angle
								* where ds(t)/dt  = w
								* dP(s)/ds = dRotM(s,w)/ds * CP0, other part is constant ,so is zero
								*
								* and vel = dRotM(s,w)/ds * CP0 * w
								* */
								//v = w*r    w = v/r
								circle_w = circle_vel_next * p_motion_next->pose.circle_radius_inv;
								matxx_axisAngle2protm(circle_angle_delta, 1,			//angle
									&p_motion_next->pose.unit_vector,	//axis
									&p_motion_next->pose.rotation_matxx); //rotate matrix differential

								matxx_copy(&p_motion_next->pose.rotation_matxx, &p_motion_next->pose.rot_tmp);
								matxx_transpose(&p_motion_next->pose.rot_tmp, &p_motion_next->pose.rotation_matxx);

								matxx_mat_vec_opt(&p_motion_next->pose.center_vector,	//cp0
									&p_motion_next->pose.rotation_matxx,   //rotate matrix differential
									circle_w,						    //w
									0.0,
									&p_motion_next->pose.cartesian_xyz_vel);
								for (cart_id = 0; cart_id < p_config->cart_trans_dim; cart_id++)
								{
									cart_next[cart_id] = *(*(p_motion_next->pose.cartesian_xyz.point + 0) + cart_id);
									cart_vel_next[cart_id] = *(*(p_motion_next->pose.cartesian_xyz_vel.point + 0) + cart_id);
								}
							}
							else
							{
								return 0;
							}
							break;
						}
					}

					//calculate the posture part
					// when blending ,position and posture have the same time.
					for (counter = 0; counter < 7; counter++)
					{
						if ((blending_current_time > p_motion_next->traj_cart_rot.time_series[counter]) && \
							(blending_current_time <= p_motion_next->traj_cart_rot.time_series[counter + 1]))
						{
							/* determine the time interval,and calculate the states in the time interval */
							clock_time = blending_current_time - p_motion_next->traj_cart_rot.time_series[counter];
							clock_time *= p_motion_next->profile.profile_sample_time;
							//calculate pos
							calculate_position_from_profile(clock_time,
								p_motion_next->traj_cart_rot.jerk_series[counter],
								p_motion_next->traj_cart_rot.acc_series[counter],
								p_motion_next->traj_cart_rot.vel_series[counter],
								p_motion_next->traj_cart_rot.pos_series[counter],
								&p_motion_next->traj_cart_rot.pos);

							calculate_velocity_from_profile(clock_time,
								p_motion_next->traj_cart_rot.jerk_series[counter],
								p_motion_next->traj_cart_rot.acc_series[counter],
								p_motion_next->traj_cart_rot.vel_series[counter],
								&p_motion_next->traj_cart_rot.vel);
							if (p_motion_next->pose.axis_angle_flag == 1)
							{
								//angle-axis convert to rotate matrix
								matxx_axisAngl2Rotm(p_motion_next->traj_cart_rot.pos,		//angle in rad
									&p_motion_next->pose.axis_vector,		//axis
									&p_motion_next->pose.rotation_matxx);
								/*  B=R*A */
								matxx_multiply(&p_motion_next->pose.rotation_matxx, &p_motion_next->pose.rot_start, &p_motion_next->pose.rotation_matxx);

								//change_10_29
								/*
								// calculate   pitch, roll, yaw
								cart_next[4] = atan2(-*(*(p_motion_next->pose.rotation_matxx.point + 0) + 2), pow(pow(*(*(p_motion_next->pose.\
								rotation_matxx.point + 0) + 0), 2) + pow(*(*(p_motion_next->pose.rotation_matxx.point + 0) + 1), 2), 0.5));

								if (fabs(cart_next[4] - MOTION_MODULE_CONSTANT_PI / 2) < 1E-10)
								{
								cart_next[5] = 0;
								cart_next[3] = atan2(*(*(p_motion_next->pose.rotation_matxx.point + 1) + 0), *(*(p_motion_next->pose.rotation_matxx.point + 1) + 1));
								}
								else if (fabs(cart_next[4] + MOTION_MODULE_CONSTANT_PI / 2) < 1E-10)
								{
								cart_next[5] = 0;
								cart_next[3] = -atan2(*(*(p_motion_next->pose.rotation_matxx.point + 1) + 0), *(*(p_motion_next->pose.rotation_matxx.point + 1) + 1));
								}
								else
								{
								cart_next[5] = atan2(*(*(p_motion_next->pose.rotation_matxx.point + 0) + 1) / cos(cart_next[4]), *(*(p_motion_next->pose.\
								rotation_matxx.point + 0) + 0) / cos(cart_next[4]));
								cart_next[3] = atan2(*(*(p_motion_next->pose.rotation_matxx.point + 1) + 2) / cos(cart_next[4]), *(*(p_motion_next->pose.\
								rotation_matxx.point + 2) + 2) / cos(cart_next[4]));
								}
								*/

								r2xyz(&p_motion_next->pose.rotation_matxx, &cart_next[3], 1);

								//decompose the angular velocity to basical coordinate system
								matxx_transpose(&p_motion_next->pose.rot_start, &p_motion_next->pose.rot_tmp);
								matxx_multiply(&p_motion_next->pose.rot_tmp, &p_motion_next->pose.axis_vector, &p_motion_next->pose.tmp31);
								// Wx  [ axis_x ]
								// Wy =[ axis_y ] * angular_vel
								// Wz  [ axis_z ]
								matxx_k_mult(p_motion_next->traj_cart_rot.vel, &p_motion_next->pose.tmp31);

								//change_10_29
								/*
								w[0] = *(*(p_motion_next->pose.tmp31.point + 0) + 0);
								w[1] = *(*(p_motion_next->pose.tmp31.point + 0) + 1);
								w[2] = *(*(p_motion_next->pose.tmp31.point + 0) + 2);

								// d a /d t          [ 0    sr    cr    ]   [ Wx ]
								// d b /d t = 1/cb * [ 0    cscb  -srcb ] * [ Wy ]
								// d r /d t          [ cb   srsb  crsb  ]   [ Wz ]
								if (fabs(cart_next[4] - MOTION_MODULE_CONSTANT_PI / 2) < MOTION_MODULE_CONSTANT_MIN_POSITIVE)
								{
								cart_vel_next[4] = w[1] / cos(cart_next[3]);
								cart_vel_next[3] = -w[2] / sin(cart_next[3]);
								cart_vel_next[5] = cart_vel_next[3] - w[0];

								}
								else
								{
								cart_vel_next[5] = 1 / cos(cart_next[4])*(sin(cart_next[3])*w[1] + cos(cart_next[3])* w[2]);
								cart_vel_next[4] = 1 / cos(cart_next[4])*(cos(cart_next[3])*cos(cart_next[4])*w[1] - sin(cart_next[3])*cos(cart_next[4])*w[2]);
								cart_vel_next[3] = w[0] + sin(cart_next[3])*tan(cart_next[4])*w[1] + cos(cart_next[3])*tan(cart_next[4])*w[2];
								}

								cart_next[3] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
								cart_next[4] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
								cart_next[5] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
								cart_vel_next[3] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
								cart_vel_next[4] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
								cart_vel_next[5] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
								*/
								axisvel2dxyz(&p_motion_next->pose.tmp31, &cart_next[3], &cart_vel_next[3], 0);

							}
							else
							{
								cart_next[3] = p_motion_next->pose.pose_start[3];
								cart_next[4] = p_motion_next->pose.pose_start[4];
								cart_next[5] = p_motion_next->pose.pose_start[5];
								cart_vel_next[3] = 0;
								cart_vel_next[4] = 0;
								cart_vel_next[5] = 0;
							}

							/* Break the FOR */
							break;
						}
					}
				}

				// blending in joint
				else
				{
					for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
					{
						for (counter = 0; counter < 8; counter++)
						{
							if ((blending_current_time > p_motion_next->traj_joint[joint_id].time_series[counter]) && \
								(blending_current_time <= p_motion_next->traj_joint[joint_id].time_series[counter + 1]))
							{
								/* determine the time interval,and calculate the states in the time interval */
								clock_time = blending_current_time - p_motion_next->traj_joint[joint_id].time_series[counter];
								clock_time *= p_motion_next->prf_joint[joint_id].profile_sample_time;

								calculate_acceleration_from_profile(clock_time,
									p_motion_next->traj_joint[joint_id].jerk_series[counter],
									p_motion_next->traj_joint[joint_id].acc_series[counter],
									&p_motion_next->traj_joint[joint_id].acc);
								calculate_velocity_from_profile(clock_time,
									p_motion_next->traj_joint[joint_id].jerk_series[counter],
									p_motion_next->traj_joint[joint_id].acc_series[counter],
									p_motion_next->traj_joint[joint_id].vel_series[counter],
									&p_motion_next->traj_joint[joint_id].vel);
								calculate_position_from_profile(clock_time,
									p_motion_next->traj_joint[joint_id].jerk_series[counter],
									p_motion_next->traj_joint[joint_id].acc_series[counter],
									p_motion_next->traj_joint[joint_id].vel_series[counter],
									p_motion_next->traj_joint[joint_id].pos_series[counter],
									&p_motion_next->traj_joint[joint_id].pos);
								joint_next[joint_id] = (*(p_motion_next->pose.joint_start_vector.point))[joint_id] + p_motion_next->traj_joint[joint_id].pos;
								joint_vel_next[joint_id] = p_motion_next->traj_joint[joint_id].vel;
								break;
							}
						}
					}
				}
			}
		}
		//calculate the current cart pos and posture to cart
		for (cart_id = 0; cart_id < p_config->cart_dim; cart_id++)
		{
			if (cart_id < ROBOT_CONFIG_MODULE_CARTESIAN_TRANS_3_DIM)
			{
				//pos
				p_cart->cart_vel_cmd[cart_id] = line_vel * p_motion->pose.pose_unit_vector[cart_id];
				p_cart->cart_pos_cmd[cart_id] = line_pos * p_motion->pose.pose_unit_vector[cart_id];

				/* get the current pos command */
				p_cart->cart_pos_cmd[cart_id] += p_motion->pose.pose_start[cart_id];
			}
			else
			{
				//posture
				if (p_motion->pose.axis_angle_flag == 1)
				{
					//just for scara,change rad to degree
					p_cart->cart_vel_cmd[cart_id] = cart_vel[cart_id];
					p_cart->cart_pos_cmd[cart_id] = cart[cart_id];
				}
				else
				{
					p_cart->cart_vel_cmd[cart_id] = 0.0;
					p_cart->cart_pos_cmd[cart_id] = p_motion->pose.pose_start[cart_id];
				}
			}
			cart[cart_id] = p_cart->cart_pos_cmd[cart_id];
			cart_vel[cart_id] = p_cart->cart_vel_cmd[cart_id];
		}
		//FILE* fp30;
	   //fp30 = fopen("C://Users//hqi//Desktop//debugging_data//cart_cmd21.txt", "w+");
	   //fprintf(fp30,"%lf  %lf  %lf  %lf  %lf  %lf\n", cart[0], cart[1], cart[2], cart[3], cart[4], cart[5]);
		
		if ((p_motion_next->blending.is_need_blending) && (blending_current_time > 0))
		{
			if (p_motion_next->prfcoord_type == COORD_TYPE_CART)
			{
				//position blending in joint
				gRobot.robot_dev[id].module.kinematics.pfIK(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart, joint, 1);
				gRobot.robot_dev[id].module.kinematics.pfIK(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_next, joint_next, 1);

				//---------------------------------------
				//vel blending in joint
				for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
				{
					p_joint[joint_id].joint_pos_cmd = joint[joint_id];
				}
				gRobot.robot_dev[id].module.kinematics.pfIJ(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_vel, joint_vel);

				for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
				{
					p_joint[joint_id].joint_pos_cmd = joint_next[joint_id];
				}
				gRobot.robot_dev[id].module.kinematics.pfIJ(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_vel_next, joint_vel_next);
			}
			else
			{
				//because the next has calculated joint ,so we do not calculate again,
				//we just calculate the current position blending in joint
				gRobot.robot_dev[id].module.kinematics.pfIK(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart, joint, 1);

				//---------------------------------------
				//vel blending in joint
				for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
				{
					p_joint[joint_id].joint_pos_cmd = joint[joint_id];
				}
				gRobot.robot_dev[id].module.kinematics.pfIJ(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_vel, joint_vel);

				//joint_next and joint_vel_next come from before
			}

			//----------------------------------------------------------------
			//common part
			for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
			{

				p_joint[joint_id].joint_vel_cmd = joint_vel[joint_id] + joint_vel_next[joint_id];

				p_joint[joint_id].joint_pos_cmd = joint[joint_id] + joint_next[joint_id] - (*p_motion->pose.joint_end_vector.point)[joint_id];

				joint[joint_id] = p_joint[joint_id].joint_pos_cmd;
				joint_vel[joint_id] = p_joint[joint_id].joint_vel_cmd;
			}
			///---------------------------------------------------------------
			//change back to the cart
			gRobot.robot_dev[id].module.kinematics.pfFK(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart, joint);
			gRobot.robot_dev[id].module.kinematics.pfFJ(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_vel, joint_vel);

			//set the value to the cart module
			for (cart_id = 0; cart_id < p_config->cart_dim; cart_id++)
			{
				p_cart->cart_pos_cmd[cart_id] = cart[cart_id];
				p_cart->cart_vel_cmd[cart_id] = cart_vel[cart_id];
			}
		}
		else
		{
			//if do not need blending.

			//get joint pos
			gRobot.robot_dev[id].module.kinematics.pfIK(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart, joint, 1);

			for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
			{
				p_joint[joint_id].joint_pos_cmd = joint[joint_id];

				JointPosCmdCheckLimit(&(p_joint[joint_id]));
			}
			//get joint vel
			gRobot.robot_dev[id].module.kinematics.pfIJ(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_vel, joint_vel);

			for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
			{
				p_joint[joint_id].joint_vel_cmd = joint_vel[joint_id];
			}

		}
		//----------------------------------------------------------------------------------------------
		/* finish */
		if (p_trajectory->current_time >= p_motion->traj_cart_rot.time_series[7])
		{
			//if (gRobot.robot_dev[id].mode.mode == ROBOT_MODULE_SCRIPT_MODE || gRobot.robot_dev[id].mode.mode == ROBOT_MODULE_MACRO_PROGRAM_MODE)
			//{
		//		fclose(fp30);
			p_motion->state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_IDL;

			// clear states
			p_trajectory->current_time = 0;
			p_trajectory->motion_block_execute = p_motion_next;


			/*if (gRobot.robot_dev[id].mode.resume_flag == ROBOT_RESUME_REPEATE_MODE_PART2_REPEAT)
			{
				//reset resume flag
				gRobot.robot_dev[id].mode.resume_flag = ROBOT_RESUME_REPEATE_MODE_PART0_FINISH;
				// change the  calculate pointer to correct block
				p_trajectory->motion_block_calculate = p_motion;

			}
			/* inform to create new motion block ,then you can create new motion block
			//rtems_semaphore_release(gRobot.robot_dev[id].mgr.script_task_sem);
			//rtems_semaphore_release(gRobot.robot_dev[id].mgr.block_finish_sem);

			if (gRobot.robot_dev[id].mode.mode == ROBOT_MODULE_MACRO_PROGRAM_MODE) {
				gRobot.robot_dev[id].mode.rmacro.block_nr--;
			}
		}
		else if (gRobot.robot_dev[id].mode.mode == ROBOT_MODULE_JOG_MODE)
		{
			p_motion->state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_IDL;

			// clear states
			p_trajectory->current_time = 0;
			p_trajectory->motion_block_execute = p_motion_next;

			p_trajectory->motion_block_buffer[0].state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_IDL;
			p_trajectory->motion_block_buffer[1].state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_IDL;
			p_trajectory->motion_block_buffer[2].state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_IDL;
			gRobot.robot_dev[id].mode.rjog.block_num = 0;
			gRobot.robot_dev[id].mode.rjog.stop_over = 1;
			gRobot.robot_dev[id].mode.rjog.run_key = 0;
			gRobot.robot_dev[id].mode.rjog.jog_dir = gRobot.robot_dev[id].mode.rjog.jog_dir_tmp;
		}*/
		}
	}
		break;
	
	case TRAJECTORY_MODULE_MOTION_TYPE_CIRCL:
	{
		p_motion_next = get_next_motion_block(p_trajectory->motion_block_buffer, p_motion_next);
		p_last_motion = get_last_motion_block(p_trajectory->motion_block_buffer, p_last_motion);

		if ((!p_trajectory->current_time) && (p_motion_next->blending.is_need_blending))
		{
			/* when start motion, check the next blending condition first */
			if (!p_motion_next->blending.is_ready_blending)
			{
				/* Wait to finish calculating the motion profile parameter and blending parameter*/
				return 0;
			}
		}
		// update clock
		p_trajectory->current_time++;
		//p_trajectory->abs_time ++;

		if (p_motion->blending.is_need_blending)
		{
			/* start motion at the time of finishing blending.*/
			p_trajectory->current_time = p_motion->blending.blending_time_length + 1;
			p_motion->blending.is_need_blending = 0; /* no need now, for blending is finished! */
		}

		// Trajectory generated by a polynomial.
		//this part is do not need blending ,which in the center of the block ,maybe start and end all need blending

		if (p_motion->cartcircle_sync == CARTCIRCLE_TYPE_SYNC_TIME_SCALE)
		{
			//if time scale mode ,pos and posture every sector time is different,but the arrived time is same .so we need seperate them
			//pos
			for (counter = 0; counter < 7; counter++)
			{
				if ((p_trajectory->current_time > p_motion->trajectory.time_series[counter]) && \
					(p_trajectory->current_time <= p_motion->trajectory.time_series[counter + 1]))
				{
					/* determine the time interval,and calculate the states in the time interval */
					clock_time = p_trajectory->current_time - p_motion->trajectory.time_series[counter];
					clock_time *= p_motion->profile.profile_sample_time;

					calculate_acceleration_from_profile(clock_time,
						p_motion->trajectory.jerk_series[counter],
						p_motion->trajectory.acc_series[counter],
						&p_motion->trajectory.acc);
					calculate_velocity_from_profile(clock_time,
						p_motion->trajectory.jerk_series[counter],
						p_motion->trajectory.acc_series[counter],
						p_motion->trajectory.vel_series[counter],
						&p_motion->trajectory.vel);
					calculate_position_from_profile(clock_time,
						p_motion->trajectory.jerk_series[counter],
						p_motion->trajectory.acc_series[counter],
						p_motion->trajectory.vel_series[counter],
						p_motion->trajectory.pos_series[counter],
						&p_motion->trajectory.pos);
					circle_pos = p_motion->trajectory.pos;
					circle_vel = p_motion->trajectory.vel;
					break;
				}
			}
			//posture
			for (counter1 = 0; counter1 < 7; counter1++)
			{
				if ((p_trajectory->current_time > p_motion->traj_cart_rot.time_series[counter1]) && \
					(p_trajectory->current_time <= p_motion->traj_cart_rot.time_series[counter1 + 1]))
				{
					/* determine the time interval,and calculate the states in the time interval */
					clock_time = p_trajectory->current_time - p_motion->traj_cart_rot.time_series[counter1];
					clock_time *= p_motion->prf_cart_rot.profile_sample_time;

					if (p_motion->pose.axis_angle_flag == 1)
					{
						//calculate rotate angle pos
						calculate_position_from_profile(clock_time,
							p_motion->traj_cart_rot.jerk_series[counter1],
							p_motion->traj_cart_rot.acc_series[counter1],
							p_motion->traj_cart_rot.vel_series[counter1],
							p_motion->traj_cart_rot.pos_series[counter1],
							&p_motion->traj_cart_rot.pos);
						//calculate rotate angle spd
						calculate_velocity_from_profile(clock_time,
							p_motion->traj_cart_rot.jerk_series[counter1],
							p_motion->traj_cart_rot.acc_series[counter1],
							p_motion->traj_cart_rot.vel_series[counter1],
							&p_motion->traj_cart_rot.vel);
					}
					else
					{
					}
					break;
				}
			}
		}
		else
		{
			//if time same mode ,pos and posture every sector time is same ,so we just use same time sector to do pos's trajectory and posture's trajectory
			for (counter = 0; counter < 7; counter++)
			{
				if ((p_trajectory->current_time > p_motion->trajectory.time_series[counter]) && \
					(p_trajectory->current_time <= p_motion->trajectory.time_series[counter + 1]))
				{
					/* determine the time interval,and calculate the states in the time interval */
					clock_time = p_trajectory->current_time - p_motion->trajectory.time_series[counter];
					clock_time *= p_motion->profile.profile_sample_time;

					calculate_acceleration_from_profile(clock_time,
						p_motion->trajectory.jerk_series[counter],
						p_motion->trajectory.acc_series[counter],
						&p_motion->trajectory.acc);
					calculate_velocity_from_profile(clock_time,
						p_motion->trajectory.jerk_series[counter],
						p_motion->trajectory.acc_series[counter],
						p_motion->trajectory.vel_series[counter],
						&p_motion->trajectory.vel);
					calculate_position_from_profile(clock_time,
						p_motion->trajectory.jerk_series[counter],
						p_motion->trajectory.acc_series[counter],
						p_motion->trajectory.vel_series[counter],
						p_motion->trajectory.pos_series[counter],
						&p_motion->trajectory.pos);
					circle_pos = p_motion->trajectory.pos;
					circle_vel = p_motion->trajectory.vel;

					if (p_motion->pose.axis_angle_flag == 1)
					{
						//calculate rotate angle pos
						calculate_position_from_profile(clock_time,
							p_motion->traj_cart_rot.jerk_series[counter],
							p_motion->traj_cart_rot.acc_series[counter],
							p_motion->traj_cart_rot.vel_series[counter],
							p_motion->traj_cart_rot.pos_series[counter],
							&p_motion->traj_cart_rot.pos);
						//calculate rotate angle spd
						calculate_velocity_from_profile(clock_time,
							p_motion->traj_cart_rot.jerk_series[counter],
							p_motion->traj_cart_rot.acc_series[counter],
							p_motion->traj_cart_rot.vel_series[counter],
							&p_motion->traj_cart_rot.vel);
					}
					else
					{
					}

					break;
				}
			}
		}

		//calculate posture
		if (p_motion->pose.axis_angle_flag == 1)
		{
			//angle-axis convert to rotate matrix
			matxx_axisAngl2Rotm(p_motion->traj_cart_rot.pos,		//angle in rad
				&p_motion->pose.axis_vector,		//axis
				&p_motion->pose.rotation_matxx);

			/*  B=R*A */
			matxx_multiply(&p_motion->pose.rotation_matxx, &p_motion->pose.rot_start, &p_motion->pose.rotation_matxx);

			//change_10_29
			/*
			// calculate   pitch, roll, yaw
			cart[4] = atan2(-*(*(p_motion->pose.rotation_matxx.point + 0) + 2), pow(pow(*(*(p_motion->pose.\
			rotation_matxx.point + 0) + 0), 2) + pow(*(*(p_motion->pose.rotation_matxx.point + 0) + 1), 2), 0.5));

			if (fabs(cart[4] - MOTION_MODULE_CONSTANT_PI / 2)<1E-10)
			{
			cart[5] = 0;
			cart[3] = atan2(*(*(p_motion->pose.rotation_matxx.point + 1) + 0), *(*(p_motion->pose.rotation_matxx.point + 1) + 1));
			}
			else if (fabs(cart[4] + MOTION_MODULE_CONSTANT_PI / 2)<1E-10)
			{
			cart[5] = 0;
			cart[3] = -atan2(*(*(p_motion->pose.rotation_matxx.point + 1) + 0), *(*(p_motion->pose.rotation_matxx.point + 1) + 1));
			}
			else
			{
			cart[5] = atan2(*(*(p_motion->pose.rotation_matxx.point + 0) + 1) / cos(cart[4]), *(*(p_motion->pose.\
			rotation_matxx.point + 0) + 0) / cos(cart[4]));
			cart[3] = atan2(*(*(p_motion->pose.rotation_matxx.point + 1) + 2) / cos(cart[4]), *(*(p_motion->pose.\
			rotation_matxx.point + 2) + 2) / cos(cart[4]));
			}
			*/
			r2xyz(&p_motion->pose.rotation_matxx, &cart[3], 1);

			matxx_transpose(&p_motion->pose.rot_start, &p_motion->pose.rot_tmp);

			matxx_multiply(&p_motion->pose.rot_tmp, &p_motion->pose.axis_vector, &p_motion->pose.tmp31);

			matxx_k_mult(p_motion->traj_cart_rot.vel, &p_motion->pose.tmp31);

			//change_10_29
			/*
			w[0] = *(*(p_motion->pose.tmp31.point + 0) + 0);
			w[1] = *(*(p_motion->pose.tmp31.point + 0) + 1);
			w[2] = *(*(p_motion->pose.tmp31.point + 0) + 2);
			// d a /d t          [ 0    sr    cr    ]   [ Wx ]
			// d b /d t = 1/cb * [ 0    cscb  -srcb ] * [ Wy ]
			// d r /d t          [ cb   srsb  crsb  ]   [ Wz ]
			if (fabs(cart_next[4] - MOTION_MODULE_CONSTANT_PI / 2) < MOTION_MODULE_CONSTANT_MIN_POSITIVE)
			{
			cart_vel_next[4] = w[1] / cos(cart_next[3]);
			cart_vel_next[3] = -w[2] / sin(cart_next[3]);
			cart_vel_next[5] = cart_vel_next[3] - w[0];

			}
			else
			{
			cart_vel_next[5] = 1 / cos(cart_next[4])*(sin(cart_next[3])*w[1] + cos(cart_next[3])* w[2]);
			cart_vel_next[4] = 1 / cos(cart_next[4])*(cos(cart_next[3])*cos(cart_next[4])*w[1] - sin(cart_next[3])*cos(cart_next[4])*w[2]);
			cart_vel_next[3] = w[0] + sin(cart_next[3])*tan(cart_next[4])*w[1] + cos(cart_next[3])*tan(cart_next[4])*w[2];
			}

			cart[3] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
			cart[4] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
			cart[5] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
			cart_vel[3] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
			cart_vel[4] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
			cart_vel[5] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
			*/

			axisvel2dxyz(&p_motion->pose.tmp31, &cart[3], &cart_vel[3], 0);


		}
		else
		{
			cart[3] = p_motion->pose.pose_start[3];
			cart[4] = p_motion->pose.pose_start[4];
			cart[5] = p_motion->pose.pose_start[5];
			cart_vel[3] = 0;
			cart_vel[4] = 0;
			cart_vel[5] = 0;

		}
		/* get position command
		* P0 + [RotM(s,w) - I] * CP0 ,where : s is angle,w is axis
		* */
		// get arc length corresponding angle in Rad
		circle_angle_delta = circle_pos * p_motion->pose.circle_radius_inv;

		//angle-axis convert to rotate matrix
		matxx_axisAngl2Rotm(-circle_angle_delta,		//angle
			&p_motion->pose.unit_vector,		//axis
			&p_motion->pose.rotation_matxx);


		// Let XYZ = P0
		matxx_assign(&p_motion->pose.cartesian_xyz,
			p_motion->pose.pose_start,
			3);
		/* RotM(s,w) - I */
		for (counter = 0; counter < 3; counter++)
		{
			(*(p_motion->pose.rotation_matxx.point + counter))[counter] -= 1.0;
		}

		/* do rotation */
		//cartesian_xyz = center_vector*rotation_matxx + cartesian_xyz(this is start pos)
		matxx_mat_vec_opt(&p_motion->pose.center_vector,
			&p_motion->pose.rotation_matxx,
			1.0,
			1.0,
			&p_motion->pose.cartesian_xyz);
		/* get velocity command
		* dP(s(t))/dt = dP(s)/ds * ds(t)/dt   ---->s(t) is angle
		* where ds(t)/dt  = w
		* dP(s)/ds = dRotM(s,w)/ds * CP0, other part is constant ,so is zero
		*
		* and vel = dRotM(s,w)/ds * CP0 * w
		* */
		//v = w*r    w = v/r
		circle_w = circle_vel* p_motion->pose.circle_radius_inv;
		matxx_axisAngle2protm(circle_angle_delta,			//angle
			circle_angle_delta,			//just for not error
			&p_motion->pose.unit_vector,	//axis
			&p_motion->pose.rotation_matxx); //rotate matrix differential

		matxx_mat_vec_opt(&p_motion->pose.center_vector,	//cp0
			&p_motion->pose.rotation_matxx,   //rotate matrix differential
			circle_w,						    //w
			0.0,
			&p_motion->pose.cartesian_xyz_vel);
		//---------------------------------------------------------------
		/* get the command */
		for (cart_id = 0; cart_id < p_config->cart_trans_dim; cart_id++)
		{
			matxx_get_element(&p_motion->pose.cartesian_xyz,
				cart_id,
				0,
				&p_cart->cart_pos_cmd[cart_id]);

			matxx_get_element(&p_motion->pose.cartesian_xyz_vel,
				cart_id,
				0,
				&p_cart->cart_vel_cmd[cart_id]);
			cart[cart_id] = p_cart->cart_pos_cmd[cart_id];
			cart_vel[cart_id] = p_cart->cart_vel_cmd[cart_id];
		}
		//--------------------------------------
		//the next block need blending,so do the end blending ,which also is the next start blending
		if (p_motion_next->blending.is_need_blending)
		{
			blending_current_time = p_trajectory->current_time - p_motion_next->blending.blending_start_time;
			if (blending_current_time > 0)
			{
				if (p_motion_next->prfcoord_type == COORD_TYPE_CART)
				{
					for (counter = 0; counter < 7; counter++)
					{
						if ((blending_current_time > p_motion_next->trajectory.time_series[counter]) && \
							(blending_current_time <= p_motion_next->trajectory.time_series[counter + 1]))
						{
							/* determine the time interval,and calculate the states in the time interval */
							clock_time = blending_current_time - p_motion_next->trajectory.time_series[counter];
							clock_time *= p_motion_next->profile.profile_sample_time;

							calculate_acceleration_from_profile(clock_time,
								p_motion_next->trajectory.jerk_series[counter],
								p_motion_next->trajectory.acc_series[counter],
								&p_motion_next->trajectory.acc);
							calculate_velocity_from_profile(clock_time,
								p_motion_next->trajectory.jerk_series[counter],
								p_motion_next->trajectory.acc_series[counter],
								p_motion_next->trajectory.vel_series[counter],
								&p_motion_next->trajectory.vel);
							calculate_position_from_profile(clock_time,
								p_motion_next->trajectory.jerk_series[counter],
								p_motion_next->trajectory.acc_series[counter],
								p_motion_next->trajectory.vel_series[counter],
								p_motion_next->trajectory.pos_series[counter],
								&p_motion_next->trajectory.pos);
							break;
						}
					}
					if (p_motion_next->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_LIN)
					{
						line_pos_next = p_motion_next->trajectory.pos;
						line_vel_next = p_motion_next->trajectory.vel;
						for (cart_id = 0; cart_id < p_config->cart_trans_dim; cart_id++)
						{
							cart_next[cart_id] = p_motion_next->pose.pose_start[cart_id] + line_pos_next * p_motion_next->pose.pose_unit_vector[cart_id];
							cart_vel_next[cart_id] = line_vel_next * p_motion_next->pose.pose_unit_vector[cart_id];
						}
					}
					else if (p_motion_next->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_CIRCL)
					{
						circle_pos_next = p_motion_next->trajectory.pos;
						circle_vel_next = p_motion_next->trajectory.vel;
						/* get position command
						* P0 + [RotM(s,w) - I] * CP0 ,where : s is angle,w is axis
						* */
						// get arc length corresponding angle in Rad
						circle_angle_delta = circle_pos_next * p_motion_next->pose.circle_radius_inv;

						//angle-axis convert to rotate matrix
						matxx_axisAngl2Rotm(-circle_angle_delta,		//angle in rad
							&p_motion_next->pose.unit_vector,		//axis
							&p_motion_next->pose.rotation_matxx);

						// Let XYZ = P0
						matxx_assign(&p_motion_next->pose.cartesian_xyz,
							p_motion_next->pose.pose_start,
							3);
						/* RotM(s,w) - I */
						for (counter = 0; counter < 3; counter++)
						{
							(*(p_motion_next->pose.rotation_matxx.point + counter))[counter] -= 1.0;
						}

						/* do rotation */
						//cartesian_xyz = center_vector*rotation_matxx + cartesian_xyz(this is start pos)
						matxx_mat_vec_opt(&p_motion_next->pose.center_vector,
							&p_motion_next->pose.rotation_matxx,
							1.0,
							1.0,
							&p_motion_next->pose.cartesian_xyz);
						/* get velocity command
						* dP(s(t))/dt = dP(s)/ds * ds(t)/dt   ---->s(t) is angle
						* where ds(t)/dt  = w
						* dP(s)/ds = dRotM(s,w)/ds * CP0, other part is constant ,so is zero
						*
						* and vel = dRotM(s,w)/ds * CP0 * w
						* */
						//v = w*r    w = v/r
						circle_w = circle_vel_next * p_motion_next->pose.circle_radius_inv;
						matxx_axisAngle2protm(circle_angle_delta, 1,			//angle
							&p_motion_next->pose.unit_vector,	//axis
							&p_motion_next->pose.rotation_matxx); //rotate matrix differential

						matxx_transpose(&p_motion_next->pose.rotation_matxx, &p_motion_next->pose.rot_tmp);
						matxx_copy(&p_motion_next->pose.rot_tmp, &p_motion_next->pose.rotation_matxx);

						matxx_mat_vec_opt(&p_motion_next->pose.center_vector,	//cp0
							&p_motion_next->pose.rotation_matxx,   //rotate matrix differential
							circle_w,						    //w
							0.0,
							&p_motion_next->pose.cartesian_xyz_vel);
						for (cart_id = 0; cart_id < p_config->cart_trans_dim; cart_id++)
						{
							cart_next[cart_id] = *(*(p_motion_next->pose.cartesian_xyz.point + 0) + cart_id);
							cart_vel_next[cart_id] = *(*(p_motion_next->pose.cartesian_xyz_vel.point + 0) + cart_id);
						}
					}
					else
					{
						return -1;
					}

					//calculate the posture.
					// when blending ,position and posture have the same time.
					for (counter = 0; counter < 7; counter++)
					{
						if ((blending_current_time > p_motion_next->traj_cart_rot.time_series[counter]) && \
							(blending_current_time <= p_motion_next->traj_cart_rot.time_series[counter + 1]))
						{
							/* determine the time interval,and calculate the states in the time interval */
							clock_time = blending_current_time - p_motion_next->traj_cart_rot.time_series[counter];
							clock_time *= p_motion_next->profile.profile_sample_time;
							//calculate pos
							calculate_position_from_profile(clock_time,
								p_motion_next->traj_cart_rot.jerk_series[counter],
								p_motion_next->traj_cart_rot.acc_series[counter],
								p_motion_next->traj_cart_rot.vel_series[counter],
								p_motion_next->traj_cart_rot.pos_series[counter],
								&p_motion_next->traj_cart_rot.pos);

							calculate_velocity_from_profile(clock_time,
								p_motion_next->traj_cart_rot.jerk_series[counter],
								p_motion_next->traj_cart_rot.acc_series[counter],
								p_motion_next->traj_cart_rot.vel_series[counter],
								&p_motion_next->traj_cart_rot.vel);

							if (p_motion_next->pose.axis_angle_flag == 1)
							{

								//angle-axis convert to rotate matrix
								matxx_axisAngl2Rotm(p_motion_next->traj_cart_rot.pos,		//angle in rad
									&p_motion_next->pose.axis_vector,		//axis
									&p_motion_next->pose.rotation_matxx);
								/*  B=R*A */
								matxx_multiply(&p_motion_next->pose.rotation_matxx, &p_motion_next->pose.rot_start, &p_motion_next->pose.rotation_matxx);

								/* calculate   pitch, roll, yaw */
								//change_10_29
								/*
								cart_next[4] = atan2(-*(*(p_motion_next->pose.rotation_matxx.point + 0) + 2), pow(pow(*(*(p_motion_next->pose.\
								rotation_matxx.point + 0) + 0), 2) + pow(*(*(p_motion_next->pose.rotation_matxx.point + 0) + 1), 2), 0.5));

								if (fabs(cart_next[4] - MOTION_MODULE_CONSTANT_PI / 2) < 1E-10)
								{
								cart_next[5] = 0;
								cart_next[3] = atan2(*(*(p_motion_next->pose.rotation_matxx.point + 1) + 0), *(*(p_motion_next->pose.rotation_matxx.point + 1) + 1));
								}
								else if (fabs(cart_next[4] + MOTION_MODULE_CONSTANT_PI / 2) < 1E-10)
								{
								cart_next[5] = 0;
								cart_next[3] = -atan2(*(*(p_motion_next->pose.rotation_matxx.point + 1) + 0), *(*(p_motion_next->pose.rotation_matxx.point + 1) + 1));
								}
								else
								{
								cart_next[5] = atan2(*(*(p_motion_next->pose.rotation_matxx.point + 0) + 1) / cos(cart_next[4]), *(*(p_motion_next->pose.\
								rotation_matxx.point + 0) + 0) / cos(cart_next[4]));
								cart_next[3] = atan2(*(*(p_motion_next->pose.rotation_matxx.point + 1) + 2) / cos(cart_next[4]), *(*(p_motion_next->pose.\
								rotation_matxx.point + 2) + 2) / cos(cart_next[4]));
								}
								*/

								r2xyz(&p_motion_next->pose.rotation_matxx, &cart_next[3], 1);


								//decompose the angular velocity to basical coordinate system
								matxx_transpose(&p_motion_next->pose.rot_start, &p_motion_next->pose.rot_tmp);
								matxx_multiply(&p_motion_next->pose.rot_tmp, &p_motion_next->pose.axis_vector, &p_motion_next->pose.tmp31);

								// Wx  [ axis_x ]
								// Wy =[ axis_y ] * angular_vel
								// Wz  [ axis_z ]
								matxx_k_mult(p_motion_next->traj_cart_rot.vel, &p_motion_next->pose.tmp31);

								//change_10_29
								/*
								w[0] = *(*(p_motion_next->pose.tmp31.point + 0) + 0);
								w[1] = *(*(p_motion_next->pose.tmp31.point + 0) + 1);
								w[2] = *(*(p_motion_next->pose.tmp31.point + 0) + 2);

								// d a /d t          [ 0    sr    cr    ]   [ Wx ]
								// d b /d t = 1/cb * [ 0    cscb  -srcb ] * [ Wy ]
								// d r /d t          [ cb   srsb  crsb  ]   [ Wz ]
								if (fabs(cart_next[4] - MOTION_MODULE_CONSTANT_PI / 2) < MOTION_MODULE_CONSTANT_MIN_POSITIVE)
								{
								cart_vel_next[4] = w[1] / cos(cart_next[3]);
								cart_vel_next[3] = -w[2] / sin(cart_next[3]);
								cart_vel_next[5] = cart_vel_next[3] - w[0];

								}
								else
								{
								cart_vel_next[5] = 1 / cos(cart_next[4])*(sin(cart_next[3])*w[1] + cos(cart_next[3])* w[2]);
								cart_vel_next[4] = 1 / cos(cart_next[4])*(cos(cart_next[3])*cos(cart_next[4])*w[1] - sin(cart_next[3])*cos(cart_next[4])*w[2]);
								cart_vel_next[3] = w[0] + sin(cart_next[3])*tan(cart_next[4])*w[1] + cos(cart_next[3])*tan(cart_next[4])*w[2];
								}

								cart_next[3] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
								cart_next[4] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
								cart_next[5] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
								cart_vel_next[3] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
								cart_vel_next[4] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
								cart_vel_next[5] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
								*/
								axisvel2dxyz(&p_motion_next->pose.tmp31, &cart_next[3], &cart_vel_next[3], 0);



							}
							else
							{
								cart_next[3] = p_motion_next->pose.pose_start[3];
								cart_next[4] = p_motion_next->pose.pose_start[4];
								cart_next[5] = p_motion_next->pose.pose_start[5];
								cart_vel_next[3] = 0;
								cart_vel_next[4] = 0;
								cart_vel_next[5] = 0;

							}

							/* Break the FOR */
							break;
						}
					}
				}

				// blending in joint
				else
				{
					for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
					{
						for (counter = 0; counter<8; counter++)
						{
							if ((blending_current_time > p_motion_next->traj_joint[joint_id].time_series[counter]) && \
								(blending_current_time <= p_motion_next->traj_joint[joint_id].time_series[counter + 1]))
							{
								/* determine the time interval,and calculate the states in the time interval */
								clock_time = blending_current_time - p_motion_next->traj_joint[joint_id].time_series[counter];
								clock_time *= p_motion_next->prf_joint[joint_id].profile_sample_time;

								calculate_acceleration_from_profile(clock_time,
									p_motion_next->traj_joint[joint_id].jerk_series[counter],
									p_motion_next->traj_joint[joint_id].acc_series[counter],
									&p_motion_next->traj_joint[joint_id].acc);
								calculate_velocity_from_profile(clock_time,
									p_motion_next->traj_joint[joint_id].jerk_series[counter],
									p_motion_next->traj_joint[joint_id].acc_series[counter],
									p_motion_next->traj_joint[joint_id].vel_series[counter],
									&p_motion_next->traj_joint[joint_id].vel);
								calculate_position_from_profile(clock_time,
									p_motion_next->traj_joint[joint_id].jerk_series[counter],
									p_motion_next->traj_joint[joint_id].acc_series[counter],
									p_motion_next->traj_joint[joint_id].vel_series[counter],
									p_motion_next->traj_joint[joint_id].pos_series[counter],
									&p_motion_next->traj_joint[joint_id].pos);
								joint_next[joint_id] = (*(p_motion_next->pose.joint_start_vector.point))[joint_id] + p_motion_next->traj_joint[joint_id].pos;
								joint_vel_next[joint_id] = p_motion_next->traj_joint[joint_id].vel;
								break;
							}
						}
					}
				}
			}
		}

		if ((p_motion_next->blending.is_need_blending) && (blending_current_time > 0))
		{
			if (p_motion_next->prfcoord_type == COORD_TYPE_CART)
			{
				//position blending in joint
				gRobot.robot_dev[id].module.kinematics.pfIK(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart, joint, 1);
				gRobot.robot_dev[id].module.kinematics.pfIK(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_next, joint_next, 1);

				//---------------------------------------
				//vel blending in joint
				for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
				{
					p_joint[joint_id].joint_pos_cmd = joint[joint_id];
				}
				gRobot.robot_dev[id].module.kinematics.pfIJ(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_vel, joint_vel);

				for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
				{
					p_joint[joint_id].joint_pos_cmd = joint_next[joint_id];
				}
				gRobot.robot_dev[id].module.kinematics.pfIJ(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_vel_next, joint_vel_next);
			}
			else
			{
				//because the next has calculated joint ,so we do not calculate again,
				//we just calculate the current position blending in joint
				gRobot.robot_dev[id].module.kinematics.pfIK(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart, joint, 1);

				//---------------------------------------
				//vel blending in joint
				for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
				{
					p_joint[joint_id].joint_pos_cmd = joint[joint_id];
				}
				gRobot.robot_dev[id].module.kinematics.pfIJ(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_vel, joint_vel);

				//joint_next and joint_vel_next come from before
			}

			//----------------------------------------------------------------
			//common part
			for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
			{

				p_joint[joint_id].joint_vel_cmd = joint_vel[joint_id] + joint_vel_next[joint_id];

				p_joint[joint_id].joint_pos_cmd = joint[joint_id] + joint_next[joint_id] - (*p_motion->pose.joint_end_vector.point)[joint_id];

				joint[joint_id] = p_joint[joint_id].joint_pos_cmd;
				joint_vel[joint_id] = p_joint[joint_id].joint_vel_cmd;
			}
			///---------------------------------------------------------------
			//change back to the cart
			gRobot.robot_dev[id].module.kinematics.pfFK(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart, joint);
			gRobot.robot_dev[id].module.kinematics.pfFJ(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_vel, joint_vel);

			//set the value to the cart module
			for (cart_id = 0; cart_id < p_config->cart_dim; cart_id++)
			{
				p_cart->cart_pos_cmd[cart_id] = cart[cart_id];
				p_cart->cart_vel_cmd[cart_id] = cart_vel[cart_id];
			}
		}
		else
		{
			//if do not need blending.

			//get joint pos
			gRobot.robot_dev[id].module.kinematics.pfIK(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart, joint, 1);

			for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
			{
				p_joint[joint_id].joint_pos_cmd = joint[joint_id];

				JointPosCmdCheckLimit(&(p_joint[joint_id]));
			}
			//get joint vel
			gRobot.robot_dev[id].module.kinematics.pfIJ(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart_vel, joint_vel);

			for (joint_id = 0; joint_id < p_config->joint_dim; joint_id++)
			{
				p_joint[joint_id].joint_vel_cmd = joint_vel[joint_id];
			}

		}
		//---------------------------------------
		/* finish */
		if (p_trajectory->current_time >= p_motion->trajectory.time_series[7])
		{
			//if (gRobot.robot_dev[id].mode.mode == ROBOT_MODULE_SCRIPT_MODE || gRobot.robot_dev[id].mode.mode == ROBOT_MODULE_MACRO_PROGRAM_MODE)
			{

				p_motion->state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_IDL;

				// clear states
				p_trajectory->current_time = 0;
				p_trajectory->motion_block_execute = p_motion_next;


				/*if (gRobot.robot_dev[id].mode.resume_flag == ROBOT_RESUME_REPEATE_MODE_PART2_REPEAT)
				{
					//reset resume flag
					gRobot.robot_dev[id].mode.resume_flag = ROBOT_RESUME_REPEATE_MODE_PART0_FINISH;
					// change the  calculate pointer to correct block
					p_trajectory->motion_block_calculate = p_motion;

				}
				/* inform to create new motion block ,then you can create new motion block
				//rtems_semaphore_release(gRobot.robot_dev[id].mgr.script_task_sem);
				rtems_semaphore_release(gRobot.robot_dev[id].mgr.block_finish_sem);

				if (gRobot.robot_dev[id].mode.mode == ROBOT_MODULE_MACRO_PROGRAM_MODE) {
					gRobot.robot_dev[id].mode.rmacro.block_nr--;
				}*/
			}
			//else
			{

			}
		}

		break;
	}
	case TRAJECTORY_MODULE_MOTION_TYPE_NONE:
	default:
		break;
	}
	return 0;
}



/* EOF
* */
