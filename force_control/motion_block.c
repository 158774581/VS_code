#include "robot_header_files.h"
#include "traj_Gen.h"

int16 InsertNewMotionBlockCnvRbt(trajectory_module *m_traj, robot_config_module* p_config, mb_insert* mb, Uint8 id)
{
	motion_block* new = m_traj->motion_block_new_insert;
	motion_block* last = NULL;
	Uint8 counter = 0;
	Uint8 i;
	last = get_last_motion_block(m_traj->motion_block_buffer, new);
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
		new->motion_type = mb->type;

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
				p_config, new->pose.pose_end, mb->loc_end.loc_pos.axis);
			gRobot.robot_dev[id].module.kinematics.pfIK(&(gRobot.robot_dev[id].module.cart), &(gRobot.robot_dev[id].module.joint[0]), \
				&(gRobot.robot_dev[id].module.kinematics), \
				p_config, new->pose.pose_end, mb->loc_end.loc_pos.axis, 0);

			// assign end joint angle in degree
			matxx_assign(&new->pose.joint_end_vector, mb->loc_end.loc_pos.axis, p_config->joint_dim);

			//copy last end vector to the new start vector or set start pose
			if (mb->pos_start_assigned == 0)
			{
				matxx_copy(&last->pose.joint_end_vector, &new->pose.joint_start_vector);
			}
			else
			{
				memcpy(new->pose.joint_start_vector.point[0],mb->loc_start.loc_pos.axis, p_config->joint_dim * sizeof(double));
			}

			//now we need profile [0,0,0,0]----[90,80,100,20]  100---mm 90--degree
			//MOTION PARAMETER profile value is % value
			for (i = 0; i < p_config->joint_dim; ++i)
			{
				new->prf_joint[i].profile_vel = mb->prf.vel * gRobot.robot_dev[id].module.joint[i].prm.joint_sys_vel * MOTION_MODULE_CONSTANT_PERCENTAGE;
				new->prf_joint[i].profile_acc = mb->prf.acc * gRobot.robot_dev[id].module.joint[i].prm.joint_sys_acc * MOTION_MODULE_CONSTANT_PERCENTAGE;
				new->prf_joint[i].profile_tacc = mb->prf.accramp; //s
				new->prf_joint[i].profile_dec = mb->prf.dec * gRobot.robot_dev[id].module.joint[i].prm.joint_sys_dec * MOTION_MODULE_CONSTANT_PERCENTAGE;
				new->prf_joint[i].profile_tdec = mb->prf.decramp; //s
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
				new->pose.pose_end[counter] = mb->loc_end.loc_pos.cart.array[counter];
				// Last motion end pose is the new motion start pose or set start pose
				if (mb->pos_start_assigned == 0)
				{
					new->pose.pose_start[counter] = last->pose.pose_end[counter];
				}
				else
				{
					new->pose.pose_start[counter] = mb->loc_start.loc_pos.cart.array[counter];
				}
			}

			//to get joint end vector
			gRobot.robot_dev[id].module.kinematics.pfIK(&(gRobot.robot_dev[id].module.cart), &(gRobot.robot_dev[id].module.joint[0]), \
				&(gRobot.robot_dev[id].module.kinematics), \
				p_config, new->pose.pose_end, *(new->pose.joint_end_vector.point), 0);

			// MOTION PARAMETER for cartesian translate move
			new->profile.profile_vel = mb->prf.vel * gRobot.robot_dev[id].module.cart.prm.cart_trans_sys_vel * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->profile.profile_acc = mb->prf.acc * gRobot.robot_dev[id].module.cart.prm.cart_trans_sys_acc * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->profile.profile_tacc = mb->prf.accramp;  //s
			new->profile.profile_dec = mb->prf.dec * gRobot.robot_dev[id].module.cart.prm.cart_trans_sys_dec * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->profile.profile_tdec = mb->prf.decramp;  //s
			new->profile.profile_sample_time = MOTION_MODULE_SAMPLE_TIME;


			// MOTION PARAMETER for cartesian rotate move
			new->prf_cart_rot.profile_vel = mb->prf.vel * gRobot.robot_dev[id].module.cart.prm.cart_rotate_sys_vel * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->prf_cart_rot.profile_acc = mb->prf.acc * gRobot.robot_dev[id].module.cart.prm.cart_rotate_sys_acc * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->prf_cart_rot.profile_tacc = mb->prf.accramp;
			new->prf_cart_rot.profile_dec = mb->prf.dec * gRobot.robot_dev[id].module.cart.prm.cart_rotate_sys_dec * MOTION_MODULE_CONSTANT_PERCENTAGE;
			new->prf_cart_rot.profile_tdec = mb->prf.decramp;
			new->prf_cart_rot.profile_sample_time = MOTION_MODULE_SAMPLE_TIME;

			//set the line pose method,now just for test
			new->cartline_method = CARTLINE_TYPE_AXIS_ANGLE;
			new->cartline_sync = CARTLINE_TYPE_SYNC_TIME_SAME;
			//new->cartline_sync = CARTLINE_TYPE_SYNC_TIME_SCALE;
			break;
		}
		case TRAJECTORY_MODULE_MOTION_TYPE_NONE:
		default:
			break;
		}

		new->state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_USED;
		//--------------------------------------------------------------------------------
		/* point to next motion block */
		m_traj->motion_block_new_insert = get_next_motion_block(m_traj->motion_block_buffer, m_traj->motion_block_new_insert);
	}
	else
	{
		return -1;
	}
	return 0;
}