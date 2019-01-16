/*
* conveyor_tracking.c
*
*  Created on: Nov 26, 2018
*      Author: hqi
*/
#include "robot_ctl_utility.h"
#include "conveyor_tracking.h"
#include "string.h"
#include "stdio.h"
#include "windows.h"
#include "trajectory_generator.h"
#include "robot_app.h"

#define  ullimit(u,l,a)	  a=a>u? u:a<l? l:a
//extern robot_coordinate_system		robot_cs;
int16 InitConveyorTracking(conveyor_tracking* cnv_tra,Uint8 id)
{
	Uint8	i;
	cnv_tra->NrOfRobot					= 1;
	cnv_tra->NrOfConveyor				= 1;

	for (i = 0; i < cnv_tra->NrOfConveyor; i++)
	{
		InitConveyor(&cnv_tra->conveyor[i],i,id);
	}
	for (i = 0; i < cnv_tra->NrOfRobot; i++)
	{
		// user set pose of tool for base while picking
		;
	}

	return 0;
}

int16 InitConveyor(conveyor* cnv, Uint8 id,Uint8 robot_id)
{
	Uint16 i;
	cnv->id = id;
	cnv->last_conveyor = 0;
	cnv->next_conveyor = 0;
	cnv->counts_per_meter = 10000;
	cnv->pulse_2_mm = 1000.0 / cnv->counts_per_meter;
	cnv->mot_pos_fd = 0;
	cnv->sort_strategy = 0;
	cnv->type = 0;
	cnv->queue.wobj_num = 0;

	// vel_fd ;Unit pulse/s
	cnv->mot_vel_fd = 0;
	cnv->mot_acc_fd = 0;
	cnv->mot_vel_cmd = 200;  //pulse /s
	cnv->mot_acc_cmd = 0;
	cnv->mot_pos_cmd = 0;  // pulse
	cnv->spd_change_t = 2;  // s
	cnv->acc_max = 250;  // pulse /s^2
	cnv->vel_max = 500;  // pulse /s

	cnv->pyramid.up_down_time = 0;
	cnv->pyramid.comp_time = 0; // ms

	cnv->pyramid.max_iter = 20;
	cnv->pyramid.eps_pyr = 0.05;//mm
	cnv->pyramid.eps2_pyr = 0.01;//mm

													// PID control
	memset(cnv->pid.Derr, 0, 3 * sizeof(double));
	memset(cnv->pid.Ierr, 0, 3 * sizeof(double));
	memset(cnv->pid.err, 0, 3 * sizeof(double));
	memset(cnv->pid.m, 0, 3 * sizeof(double));
	cnv->pid.Kp = 1;
	cnv->pid.Td = 0.01;
	cnv->pid.Ti = 1000.0;
	cnv->pid.Ierr_max = 2;
	cnv->pid.err_max = 100;  // mm
	cnv->pid.m_max = 100;  // mm
	cnv->pid.eps_pid = 1.0E-2;  // mm 
	cnv->pid.prm_trck_time = 200;  // ms
	cnv->pid.cur_trck_time = 0;  // ms

	cnv->cam.d = 10;  // mm

	// init work object id.
	for (i = 0; i < MAX_WOBJ_NUM; i++)
	{
		cnv->queue.wobj_buff[i].id		= i;
		cnv->queue.wobj_buff[i].state	= WOBJ_STATE_IDLE;
	}
	cnv->queue.insrt_wobj = cnv->queue.wobj_buff;
	cnv->queue.cal_wobj = cnv->queue.wobj_buff;
	cnv->queue.trck_wobj = cnv->queue.wobj_buff;

	cnv->pick_pose[0] = 0;
	cnv->pick_pose[1] = 0;
	cnv->pick_pose[2] = 180;

	return 0;
}

wobj* get_next_wobj(wobj* buff, wobj* cur)
{
	Uint16 id = cur->id;
	id++;
	if (id >= MAX_WOBJ_NUM)
	{
		return buff;
	}
	return &buff[id];
}

wobj* get_last_wobj(wobj* buff, wobj* cur)
{
	int16 id = cur->id;
	id--;
	if (id < 0 || id >= MAX_WOBJ_NUM)
	{
		return &buff[MAX_WOBJ_NUM - 1];
	}
	return &buff[id];
}

void clear_wobj_queue(wobj_queue* queue)
{
	queue->wobj_num		= 0;
	queue->insrt_wobj	= queue->wobj_buff;
	queue->cal_wobj		= queue->wobj_buff;
	queue->trck_wobj	= queue->wobj_buff;
}

// run in master
int16 QueueIn(wobj_queue* queue,Uint8 type,Uint8 robot_id,int32 init_mot_pos, int32 init_mot_vel,double init_y,time* init_time)
{
	// have no space.
	if (queue->wobj_num>= MAX_WOBJ_NUM-1)
	{
		return -1;
	}
	queue->insrt_wobj->type = type;
	queue->insrt_wobj->robot_id = robot_id;
	queue->insrt_wobj->init_mot_pos = init_mot_pos;
	queue->insrt_wobj->init_mot_vel = init_mot_vel;
	queue->insrt_wobj->init_y = init_y;

	queue->insrt_wobj->init_time.year = init_time->year;
	queue->insrt_wobj->init_time.month = init_time->month;
	queue->insrt_wobj->init_time.day = init_time->day;
	queue->insrt_wobj->init_time.hour = init_time->hour;
	queue->insrt_wobj->init_time.minute = init_time->minute;
	queue->insrt_wobj->init_time.second = init_time->second;
	queue->insrt_wobj->init_time.ticks = init_time->ticks;
	queue->insrt_wobj->state = WOBJ_STATE_USED;

	queue->insrt_wobj = get_next_wobj(queue->wobj_buff, queue->insrt_wobj);
	queue->wobj_num++;

	return 0;
}

int16 MallocMotionBlock(motion_block* block)
{


	matxx_malloc(&block->pose.cartesian_xyz, 3, 1);
	matxx_malloc(&(block->pose.joint_start_vector), 8, 1);
	matxx_malloc(&(block->pose.joint_end_vector), 8, 1);
	matxx_malloc(&(block->pose.joint_unit_vector), 8, 1);
	matxx_malloc(&(block->pose.joint_pos), 8, 1);
	matxx_malloc(&(block->pose.joint_temp_vector), 8, 1);
	//---------------------------------------------------------------------
	/* rotation matrix */
	VECTOR3(&(block->pose.axis_vector));
	VECTOR3(&(block->pose.unit_vector));
	VECTOR3(&(block->pose.center_vector));
	SQUARE3(&(block->pose.rotation_matxx));
	SQUARE3(&(block->pose.d_rotation_matxx));
	SQUARE3(&(block->pose.rot_start));
	SQUARE3(&(block->pose.rot_end));
	SQUARE3(&(block->pose.rot_tmp));
	SQUARE3(&(block->pose.rot_tmp1));
	VECTOR3(&block->pose.cartesian_xyz_vel);
	VECTOR3(&block->pose.tmp31);
	VECTOR3(&block->pose.P0);
	VECTOR3(&block->pose.P0P1);
	VECTOR3(&block->pose.P0P2);
	VECTOR3(&block->pose.u);
	VECTOR3(&block->pose.v);
	VECTOR3(&block->pose.w);

	//---------------------------------------------------------------------

	//matxx_malloc(&block->pose.cartesian_eul, 3, 1);

	return 0;
}

int16 CalUpDownTime(conveyor* cnv, conveyor_robot* rbt, robot_config_module* cfg,double up_height,double down_height)
{
	Uint8 id = 0;
	double pos_start[6] = { 0,0,0,0,0,0 };
	double pos_end[6] = { 0,0,0,0,0,0 };
	double profile[5] = { 100, 100,0.1,100,0.1 };
	Uint32 lift_time;
	Uint32 down_time;

	trajectory_module* p_tra=&gRobot.robot_dev[0].module.trajectory;
	//----------------------motion block----------------------------------------------
	
	p_tra->motion_block_tmp.motion_block_id = 100;
	// profile
	p_tra->motion_block_tmp.profile.profile_vel = profile[0] * gRobot.robot_dev[id].module.cart.prm.cart_trans_sys_vel * MOTION_MODULE_CONSTANT_PERCENTAGE;
	p_tra->motion_block_tmp.profile.profile_acc = profile[1] * gRobot.robot_dev[id].module.cart.prm.cart_trans_sys_acc * MOTION_MODULE_CONSTANT_PERCENTAGE;
	p_tra->motion_block_tmp.profile.profile_tacc = profile[2];  //s
	p_tra->motion_block_tmp.profile.profile_dec = profile[3] * gRobot.robot_dev[id].module.cart.prm.cart_trans_sys_dec * MOTION_MODULE_CONSTANT_PERCENTAGE;
	p_tra->motion_block_tmp.profile.profile_tdec = profile[4];  //s
	p_tra->motion_block_tmp.profile.profile_sample_time = MOTION_MODULE_SAMPLE_TIME;
	p_tra->motion_block_tmp.prf_cart_rot.profile_vel = profile[0] * gRobot.robot_dev[id].module.cart.prm.cart_rotate_sys_vel * MOTION_MODULE_CONSTANT_PERCENTAGE;
	p_tra->motion_block_tmp.prf_cart_rot.profile_acc = profile[1] * gRobot.robot_dev[id].module.cart.prm.cart_rotate_sys_acc * MOTION_MODULE_CONSTANT_PERCENTAGE;
	p_tra->motion_block_tmp.prf_cart_rot.profile_tacc = profile[2];
	p_tra->motion_block_tmp.prf_cart_rot.profile_dec = profile[3] * gRobot.robot_dev[id].module.cart.prm.cart_rotate_sys_dec * MOTION_MODULE_CONSTANT_PERCENTAGE;
	p_tra->motion_block_tmp.prf_cart_rot.profile_tdec = profile[4];
	p_tra->motion_block_tmp.prf_cart_rot.profile_sample_time = MOTION_MODULE_SAMPLE_TIME;
	p_tra->motion_block_tmp.cartline_method = CARTLINE_TYPE_AXIS_ANGLE;
	p_tra->motion_block_tmp.motion_type = TRAJECTORY_MODULE_MOTION_TYPE_LIN;
	// pos start
	memcpy(p_tra->motion_block_tmp.pose.pose_start, pos_start, 6 * sizeof(double));
	// pos end 
	pos_end[2] = up_height;
	memcpy(p_tra->motion_block_tmp.pose.pose_end, pos_end, 6 * sizeof(double));
	calculate_motion_parameters(cfg, &p_tra->motion_block_tmp);
	lift_time = (Uint32)(p_tra->motion_block_tmp.trajectory.total_interval/ p_tra->motion_block_tmp.prf_cart_rot.profile_sample_time);
	// pos end 
	pos_end[2] = down_height;
	memcpy(p_tra->motion_block_tmp.pose.pose_end, pos_end, 6 * sizeof(double));
	calculate_motion_parameters(cfg, &p_tra->motion_block_tmp);
	down_time = (Uint32)(p_tra->motion_block_tmp.trajectory.total_interval/ p_tra->motion_block_tmp.prf_cart_rot.profile_sample_time);

	cnv->pyramid.up_down_time = lift_time;// +down_time;
	//cnv->pyramid.up_down_time = 0;
	return 0;
}

/*    queue_trck_dist        start_win_width          
 *     |<-------->|<------------------------------>|                    |
 *	___|__________|________________________________|____________________|_________
 *     |          |U2                              |                    |L2
 *     |          |                                |                    |
 *     |    #     | wobj_init_pos                  |                    |
 *     |          |<------------->|#               |        #           |
 *     |          |                                |                    |
 *     |          |U1                              |                    |L1
 *  ___|______upper limit_____________________valid limit__________lower limit____
 *
 *    # : work object
 */
// conveyor_robot就是gRobot，每个机器人与robot_control并列一个conveyor_tracking.在conveyor_tracking没有机器人的东西，pallas上吧其中的conveyor_robot移植到gRobot中。
// vel of robot must bigger than conveyor ,else the method is false.
int16 PyramidOptimization(conveyor* cnv, conveyor_robot* rbt, robot_config_module* cfg, robot_coordinate_system* p_cs,double* intercept)
{
	int8   rtn					= 0;
	Uint32 iter_times			= 0;
	double wobj_x				= 0;  // work object's pos x in conveyor frame.  Unit mm
	double pos_in_cnv[6]		= { 0,0,0,0,0,0 };
	double pos_in_rbt[6]		= { 0,0,0,0,0,0 };
	double tool_pose[6]			= { 0,0,0,0,0,0 };
	int32 wobj_init_pos			= 0;  // the pos when robot begin moving relate to upper limit. Unit pulse
	int32 xlm					= (Uint32)(cnv->L2[0] / 1000.0*cnv->counts_per_meter);
	int32 xum					= (Uint32)(cnv->U2[0] / 1000.0*cnv->counts_per_meter);

	trajectory_module* p_tra	= &gRobot.robot_dev[0].module.trajectory;
	motion_block* p_block		= &p_tra->motion_block_tmp;

	if (cnv->queue.cal_wobj->state != WOBJ_STATE_USED)
	{
		return -1;
	}

	// simulate robot waiting working get to upper limit
	wobj_init_pos = cnv->mot_pos_fd - cnv->queue.cal_wobj->init_mot_pos - cnv->queue_trck_dist;

	// Conveyor Belt Speed Coordination 
	if (wobj_init_pos < 0)
	{
		// Conveyor Belt Speed Coordination 
		cnv->vel_new = cnv->vel_max;
		return -1;
	}
	else 
	{
		if (wobj_init_pos <= xlm)
		{
			cnv->vel_new = cnv->vel_max*(xlm- wobj_init_pos)/(xlm-xum);
		}
		else
		{
			// work object has pass over lower limit
			printf("the work object is gave up\n");
			cnv->queue.wobj_num--;
			cnv->queue.cal_wobj = get_next_wobj(cnv->queue.wobj_buff, cnv->queue.cal_wobj);
			return -1;
		}
	}
	// calculate should be in rtems task not here. if wobj has pass the valib pick up limit ,robot should't move.
	CalUpDownTime(cnv, rbt, cfg, cnv->down_height, cnv->down_height);
	//----------------------motion block----------------------------------------------
	// profile is configed in CalUpDownTime

	memcpy(p_block->pose.pose_start, cnv->lift_point, 6 * sizeof(double));

	// iteration 's first point is lower limit for conveyor frame
	pos_in_cnv[0] = cnv->L2[0];
	pos_in_cnv[1] = cnv->queue.cal_wobj->init_y;  // work object's pos y in conveyor frame while photographing
	pos_in_cnv[2] = cnv->down_height;  // set by user
	pos_in_cnv[3] = 180; // not use
	pos_in_cnv[4] = 0;  // not use
	pos_in_cnv[5] = 0;  // not use

	// coordinate transpose from conveyor frame to robot frame.
	uP2bPf(p_cs, pos_in_cnv, p_block->pose.pose_end);

	// set pose of tool for base while picking.
	tool_pose[3] = cnv->pick_pose[0];
	tool_pose[4] = cnv->pick_pose[1];
	tool_pose[5] = cnv->pick_pose[2];
	bPt2bPf(p_cs, tool_pose, tool_pose);
	p_block->pose.pose_end[3] = tool_pose[3];
	p_block->pose.pose_end[4] = tool_pose[4];
	p_block->pose.pose_end[5] = tool_pose[5];

	//----------------------------------------------------------------------------

	while (1)
	{
		calculate_motion_parameters(cfg, p_block);

		//bPf2bPt(p_cs,p_block->pose.pose_end , tool_pose);
		// set the valiable pick up limit
		if (iter_times == 0) 
		{
			cnv->start_win_width = (int32)(cnv->L2[0] - cnv->U2[0])/1000.0* cnv->counts_per_meter - \
								   (p_block->trajectory.total_interval/ p_block->prf_cart_rot.profile_sample_time + cnv->pyramid.up_down_time + \
									cnv->pyramid.comp_time)*p_block->profile.profile_sample_time*cnv->mot_vel_fd;
			if (cnv->start_win_width < 0)
			{
				printf("the start window width is negtive");
				cnv->queue.wobj_num--;
				cnv->queue.cal_wobj = get_next_wobj(cnv->queue.wobj_buff, cnv->queue.cal_wobj);
				rtn= -1;
				break;
			}
		}

		if (wobj_init_pos > cnv->start_win_width)
		{
			printf("the work object is gave up");
			cnv->queue.wobj_num--;
			cnv->queue.cal_wobj = get_next_wobj(cnv->queue.wobj_buff, cnv->queue.cal_wobj);
			rtn = -1;
			break;
		}
		// use the mot vel when work object get to upper limit and same time robot begin moving.
		// robot move time = motion lift + motion approach + motion laydown + motion pick(use no time) + calculate time.
		wobj_x =(wobj_init_pos + cnv->mot_vel_fd*(p_block->trajectory.total_interval / p_block->prf_cart_rot.profile_sample_time + cnv->pyramid.up_down_time+\
			cnv->pyramid.comp_time)*p_block->profile.profile_sample_time) *(1000.0 / cnv->counts_per_meter);
		// set work object's pos in conveyor frame.
		pos_in_cnv[0] = wobj_x;
		pos_in_cnv[1] = cnv->queue.cal_wobj->init_y;
		pos_in_cnv[2] = 0;
		pos_in_cnv[3] = 0;
		pos_in_cnv[4] = 0;
		pos_in_cnv[5] = 0;
		// coordinate transpose from conveyor to robot.
		uP2bP(p_cs, pos_in_cnv, pos_in_rbt);
		
		// tracking in x,y frame axis only.
		if (fabs(pos_in_rbt[1] - p_block->pose.pose_end[1]) < cnv->pyramid.eps_pyr)
		{
			memcpy(cnv->approach_point, p_block->pose.pose_end, 6 * sizeof(double));
			rtn = 1;
			break; // 2 level eps is unuseful,can remove it.
		}
		// if satisfy the second level eps.
		if (fabs(pos_in_rbt[1] - p_block->pose.pose_end[1]) < cnv->pyramid.eps2_pyr)
		{
			memcpy(cnv->approach_point, p_block->pose.pose_end, 6 * sizeof(double));
			rtn = 2;
			break;
		}

		iter_times++;
		// can't find the intercept point
		if (iter_times >= cnv->pyramid.max_iter)
		{
			if (rtn < 1)
			{
				printf("itration time over max time");
				cnv->queue.wobj_num--;
				cnv->queue.cal_wobj = get_next_wobj(cnv->queue.wobj_buff, cnv->queue.cal_wobj);
			}
			break;
		}
		if (rtn < 1)
		{
			p_block->pose.pose_end[1] = (p_block->pose.pose_end[1] + pos_in_rbt[1]) / 2.0;
		}
		else if (rtn==1)
		{
			if (p_block->pose.pose_end[1] < pos_in_rbt[1])
			{
				p_block->pose.pose_end[1] += cnv->pyramid.eps2_pyr;
			}
			else
			{
				p_block->pose.pose_end[1] -= cnv->pyramid.eps2_pyr;
			}
		}

	}
	// set the lay down point.
	memcpy(cnv->laydown_point, p_block->pose.pose_end, 6 * sizeof(double));
	cnv->laydown_point[2] = cnv->laydown_point[2] - cnv->down_height;

	printf("\niteration time %d  \npick up point %lf  %lf  %lf  %lf  %lf  %lf", iter_times, cnv->approach_point[0], cnv->approach_point[1], cnv->approach_point[2], cnv->approach_point[3], cnv->approach_point[4], cnv->approach_point[5]);

	return rtn;
}

int16 InsertInterceptBlocks(trajectory_module* tra,robot_config_module* cfg, conveyor* cnv,double* start_pos,double* end_pos,double* aux_pos)
{
	Uint8 id = 0;
	double BlendDelayArc[5] = { 0,0,0,0,0 };
	double profile[5] = { 100, 100,0.1,100,0.1};
	InsertNewMotionBlock(tra,cfg, start_pos, end_pos, NULL, profile, BlendDelayArc, TRAJECTORY_MODULE_MOTION_TYPE_LIN, id);

	return 0;
}

int16 PIDTrack(conveyor* cnv, cartesian_module* p_cart,joint_module* p_joint, robot_config_module* cfg)
{
	Uint8 i;
	cnv->pid.cur_trck_time++;
	// pos in conveyor
	double pos[6]	= { 0,0,0,0,0,0 };
	pos[0]			= (cnv->mot_pos_fd - cnv->queue.cal_wobj->init_mot_pos - cnv->queue_trck_dist) *(1000 / cnv->counts_per_meter);
	pos[1]			= cnv->queue.trck_wobj->init_y;
	// pos in robot
	uP2bP(&gRobot.robot_dev[0].module.coordinate, pos, pos);

	for (i = 0; i < 2; i++)
	{
		// the differential of error.
		cnv->pid.Derr[i] = pos[i] - p_cart->cart_pos_fd[i] - cnv->pid.err[i];

		// error.
		cnv->pid.err[i] = pos[i] - p_cart->cart_pos_fd[i];
		// check error limit.
		ullimit(cnv->pid.err_max, -cnv->pid.err_max, cnv->pid.err[i]);
		
		// the integral of error.
		cnv->pid.Ierr[i] += cnv->pid.err[i];
		// check the integral of error limit.
		ullimit(cnv->pid.Ierr_max, -cnv->pid.Ierr_max, cnv->pid.Ierr[i]);
	
		// PID adjustment in x,y.
		cnv->pid.m[i] = cnv->pid.Kp*(cnv->pid.err[i] + 1 / cnv->pid.Ti*cnv->pid.Ierr[i] + cnv->pid.Td*cnv->pid.Derr[i]);
		// check adjustment limit.
		ullimit(cnv->pid.m_max, -cnv->pid.m_max, cnv->pid.m[i]);

		p_cart->cart_pos_cmd[i] += cnv->pid.m[i];
	}
	return 0;
}
// this function is running in 1 ms interupt.
int16 conveyor_tracking_mode(conveyor_tracking* p_cnv_trck, robot_config_module* p_config, trajectory_module* m_traj, \
							 robot_coordinate_system* p_cs,cartesian_module* p_cart, joint_module* p_joint,Uint8 id)
{
	extern FILE* fp1;
	extern FILE* fp2;
	extern FILE* fp3;
	extern FILE* fp4;
	extern FILE* fp5;
	extern FILE* fp6;
	extern time cur_time;
	conveyor* cnv				= NULL;
	conveyor_robot* cnv_rbt		= NULL;
	double intercept[3]			= { 0,0,0 };
	double cart1[6]				= { 0,0,0,0,0,0 };
	double cart2[6]				= { 0,0,0,0,0,0 };
	int16 rtn					= 0;
	Uint8 j						= 0;
	cnv							= &p_cnv_trck->conveyor[0];
	cnv_rbt						= &p_cnv_trck->conv_rbt[0];
	wobj* p_cal					= NULL;
	double wobj_pos				= 0; 
	double wobj_x				= 0;
	double tmp					= 0;
	double tmp1					= 0;
	// set work object's pos in conveyor frame.
	double pos_in_cnv[6]		= { 0,0,0,0,0,0 };
	double pos_in_rbt[6]		= { 0,0,0,0,0,0 };
	// rand mum
	double vel					= 0;
	Uint32 randnum				= 0;
	Uint64 seed					= cur_time.cur_ticks;

	// rand num
	srand(seed);
	randnum = rand();
	randnum = randnum%((Uint16)(cnv->mot_vel_cmd*CONVEYOR_VEL_AMPLITUDE));
	vel = cnv->mot_vel_cmd +(double)randnum;
	cnv->mot_pos_cmd += vel / 1000.0;
	//-----------------------------
	//cnv->mot_pos_cmd += cnv->mot_vel_cmd / 1000.0;
	cnv->mot_pos_fd = (int32)cnv->mot_pos_cmd;
	cnv->mot_vel_fd = (int32)cnv->mot_vel_cmd;

	if (cnv->mot_vel_fd < 0)
	{
		cnv->direction = 1;
	}
	else
	{
		cnv->direction = 0;
	}

	fprintf(fp1, "%d\n", cnv->mot_pos_fd);

	// insert work object
	if (cur_time.cur_ticks == 500|| cur_time.cur_ticks == 8500 || cur_time.cur_ticks == 8501 || cur_time.cur_ticks == 8508 || cur_time.cur_ticks == 8512 || cur_time.cur_ticks == 8520)
	{
		QueueIn(&p_cnv_trck->conveyor[0].queue, 0, 0, p_cnv_trck->conveyor[0].mot_pos_fd, p_cnv_trck->conveyor[0].mot_vel_fd, 0, &cur_time);
	}

	switch (cnv->state)
	{
	case 	IDLE:
	{
		break;
	}

	case PICK_READY:
	{
		rtn = PyramidOptimization(cnv, cnv_rbt,p_config, p_cs, intercept);  // simulate robot calculate
		if (rtn > 0)
		{
			printf("\niteratia eps level: %d\n", rtn);
			InsertInterceptBlocks(m_traj, p_config, cnv, cnv->place_point, cnv->lift_point, NULL);
			CalcMotionProfile(m_traj, p_config);
			InsertInterceptBlocks(m_traj, p_config, cnv, cnv->lift_point, cnv->approach_point, NULL);
			CalcMotionProfile(m_traj, p_config);
			//InsertInterceptBlocks(m_traj, p_config, cnv, cnv->approach_point, cnv->laydown_point, NULL);
			//CalcMotionProfile(m_traj, p_config);

			cnv->state = PICK_INTERCEPT;
		}
		break;
	}

	case PICK_INTERCEPT:
	{
		TrajectoryGenerator(m_traj, p_cart, p_joint, p_config, id);
		if (m_traj->current_time != 0)
		{
			fprintf(fp2, "%lf  %lf  %lf  %lf  %lf  %lf\n", p_cart->cart_pos_cmd[0], p_cart->cart_pos_cmd[1], \
				p_cart->cart_pos_cmd[2], p_cart->cart_pos_cmd[3], p_cart->cart_pos_cmd[4], p_cart->cart_pos_cmd[5]);
			fprintf(fp3, "%lf  %lf  %lf  %lf\n", p_joint[0].joint_pos_cmd, p_joint[1].joint_pos_cmd, p_joint[2].joint_pos_cmd, p_joint[3].joint_pos_cmd);

		}
		else
		{
			cnv->intercept_sts++;
			//printf("current time:%d ms\n", cur_time.cur_ticks);
			if (cnv->intercept_sts == INTERCEPT_APPROACH)
			{
				cnv->state = PICK_TRACKING;
				cnv->intercept_sts = INTERCEPT_IDLE;

				// intercept error 
				p_cal = cnv->queue.cal_wobj;
				wobj_pos = cnv->mot_pos_fd - p_cal->init_mot_pos - cnv->queue_trck_dist;
				wobj_x = wobj_pos *(1000 / cnv->counts_per_meter);
				// set work object's pos in conveyor frame.
				pos_in_cnv[0] = wobj_x;
				pos_in_cnv[1] = p_cal->init_y;
				pos_in_cnv[2] = 0;
				pos_in_cnv[3] = 0;
				pos_in_cnv[4] = 0;
				pos_in_cnv[5] = 0;
				// coordinate transpose from conveyor to robot.
				uP2bP(p_cs, pos_in_cnv, pos_in_rbt);
				printf("\n intercept error:%lf\n current time:%d ms\n", p_cart->cart_pos_cmd[1]- pos_in_rbt[1], cur_time.cur_ticks);

				// calculate cam curve
				tmp			= p_cart->cart_pos_cmd[1] - pos_in_rbt[1];  
				cnv->cam.c0 = tmp;  // 机器人相对于工件的位置
				tmp1		= cnv->mot_vel_fd *cnv->pulse_2_mm;
				cnv->cam.c1 = tmp1;
				cnv->cam.c2 = -6 * tmp - 3 * tmp1;
				cnv->cam.c3 = 8 * tmp + 3 * tmp1;
				cnv->cam.c4 = -3 * tmp - tmp1;

				cnv->cam.init_cam_pos = cnv->mot_pos_fd;
			}
		}

		break;
	}
	// 电子凸轮跟踪，机器人位置与工件同步后进行追赶工件，使位置和速度双同步。电子凸轮方法理论上随传送带速度实时变化,\
	// 机器人末速度等于传送带真实速度。
	case PICK_CHASE:
	{
		memcpy(p_cart->cart_pos_fd, p_cart->cart_pos_cmd, p_config->cart_dim * sizeof(double));

		tmp		= fabs(cnv->mot_pos_fd - cnv->cam.init_cam_pos)*cnv->pulse_2_mm /cnv->cam.d;
		// the displacement relate to work object.
		tmp1	= cnv->cam.c0 + cnv->cam.c1*tmp + cnv->cam.c2*pow(tmp, 2) + cnv->cam.c3*pow(tmp, 3) \
				  + cnv->cam.c4*pow(tmp, 4);

		p_cal = cnv->queue.cal_wobj;
		wobj_pos = cnv->mot_pos_fd - p_cal->init_mot_pos - cnv->queue_trck_dist;
		wobj_x = wobj_pos *(1000 / cnv->counts_per_meter);
		// set work object's pos in conveyor frame.
		pos_in_cnv[0] = wobj_x;
		pos_in_cnv[1] = p_cal->init_y;
		pos_in_cnv[2] = 0;
		pos_in_cnv[3] = 0;
		pos_in_cnv[4] = 0;
		pos_in_cnv[5] = 0;
		// coordinate transpose from conveyor to robot.
		uP2bP(p_cs, pos_in_cnv, pos_in_rbt);
		// tcp in robot base = tcp in work object + work object in robot base.
		p_cart->cart_pos_cmd[1] = pos_in_rbt[1] + tmp1; 

		if (fabs(cnv->mot_pos_fd - cnv->cam.init_cam_pos)*cnv->pulse_2_mm > cnv->cam.d)
		{
			cnv->state = PLACE_READY;
		}
		fprintf(fp6, "%lf\n", p_cart->cart_pos_cmd[1] - pos_in_rbt[1]);

		break;
	}

	case PICK_TRACKING:
	{
		memcpy(p_cart->cart_pos_fd, p_cart->cart_pos_cmd, p_config->cart_dim * sizeof(double));
		//p_cart->cart_pos_fd[0] = p_cart->cart_pos_cmd[0]*0.95;
		//p_cart->cart_pos_fd[1] = p_cart->cart_pos_cmd[1] * 0.95;
		//pid
		PIDTrack(cnv, p_cart,p_joint, p_config);
	
		fprintf(fp4,"%lf\n", cnv->pid.err[1]);

		if (fabs(cnv->pid.err[1]) < cnv->pid.eps_pid&&cnv->pid.cur_trck_time>= cnv->pid.prm_trck_time)
		{
			cnv->state = PLACE_READY;
			cnv->pid.cur_trck_time=0;
			//cnv->state = IDLE;
		}
		break;
	}

	case PLACE_READY:
	{
		memcpy(cart1, p_cart->cart_pos_fd, p_config->cart_dim * sizeof(double));
		memcpy(cart2, p_cart->cart_pos_fd, p_config->cart_dim * sizeof(double));
		//cart2[2] += cnv->down_height;
		//InsertInterceptBlocks(m_traj, p_config, cnv, cart1, cart2, NULL);
		//CalcMotionProfile(m_traj, p_config);

		memcpy(cart1, cnv->place_point, p_config->cart_dim * sizeof(double));
		cart1[2] += cnv->down_height;
		InsertInterceptBlocks(m_traj, p_config, cnv, cart2, cart1, NULL);
		CalcMotionProfile(m_traj, p_config);

		InsertInterceptBlocks(m_traj, p_config, cnv, cart1, cnv->place_point, NULL);
		CalcMotionProfile(m_traj, p_config);

		cnv->state = PLACE_INTERCEPT;
		break;
	}

	case PLACE_INTERCEPT:
	{
		TrajectoryGenerator(m_traj, p_cart, p_joint, p_config, id);
		if (m_traj->current_time != 0)
		{
			fprintf(fp5, "%lf  %lf  %lf  %lf  %lf  %lf\n", p_cart->cart_pos_cmd[0], p_cart->cart_pos_cmd[1], \
				p_cart->cart_pos_cmd[2], p_cart->cart_pos_cmd[3], p_cart->cart_pos_cmd[4], p_cart->cart_pos_cmd[5]);
			//fprintf(fp5, "%lf  %lf  %lf  %lf\n", p_joint[0].joint_pos_cmd, p_joint[1].joint_pos_cmd, p_joint[2].joint_pos_cmd, p_joint[3].joint_pos_cmd);

		}
		else
		{
			cnv->intercept_sts++;
			//printf("current time:%d ms\n", cur_time.cur_ticks);
			if (cnv->intercept_sts == INTERCEPT_APPROACH)
			{
				cnv->intercept_sts = INTERCEPT_IDLE;
				cnv->state = PICK_READY;
				//cnv->state = IDLE;
				cnv->queue.wobj_num--;
				cnv->queue.cal_wobj = get_next_wobj(cnv->queue.wobj_buff, cnv->queue.cal_wobj);
				
			}
		}

		break;
	}

	case PLACE_TRACKING:
	{
		break;
	}
	default:
		break;
	}

	return 0;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//计算笛卡尔下规划
//----------------------------------------------------------------------------------------------------
int16 CalcTrajProfile(motion_trajectory* traj, double clock_time, Uint8 counter)
{
	//-------------------------------------------------------
	//计算加速度
	calculate_acceleration_from_profile(clock_time,
		traj->jerk_series[counter],
		traj->acc_series[counter],
		&traj->acc);
	//-------------------------------------------------------
	//计算速度
	calculate_velocity_from_profile(clock_time,
		traj->jerk_series[counter],
		traj->acc_series[counter],
		traj->vel_series[counter],
		&traj->vel);
	//-------------------------------------------------------
	//计算位置
	calculate_position_from_profile(clock_time,
		traj->jerk_series[counter],
		traj->acc_series[counter],
		traj->vel_series[counter],
		traj->pos_series[counter],
		&traj->pos);
	//-------------------------------------------------------
	return 0;
}
//----------------------------------------------------------------------------------------
//笛卡尔下时间缩放方法。对位置和姿态进行计算，因为时间缩放位置规划和姿态的规划的每个时间段不对应，因此分为两个部分分别计算
//----------------------------------------------------------------------------------------
int16 CartAxisSyncTimeScale(trajectory_module* p_trajectory, motion_block* p_motion, robot_config_module* p_config)
{
	Uint8 counter = 0;
	Uint8 i		  = 0;
	//-------------------------------------------------------------------
	double clock_time = 0.0;
	//-------------------------------------------------------------------
	//位置
	//-------------------------------------------------------------------
	for (i = 0; i < p_config->cart_trans_dim+1; i++)
	{
		for (counter = 0; counter < 7; counter++)
		{
			if ((p_trajectory->current_time > p_motion->traj_cart_xyz_poseture[i].time_series[counter]) && \
				(p_trajectory->current_time <= p_motion->traj_cart_xyz_poseture[i].time_series[counter + 1]))
			{
				/* determine the time interval,and calculate the states in the time interval */
				clock_time = p_trajectory->current_time - p_motion->traj_cart_xyz_poseture[i].time_series[counter];
				clock_time *= p_trajectory->period_time;

				//-------------------------------------------------------------
				//计算当前的平移xyz规划
				//-------------------------------------------------------------
				if (i == p_config->cart_trans_dim && p_motion->pose.axis_angle_flag != 1)
				{
					p_motion->traj_cart_xyz_poseture[i].acc = 0;
					p_motion->traj_cart_xyz_poseture[i].vel = 0;
					p_motion->traj_cart_xyz_poseture[i].pos = 0;
				}
				else
				{
					CalcTrajProfile(&(p_motion->traj_cart_xyz_poseture[i]), clock_time, counter);			
				}

				break;
			}
		}
	}
	return 0;
}

// 传送带上机器人的运动轨迹产生。每个运动块有始末速度，在机器人运动过程中叠加传送带与预期位置的误差，使得机器人末位置和速度与工件一致。
int16 cnv_rbt_traj_gen(trajectory_module* p_trajectory, cartesian_module* p_cart, joint_module* p_joint, robot_config_module* p_config,double* err, Uint8 id)
{
	Uint8 i					= 0;
	double cart[6]			= { 0,0,0,0,0,0 };
	double cart_vel[6]		= { 0,0,0,0,0,0 };
	Uint8 pose_id			= p_config->cart_trans_dim;
	//---------------------------------------------------------------------
	//指向执行的运动块
	//---------------------------------------------------------------------
	motion_block* 			p_motion = p_trajectory->motion_block_execute;
	motion_block* 			p_next_motion = p_trajectory->motion_block_execute;
	motion_block* 			p_last_motion = p_trajectory->motion_block_execute;

	//-------------------------------------------------------------------
	// 假如当前指向的执行运动块不是可以运行的状态，那么直接返回
	//-------------------------------------------------------------------
	if (p_motion->state < TRAJECTORY_MODULE_MOTION_BLOCK_STATE_RUNNING)
	{
		//if (gRobot.robot_dev[id].mode.sct.stop_script_flag == true)
		//{
		//	gRobot.robot_dev[id].mode.sct.stop_script_flag = false;
		//}
		//// 清除运动块个数
		//gRobot.robot_dev[id].mode.rmacro.block_nr = 0;
		//设置状态
		gRobot.robot_dev[id].status = ROBOT_DEV_STS_STOP;
		return 0;
	}
	gRobot.robot_dev[id].status = ROBOT_DEV_STS_RUN;

	p_next_motion = get_next_motion_block(p_trajectory->motion_block_buffer, p_next_motion);
	p_last_motion = get_last_motion_block(p_trajectory->motion_block_buffer, p_last_motion);

	p_trajectory->current_time++;
	//p_motion->state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_RUNNING;

	//-----------------------------------------------------------------
	// 轨迹生成模块，根据不同的运动类型，进行不同的轨迹计算
	//-----------------------------------------------------------------
	switch (p_motion->motion_type)
	{
	case TRAJECTORY_MODULE_MOTION_TYPE_LIN: // line motion in Cartesian
	{
		CartAxisSyncTimeScale(p_trajectory, p_motion, p_config);

		for (i = 0; i < p_config->cart_trans_dim; i++)
		{
			cart[i]			= p_motion->traj_cart_xyz_poseture[i].pos;
			cart_vel[i]		= p_motion->traj_cart_xyz_poseture[i].vel;
		}
		//----------------------------------------------------------------------------------------------------
		//计算当前的姿态,从轴角转化成固定角
		//----------------------------------------------------------------------------------------------------
		if (p_motion->pose.axis_angle_flag == 1)
		{
			//angle-axis convert to rotate matrix
			matxx_axisAngl2Rotm(p_motion->traj_cart_xyz_poseture[pose_id].pos,		//angle in rad
				&p_motion->pose.axis_vector,		//axis
				&p_motion->pose.rotation_matxx);

			/*  B=R*A */
			matxx_multiply(&p_motion->pose.rotation_matxx, &p_motion->pose.rot_start, &p_motion->pose.rotation_matxx);

			// 旋转矩阵转化为固定角
			r2xyz(&p_motion->pose.rotation_matxx, &cart[3], 1);
			//-------------------------------------------------------------------------------------------
			//decouple axis to xyz to calculate the vel
			matxx_transpose(&p_motion->pose.rot_start, &p_motion->pose.rot_tmp);

			//convert the axis to the base
			matxx_multiply(&p_motion->pose.rot_tmp, &p_motion->pose.axis_vector, &p_motion->pose.tmp31);

			// vel map to the axis
			matxx_k_mult(p_motion->traj_cart_xyz_poseture[pose_id].vel, &p_motion->pose.tmp31);

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
		extern FILE* cnv_fp1;
		extern FILE* cnv_fp2;
		fprintf(cnv_fp1, "%lf  %lf  %lf  %lf  %lf  %lf\n", cart[0], cart[1], cart[2], cart[3], cart[4], cart[5]);
		fprintf(cnv_fp2, "%lf  %lf  %lf  %lf  %lf  %lf\n", cart_vel[0], cart_vel[1], cart_vel[2], cart_vel[3], cart_vel[4], cart_vel[5]);

		if (p_trajectory->current_time >= p_motion->traj_cart_xyz_poseture[0].time_series[7])
		{
			p_trajectory->current_time = 0;
			p_motion->state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_IDL;
			p_trajectory->motion_block_execute = p_next_motion;
		}
		break;
	}
	default:
		break;
	}
	return 0;
}