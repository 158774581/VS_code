#include "stdio.h"
#include "stdlib.h"
#include "conveyor_tracking.h"
#include "process.h"
#include "windows.h"

#include "robot_app.h"

//conveyor_tracking			cnv_trck;
time						cur_time;
//robot_config_module			cfg; //in gRobot.

Uint32						run_time = 0;  //Unit ms

FILE* fp1;
FILE* fp2;
FILE* fp3;
FILE* fp4;
FILE* fp5;
FILE* fp6;
FILE* fp7;

extern int16 test_tP2bPf(robot_coordinate_system* p_robotcs);
extern int16 test_uP2bPf(robot_coordinate_system* p_robotcs);

int16 test_cnv_calibration(robot_coordinate_system* p_robotcs)
{
	// in joint
	p_robotcs->toolc.p[0][0] = 0;
	p_robotcs->toolc.p[0][1] = -0;
	p_robotcs->toolc.p[0][2] = 100;
	p_robotcs->toolc.p[0][3] = 0;

	p_robotcs->toolc.p[1][0] = 0;
	p_robotcs->toolc.p[1][1] = 0;
	p_robotcs->toolc.p[1][2] = 0;
	p_robotcs->toolc.p[1][3] = 0;

	p_robotcs->toolc.p[2][0] = 0;
	p_robotcs->toolc.p[2][1] = 0;
	p_robotcs->toolc.p[2][2] = 0;
	p_robotcs->toolc.p[2][3] = 0;

	p_robotcs->toolc.p[3][0] = 0;
	p_robotcs->toolc.p[3][1] = 0;
	p_robotcs->toolc.p[3][2] = 0;
	p_robotcs->toolc.p[3][3] = 0;

	p_robotcs->toolc.p[4][0] = 60;
	p_robotcs->toolc.p[4][1] = -120;
	p_robotcs->toolc.p[4][2] = 0;
	p_robotcs->toolc.p[4][3] = -60;

	p_robotcs->toolc.p[5][0] = 60;
	p_robotcs->toolc.p[5][1] = -120;
	p_robotcs->toolc.p[5][2] = 100;
	p_robotcs->toolc.p[5][3] = -60;
	// in cartesian relative base.

	p_robotcs->userc.p[0][0] = atan(4.0/3.0)/ MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	p_robotcs->userc.p[0][1] = 0;
	p_robotcs->userc.p[0][2] = 120;
	p_robotcs->userc.p[0][3] = 0;

	p_robotcs->userc.p[1][0] = -atan(4.0 / 3.0)/ MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	p_robotcs->userc.p[1][1] = 0;
	p_robotcs->userc.p[1][2] = 120;
	p_robotcs->userc.p[1][3] = 90;

	p_robotcs->userc.p[2][0] = -atan(4.0 / 3.0) / MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	p_robotcs->userc.p[2][1] = 0;
	p_robotcs->userc.p[2][2] = 0;
	p_robotcs->userc.p[2][3] = 90;


	//TcpCalibration(p_robotcs);
	ScaraTcpCalibration(p_robotcs);
	printf("tcp  ");
	matxx_printf(&p_robotcs->toolc.tcp);

	TcfCalibration(p_robotcs);
	printf("tcf  ");
	matxx_printf(&p_robotcs->toolc.tcf);

	FinB(p_robotcs, p_robotcs->toolc.p[1]);
	printf("FinB  ");
	matxx_printf(&p_robotcs->coor_t.T);

	TinB(p_robotcs, p_robotcs->toolc.p[4]);
	printf("TinB  ");
	matxx_printf(&p_robotcs->coor_t.T);

	printf("\n ========user=========\n");
	// user coordinate calibration
	UserCalibration(p_robotcs);
	printf("user's pose :");
	matxx_printf(&p_robotcs->user[p_robotcs->user_id].R);
	printf("user's origin :%lf  %lf  %lf\n", p_robotcs->user[p_robotcs->user_id].origin[0], p_robotcs->user[p_robotcs->user_id].origin[1], p_robotcs->user[p_robotcs->user_id].origin[2]);
	//matxx_printf(&p_robotcs->user[p_robotcs->user_id].R);
	//matxx_printf(&p_robotcs->user[p_robotcs->user_id].origin);

	//test_uP2bPf(p_robotcs);

	return 0;
}
void printf_time(FILE* fp, time* cur_time)
{
	fprintf(fp, "%d  %d  %d  %d  %d  %d  %d\n ", cur_time->year, cur_time->month, cur_time->day, cur_time->hour, cur_time->minute, cur_time->second, cur_time->ticks);
}

void td_time(time* cur_time)
{
	//while (1)
	{
		//Sleep(10);
		cur_time->ticks = cur_time->ticks+10;

		if (cur_time->ticks >= 1000) { cur_time->second++; cur_time->ticks = 0; }
		if (cur_time->second >= 60) { cur_time->minute++; cur_time->second = 0; }
		if (cur_time->minute >= 60) { cur_time->hour++; cur_time->minute = 0; }
		if (cur_time->hour >= 24) { cur_time->day++; cur_time->hour = 0; }
		if (cur_time->day >= 365) { cur_time->year++; cur_time->day = 0; }
	}
	return;
}
double time2double(time* cur_time)
{
	double time64	= 0;
	time64 = cur_time->ticks + cur_time->second*1E3 + cur_time->minute*1E5 + cur_time->hour*1E7 + cur_time->day*1E9 + cur_time->month*1E12 + cur_time->year*1E14;
	return time64;
}
void td_conveyor(conveyor* cnv)
{
	Uint32 randnum;
	fp1 = fopen("C://Users//hqi//Desktop//debugging_data//conveyor_pos.txt", "w+");
	while (1)
	{
		Sleep(10);
		td_time(&cur_time);
		// rand num to set conveyor vel
		Uint64 seed = ((Uint64)time2double(&cur_time));
		srand(seed);
		randnum = rand();
		//randnum = randnum%CONVEYOR_VEL_AMPLITUDE;

		cnv->mot_pos_fd += cnv->mot_vel_fd / 100;// +randnum % 2;
		fprintf(fp1, "%d\n", cnv->mot_pos_fd);
		//fprintf(fp1, "%d\n", randnum);
	}
	fclose(fp1);
	return;
}
void td_cal_pyramid_optimization()
{
	double intercept[3];
	int8 rtn = 0;
	conveyor* cnv = &gRobot.robot_dev[0].cnv_trck.conveyor[0];
	robot_config_module* p_config = &gRobot.robot_dev[0].cfg;
	trajectory_module* m_traj = &gRobot.robot_dev[0].module.trajectory;
	robot_coordinate_system* p_cs = &gRobot.robot_dev[0].module.coordinate;

	CalUpDownTime(&gRobot.robot_dev[0].cnv_trck.conveyor[0], &gRobot.robot_dev[0].cnv_trck.conv_rbt[0], &gRobot.robot_dev[0].cfg, gRobot.robot_dev[0].cnv_trck.conveyor[0].down_height, gRobot.robot_dev[0].cnv_trck.conveyor[0].down_height);

	while (1)
	{
		Sleep(10);
		rtn=PyramidOptimization(&gRobot.robot_dev[0].cnv_trck.conveyor[0], &gRobot.robot_dev[0].cnv_trck.conv_rbt[0], &gRobot.robot_dev[0].cfg, p_cs, intercept);  // simulate robot calculate
		if (rtn == 0)
		{
			InsertInterceptBlocks(m_traj, p_config, cnv, cnv->place_point, cnv->lift_point, NULL);
			InsertInterceptBlocks(m_traj, p_config, cnv, cnv->lift_point, cnv->approach_point, NULL);
			InsertInterceptBlocks(m_traj, p_config, cnv, cnv->approach_point, cnv->laydown_point, NULL);
		}
		// wait robot get the intercepte point ,then begin PID
		// PIDTrack()
	}
	return;
}

int16 td_CalcMotionProfile(Uint8 id)
{
	robot_config_module* p_config = &gRobot.robot_dev[0].cfg;
	trajectory_module* m_traj = &gRobot.robot_dev[0].module.trajectory;

	while (1)
	{
		Sleep(10);
		CalcMotionProfile(m_traj, p_config);
	}
	return 0;
}
int16 td_TrajectoryGenerator(Uint8 id)
{
	Uint8 write_flag = 0;
	cartesian_module* p_cart = &gRobot.robot_dev[id].module.cart;
	joint_module* p_joint = &gRobot.robot_dev[0].module.joint[0];
	robot_config_module* p_config = &gRobot.robot_dev[0].cfg;
	trajectory_module* m_traj= &gRobot.robot_dev[0].module.trajectory;

	fp2 = fopen("C://Users//hqi//Desktop//debugging_data//cart_cmd.txt", "w+");
	fp3 = fopen("C://Users//hqi//Desktop//debugging_data//joint_cmd.txt", "w+");
	while (1)
	{
		//Sleep(1);
		TrajectoryGenerator(m_traj,p_cart,p_joint,p_config,id);
		if (m_traj->current_time != 0)
		{
			fprintf(fp2, "%lf  %lf  %lf  %lf  %lf  %lf\n", p_cart->cart_pos_cmd[0], p_cart->cart_pos_cmd[1], \
				p_cart->cart_pos_cmd[2], p_cart->cart_pos_cmd[3], p_cart->cart_pos_cmd[4], p_cart->cart_pos_cmd[5]);
			fprintf(fp3, "%lf  %lf  %lf  %lf\n", p_joint[0].joint_pos_cmd, p_joint[1].joint_pos_cmd, p_joint[2].joint_pos_cmd, p_joint[3].joint_pos_cmd);
		}
	}
	fclose(fp2);
	fclose(fp3);
	return 0;
}

//void main()
//{
//	Uint8 id = 0;
//
//	InitRobot(&gRobot);
//
//	conveyor_tracking* p_cnv_trck = &gRobot.robot_dev[0].cnv_trck;
//	// coordinate system.
//	//InitRobotCoordinateSystem(&gRobot.robot_dev[0].module.coordinate);
//	test_cnv_calibration(&gRobot.robot_dev[0].module.coordinate);
//
//	// init conveyor tracking
//	//InitConveyorTracking(&gRobot.robot_dev[0].cnv_trck,id);
//
//	// time init
//	cur_time.year = 2018;
//	cur_time.month = 1;
//	cur_time.day = 1;
//	cur_time.hour = 0;
//	cur_time.minute = 0;
//	cur_time.second = 0;
//	cur_time.ticks = 0;
//
//	// conveyor prm
//	p_cnv_trck->conveyor[0].mot_pos_fd = 0;
//	p_cnv_trck->conveyor[0].mot_vel_fd = CONVEYOR_VEL;
//	p_cnv_trck->conveyor[0].mot_acc_fd = 0;
//	p_cnv_trck->conveyor[0].counts_per_meter = CONVEYOR_COUNTER_PER_METER;
//	p_cnv_trck->conveyor[0].queue_trck_dist = 2000;  // 0.2m
//	// conveyor 0 thread begining...
//	_beginthread(td_conveyor, 0, &p_cnv_trck->conveyor[0]);
//
//	// robot prm
//	p_cnv_trck->conveyor[0].down_height = 100;  // mm
//	p_cnv_trck->conveyor[0].place_point[0] = 100;// Unit mm
//	p_cnv_trck->conveyor[0].place_point[1] = -400;// Unit mm
//	p_cnv_trck->conveyor[0].place_point[2] = 100;// Unit mm
//	p_cnv_trck->conveyor[0].place_point[3] = 0;// Unit deg
//	p_cnv_trck->conveyor[0].place_point[4] = 0;// Unit deg
//	p_cnv_trck->conveyor[0].place_point[5] = 0;// Unit deg
//	p_cnv_trck->conveyor[0].down_height = 100;  // mm
//	  // lift point=place point z +down_height
//	p_cnv_trck->conveyor[0].lift_point[0] = 100;// Unit mm
//	p_cnv_trck->conveyor[0].lift_point[1] = -400;// Unit mm
//	p_cnv_trck->conveyor[0].lift_point[2] = 100+p_cnv_trck->conveyor[0].down_height;// Unit mm
//	p_cnv_trck->conveyor[0].lift_point[3] = 0;// Unit deg
//	p_cnv_trck->conveyor[0].lift_point[4] = 0;// Unit deg
//	p_cnv_trck->conveyor[0].lift_point[5] = 0;// Unit deg
//
//	p_cnv_trck->conveyor[0].U2[0]			= 0;
//	p_cnv_trck->conveyor[0].U2[1]			= 0;
//	p_cnv_trck->conveyor[0].L2[0]			= 800;
//	p_cnv_trck->conveyor[0].L2[1]			= 0;
//
//	// robot 0 calculate thread begining...
//	_beginthread(td_cal_pyramid_optimization, 0, NULL);
//
//	// motion paramenter calculate thread begining...
//	_beginthread(td_CalcMotionProfile, 0, id);
//
//	// trajectory generator thread begining...
//	_beginthread(td_TrajectoryGenerator, 0,id);
//	
//	// insert work object
//	Sleep(500);  //0.5s
//	QueueIn(&p_cnv_trck->conveyor[0].queue, 0, 0, p_cnv_trck->conveyor[0].mot_pos_fd, p_cnv_trck->conveyor[0].mot_vel_fd, 0, &cur_time);
//	Sleep(1500);  //1.5s
//	QueueIn(&p_cnv_trck->conveyor[0].queue, 0, 0, p_cnv_trck->conveyor[0].mot_pos_fd, p_cnv_trck->conveyor[0].mot_vel_fd, 0, &cur_time);
//	Sleep(6000);  //1.5s
//	QueueIn(&p_cnv_trck->conveyor[0].queue, 0, 0, p_cnv_trck->conveyor[0].mot_pos_fd, p_cnv_trck->conveyor[0].mot_vel_fd,0, &cur_time);
//	Sleep(5600);  //1.5s
//
//	QueueIn(&p_cnv_trck->conveyor[0].queue, 0, 0, p_cnv_trck->conveyor[0].mot_pos_fd, p_cnv_trck->conveyor[0].mot_vel_fd, 0, &cur_time);
//	Sleep(7000);  //1.5s
//	QueueIn(&p_cnv_trck->conveyor[0].queue, 0, 0, p_cnv_trck->conveyor[0].mot_pos_fd, p_cnv_trck->conveyor[0].mot_vel_fd, 0, &cur_time);
//	Sleep(7700);  //1.5s
//	QueueIn(&p_cnv_trck->conveyor[0].queue, 0, 0, p_cnv_trck->conveyor[0].mot_pos_fd, p_cnv_trck->conveyor[0].mot_vel_fd, 0, &cur_time);
//	Sleep(8000);  //1.5s
//	QueueIn(&p_cnv_trck->conveyor[0].queue, 0, 0, p_cnv_trck->conveyor[0].mot_pos_fd, p_cnv_trck->conveyor[0].mot_vel_fd, 0, &cur_time);
//	Sleep(5800);  //1.5s
//	QueueIn(&p_cnv_trck->conveyor[0].queue, 0, 0, p_cnv_trck->conveyor[0].mot_pos_fd, p_cnv_trck->conveyor[0].mot_vel_fd, 0, &cur_time);
//	Sleep(3000);  //1.5s
//	QueueIn(&p_cnv_trck->conveyor[0].queue, 0, 0, p_cnv_trck->conveyor[0].mot_pos_fd, p_cnv_trck->conveyor[0].mot_vel_fd, 0, &cur_time);
//	Sleep(9000);  //1.5s
//	QueueIn(&p_cnv_trck->conveyor[0].queue, 0, 0, p_cnv_trck->conveyor[0].mot_pos_fd, p_cnv_trck->conveyor[0].mot_vel_fd, 0, &cur_time);
//	Sleep(7300);  //1.5s
//	QueueIn(&p_cnv_trck->conveyor[0].queue, 0, 0, p_cnv_trck->conveyor[0].mot_pos_fd, p_cnv_trck->conveyor[0].mot_vel_fd, 0, &cur_time);
//	Sleep(8000); //50s
//	return;
//}
void main()
{
	Uint8 id = 0;
	InitRobot(&gRobot);

	conveyor_tracking* p_cnv_trck = &gRobot.robot_dev[0].cnv_trck;
	cartesian_module* p_cart = &gRobot.robot_dev[0].module.cart;
	joint_module* p_joint = gRobot.robot_dev[0].module.joint;
	robot_config_module* p_config = &gRobot.robot_dev[0].cfg;
	trajectory_module* m_traj = &gRobot.robot_dev[0].module.trajectory;
	robot_coordinate_system* p_cs = &gRobot.robot_dev[0].module.coordinate;

	test_cnv_calibration(p_cs);

	// time init
	cur_time.year = 2018;
	cur_time.month = 1;
	cur_time.day = 1;
	cur_time.hour = 0;
	cur_time.minute = 0;
	cur_time.second = 0;
	cur_time.ticks = 0;
	cur_time.cur_ticks = 0;
	// conveyor prm
	p_cnv_trck->conveyor[0].mot_pos_fd = 0;
	p_cnv_trck->conveyor[0].mot_vel_fd = 0;
	p_cnv_trck->conveyor[0].mot_acc_fd = 0;
	p_cnv_trck->conveyor[0].counts_per_meter = CONVEYOR_COUNTER_PER_METER;
	p_cnv_trck->conveyor[0].queue_trck_dist = 2000;  // 0.2m
		// up down prm
	p_cnv_trck->conveyor[0].down_height = 20;  // mm
	p_cnv_trck->conveyor[0].place_point[0] = 100;// Unit mm
	p_cnv_trck->conveyor[0].place_point[1] = -400;// Unit mm
	p_cnv_trck->conveyor[0].place_point[2] = 100;// Unit mm
	p_cnv_trck->conveyor[0].place_point[3] = 0;// Unit deg
	p_cnv_trck->conveyor[0].place_point[4] = 0;// Unit deg
	p_cnv_trck->conveyor[0].place_point[5] = 0;// Unit deg
	p_cnv_trck->conveyor[0].down_height = 20;  // mm
	  // lift point=place point z +down_height
	p_cnv_trck->conveyor[0].lift_point[0] = 100;// Unit mm
	p_cnv_trck->conveyor[0].lift_point[1] = -400;// Unit mm
	p_cnv_trck->conveyor[0].lift_point[2] = 100 + p_cnv_trck->conveyor[0].down_height;// Unit mm
	p_cnv_trck->conveyor[0].lift_point[3] = 0;// Unit deg
	p_cnv_trck->conveyor[0].lift_point[4] = 0;// Unit deg
	p_cnv_trck->conveyor[0].lift_point[5] = 0;// Unit deg

	p_cnv_trck->conveyor[0].U2[0] = 0;
	p_cnv_trck->conveyor[0].U2[1] = 0;
	p_cnv_trck->conveyor[0].L2[0] = 800;
	p_cnv_trck->conveyor[0].L2[1] = 0;

	fp1 = fopen("C://Users//hqi//Desktop//debugging_data//conveyor_pos.txt", "w+");
	fp2 = fopen("C://Users//hqi//Desktop//debugging_data//cart_cmd.txt", "w+");
	fp5 = fopen("C://Users//hqi//Desktop//debugging_data//cart_cmd_place.txt", "w+");
	fp3 = fopen("C://Users//hqi//Desktop//debugging_data//joint_cmd.txt", "w+");
	fp4 = fopen("C://Users//hqi//Desktop//debugging_data//PID_error.txt", "w+");
	fp6 = fopen("C://Users//hqi//Desktop//debugging_data//cam_cart_cmd.txt", "w+");

	p_cnv_trck->conveyor[0].state = PICK_READY;
	p_cnv_trck->conveyor[0].intercept_sts = INTERCEPT_IDLE;
	for (cur_time.cur_ticks = 0; cur_time.cur_ticks < 200000; cur_time.cur_ticks += CNV_TRCK_SIMPLE_TIME)
	{
		conveyor_tracking_mode(p_cnv_trck, p_config, m_traj, p_cs, p_cart, p_joint, 0);
	}

	return;
}

//double cart[6] = { 100,-400,0,0,0,0 };
//double joint_left[6]; double joint_right[6];
//gRobot.robot_dev[id].cfg.prm.location_config = KINEMATICS_LIBRARY_CONFIG_LEFTY;
//gRobot.robot_dev[id].module.kinematics.pfIK(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart, joint_left, 1);
//gRobot.robot_dev[id].cfg.prm.location_config = KINEMATICS_LIBRARY_CONFIG_RIGHTY;
//gRobot.robot_dev[id].module.kinematics.pfIK(p_cart, p_joint, &gRobot.robot_dev[id].module.kinematics, p_config, cart, joint_right, 1);