
#include "robot_coordinate_system.h"
#include "stdio.h"

int16 test_bPt2bPf(robot_coordinate_system* p_robotcs)
{
	double pose_in[6];
	double pose_out[6];

	pose_in[0] = 0.375;
	pose_in[1] = 0;
	pose_in[2] = 0.2;
	pose_in[3] = 180;
	pose_in[4] = 0;
	pose_in[5] = 180;

	bPt2bPf(p_robotcs, pose_in, pose_out);

	printf("\n%lf  %lf  %lf  %lf  %lf  %lf\n", pose_out[0], pose_out[1], pose_out[2], pose_out[3], pose_out[4], pose_out[5]);
	return 0;
}
int16 test_tP2bPf(robot_coordinate_system* p_robotcs)
{
	double pose_in[6];
	double pose_out[6];

	pose_in[0] = 0;
	pose_in[1] = 0;
	pose_in[2] = 0.1;
	pose_in[3] = 0;
	pose_in[4] = 0;
	pose_in[5] = 0;

	p_robotcs->axis[0].q = 0;
	p_robotcs->axis[1].q = 0;
	p_robotcs->axis[2].q = 0;
	p_robotcs->axis[3].q = 0;

	tP2bPf(p_robotcs, pose_in, pose_out);

	printf("\n%lf  %lf  %lf  %lf  %lf  %lf\n", pose_out[0], pose_out[1], pose_out[2], pose_out[3], pose_out[4], pose_out[5]);
	return 0;
}
int16 test_uP2bPf(robot_coordinate_system* p_robotcs)
{
	double pose_in[6] = { 1,1,0.2,0,0,0 };
	double pose_out[6];
	uP2bPf(p_robotcs, pose_in, pose_out);
	printf("\n%lf  %lf  %lf  %lf  %lf  %lf\n", pose_out[0], pose_out[1], pose_out[2], pose_out[3], pose_out[4], pose_out[5]);

	return 0;
}

int16 test_tcp_calibration(robot_coordinate_system* p_robotcs)
{
	// in joint
	p_robotcs->toolc.p[0][0]		= 0;
	p_robotcs->toolc.p[0][1]		= -0;
	p_robotcs->toolc.p[0][2]		= 0.1;
	p_robotcs->toolc.p[0][3]		= 0;

	p_robotcs->toolc.p[1][0]		= 0;
	p_robotcs->toolc.p[1][1]		= 0;
	p_robotcs->toolc.p[1][2]		= 0;
	p_robotcs->toolc.p[1][3]		= 0;

	p_robotcs->toolc.p[2][0]		= 56.196192699999997;
	p_robotcs->toolc.p[2][1]		= 104.477512-180;
	p_robotcs->toolc.p[2][2]		= 0;
	p_robotcs->toolc.p[2][3]		= 180-(90-18.4349488)- 37.7612439;

	p_robotcs->toolc.p[3][0]		= 60;
	p_robotcs->toolc.p[3][1]		= -120;
	p_robotcs->toolc.p[3][2]		= 0;
	p_robotcs->toolc.p[3][3]		= 120;

	p_robotcs->toolc.p[4][0]		= 60;
	p_robotcs->toolc.p[4][1]		= -120;
	p_robotcs->toolc.p[4][2]		= 0;
	p_robotcs->toolc.p[4][3]		= -60;
			
	p_robotcs->toolc.p[5][0]		= 60;
	p_robotcs->toolc.p[5][1]		= -120;
	p_robotcs->toolc.p[5][2]		= 0.1;
	p_robotcs->toolc.p[5][3]		= -60;
	// in cartesian relative base.
	
	p_robotcs->userc.p[0][0] = 0;
	p_robotcs->userc.p[0][1] = 0;
	p_robotcs->userc.p[0][2] = 0.1;
	p_robotcs->userc.p[0][3] = 0;

	p_robotcs->userc.p[1][0] = 0;
	p_robotcs->userc.p[1][1] = 0;
	p_robotcs->userc.p[1][2] = 0.1;
	p_robotcs->userc.p[1][3] = 90;

	p_robotcs->userc.p[2][0] = 0;
	p_robotcs->userc.p[2][1] = 0;
	p_robotcs->userc.p[2][2] = 0.2;
	p_robotcs->userc.p[2][3] = 90;
	

	//TcpCalibration(p_robotcs);
	ScaraTcpCalibration(p_robotcs);
	printf("tcp  ");
	matxx_printf(&p_robotcs->toolc.tcp);
	TcfCalibration(p_robotcs);
	printf("tcf  ");
	matxx_printf(&p_robotcs->toolc.tcf);

	test_tP2bPf(p_robotcs);
	printf("\n ========user=========\n");
	// user coordinate calibration
	UserCalibration(p_robotcs);
	matxx_printf(&p_robotcs->user[p_robotcs->user_id].R);
	printf("user's origin :%lf  %lf  %lf\n", p_robotcs->user[p_robotcs->user_id].origin[0], p_robotcs->user[p_robotcs->user_id].origin[1], p_robotcs->user[p_robotcs->user_id].origin[2]);
	//matxx_printf(&p_robotcs->user[p_robotcs->user_id].R);
	//matxx_printf(&p_robotcs->user[p_robotcs->user_id].origin);

	test_uP2bPf(p_robotcs);

	return 0;
}


//robot_coordinate_system		robot_cs;
void main_coord()
{
	InitRobotCoordinateSystem(&robot_cs);

	test_tcp_calibration(&robot_cs);

	return;
}


