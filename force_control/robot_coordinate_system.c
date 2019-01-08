/*
* robot_coordinate_system.c
*
*  Created on: Oct 17, 2018
*      Author: hqi
*/

#include	"robot_coordinate_system.h"

int16 InitRobotCoordinateSystem(robot_coordinate_system* p_robotcs)
{
	Uint8 i;
	// scara
	p_robotcs->robot_type	= 4;
	p_robotcs->user_id		= 0;
	p_robotcs->tool_id		= 0;

	for (i = 0; i<AXIS_COORDINATE_NUM; i++)
	{
		InitAxisCoordinate(&p_robotcs->axis[i]);
	}
	p_robotcs->axis[2].type			= 1;
	p_robotcs->axis[0].q			= 0;
	p_robotcs->axis[1].q			= 30;
	p_robotcs->axis[2].q			= 0;
	p_robotcs->axis[3].q			= 30;

	// when all joint var(q)=0
	p_robotcs->axis[0].a			= 250;
	p_robotcs->axis[0].alpha		= 0;
	p_robotcs->axis[0].d			= 300;
	p_robotcs->axis[0].theta		= 0;  // deg
	p_robotcs->axis[1].a			= 250;
	p_robotcs->axis[1].alpha		= 0;
	p_robotcs->axis[1].d			= 0.0000000001;
	p_robotcs->axis[1].theta		= 0;  // deg
	p_robotcs->axis[2].a			= 0;
	p_robotcs->axis[2].alpha		= 180;  // deg
	p_robotcs->axis[2].d			= -80;  // m
	p_robotcs->axis[2].theta		= 0;
	p_robotcs->axis[3].a			= 0;
	p_robotcs->axis[3].alpha		= 0;
	p_robotcs->axis[3].d			= 0;
	p_robotcs->axis[3].theta		= 0;  // rad
	//--------------------------------------------------
	for (i = 0; i<TOOL_COORDINATE_NUM; i++)
	{
		InitToolCoordinate(&p_robotcs->tool[i]);
	}

	for (i = 0; i < USER_COORDINATE_NUM; i++)
	{
		InitUserCoordinate(&p_robotcs->user[i]);
	}

	InitCoordinateTrans(&p_robotcs->coor_t);

	for (i = 0; i<4; i++)
	{
		matxx_malloc(&p_robotcs->toolc.R[i], 3, 3);
		matxx_malloc(&p_robotcs->toolc.d[i], 3, 1);
	}

	matxx_malloc(&p_robotcs->toolc.m33, 3, 3);
	matxx_malloc(&p_robotcs->toolc.m31, 3, 1);
	matxx_malloc(&p_robotcs->toolc.m39, 3, 9);
	matxx_malloc(&p_robotcs->toolc.m91, 9, 1);
	matxx_malloc(&p_robotcs->toolc.m91_, 9, 1);
	matxx_malloc(&p_robotcs->toolc.m93, 9, 3);
	matxx_malloc(&p_robotcs->toolc.m93_, 9, 3);
	matxx_malloc(&p_robotcs->toolc.tcp, 3, 1);
	matxx_malloc(&p_robotcs->toolc.tcf, 3, 3);
	matxx_malloc(&p_robotcs->toolc.tmp33, 3, 3);
	matxx_malloc(&p_robotcs->toolc.m44, 4, 4);
	matxx_malloc(&p_robotcs->toolc.m44_, 4, 4);

	matxx_malloc(&p_robotcs->userc.d, 3, 1);
	matxx_malloc(&p_robotcs->userc.m31, 3, 1);
	matxx_malloc(&p_robotcs->userc.m31_, 3, 1);
	matxx_malloc(&p_robotcs->userc.R, 3, 3);


	p_robotcs->pfTcpCalibration					= TcpCalibration;
	p_robotcs->pfScaraTcpCalibration = ScaraTcpCalibration;
	p_robotcs->pfTcfCalibration = TcfCalibration;
	p_robotcs->pfUserCalibration = UserCalibration;

	p_robotcs->pfTinB = TinB;
	p_robotcs->pfFinB = FinB;
	p_robotcs->pfbPt2bPf = bPt2bPf;
	p_robotcs->pftP2bPf = tP2bPf;
	p_robotcs->pfuP2bPf = uP2bPf;
	
	p_robotcs->pfInitRobotCoordinateSystem = InitRobotCoordinateSystem;

	return 0;
}

int16 InitAxisCoordinate(axis_coordinate* p_axis)
{
	p_axis->type = 0;
	p_axis->q = 0;
	p_axis->a = 0;
	p_axis->alpha = 0;
	p_axis->d = 0;
	p_axis->theta = 0;
	return 0;

}

int16 InitFlangeCoordinate(flange_coordinate* p_flange)
{
	p_flange->origin[0] = 0;
	p_flange->origin[1] = 0;
	p_flange->origin[2] = 0;
	p_flange->xyz[0] = 0;
	p_flange->xyz[1] = 0;
	p_flange->xyz[2] = 0;

	matxx_malloc(&p_flange->T, 4, 4);

	return 0;

}

int16 InitToolCoordinate(tool_coordinate* p_tool)
{
	p_tool->origin[0] = 0;
	p_tool->origin[1] = 0;
	p_tool->origin[2] = 0;
	p_tool->xyz[0] = 0;
	p_tool->xyz[1] = 0;
	p_tool->xyz[2] = 0;
	p_tool->tcp[0] = 0;
	p_tool->tcp[1] = 0;
	p_tool->tcp[2] = 0;

	matxx_malloc(&p_tool->T, 4, 4);
	return 0;

}
// init user coordinate.
int16 InitUserCoordinate(user_coordinate* p_user)
{
	p_user->origin[0] = 0;
	p_user->origin[1] = 0;
	p_user->origin[2] = 0;
	p_user->xyz[0] = 0;
	p_user->xyz[1] = 0;
	p_user->xyz[2] = 0;

	matxx_malloc(&p_user->R, 3, 3);
	matxx_malloc(&p_user->d, 3, 1);
	matxx_malloc(&p_user->T, 4, 4);

	matxx_malloc(&p_user->m31, 3, 1);
	matxx_malloc(&p_user->m33, 3, 3);
	matxx_malloc(&p_user->m44, 4, 4);
	return 0;

}

int16 InitCoordinateTrans(coordinate_transformation* p_coort)
{
//	Uint8 i;

	matxx_malloc(&p_coort->T,4,4);

	matxx_malloc(&p_coort->m31, 3, 1);
	matxx_malloc(&p_coort->m33, 3, 3);
	matxx_malloc(&p_coort->m44, 4, 4);
	matxx_malloc(&p_coort->m44_, 4, 4);
	matxx_malloc(&p_coort->m44s, 4, 4);
	matxx_malloc(&p_coort->m44_s, 4, 4);
	matxx_malloc(&p_coort->R, 3, 3);
	matxx_malloc(&p_coort->d, 3, 1);
	return 0;
}

/*
 * tool's position and posture in base.
 * input:
 *			p_robotcs ---- the point of robot_coordinate_system
 *			joint_pose ---- the joint's current pos ,if it's NULL,use axis coordinate's value.
 *
 * output:	robot_coordinate.coordinate_transepose.R ---- the rotation matrix, tool ralative to base.
 *			robot_coordinate.coordinate_transepose.d ---- the pose matrix, tool ralative to base.
 *			robot_coordinate.coordinate_transepose.T ---- the Homogeneous transpose matrix, tool ralative to base.
 */
int16 TinB(robot_coordinate_system* p_robotcs,double* joint_pose)
{
	Uint8 tool_id = p_robotcs->tool_id;
	FinB(p_robotcs, joint_pose);

	p_robotcs->coor_t.m31.point[0][0] = p_robotcs->tool[tool_id].tcp[0];
	p_robotcs->coor_t.m31.point[0][1] = p_robotcs->tool[tool_id].tcp[1];
	p_robotcs->coor_t.m31.point[0][2] = p_robotcs->tool[tool_id].tcp[2];

	pb2pa(&p_robotcs->coor_t.T, &p_robotcs->coor_t.m31, &p_robotcs->coor_t.d);

	xyz2r(p_robotcs->tool[tool_id].tcf, &p_robotcs->coor_t.m33, 0);
	matxx_multiply(&p_robotcs->coor_t.R, &p_robotcs->coor_t.m33, &p_robotcs->coor_t.R);

	rd2t(&p_robotcs->coor_t.R, &p_robotcs->coor_t.d, &p_robotcs->coor_t.T);
	return 0;
}

/* scara coordinate
 *			   ^z	  ^z
 *			   |	  |
 *			   |	  |
 *	   ||======0r==x==0r___>x
 *     ||			  ||
 *	   ||			  ||
 *     ||			  0p___x
 *     ||			  |
 *	   ||			  v z
 *	   ||             ||
 *     ||			  0r____x     x<____0 tool
 *     ^z			  |                 | 
 *	   ||			  v z               v z
 *	   ||			  
 * base||__________>x
 *
 * flange's position and posture in base.
 * input:
 *			p_robotcs ---- the point of robot_coordinate_system
 *			joint_pose ---- the joint's current pos ,if it's NULL,use axis coordinate's value.
 *
 * output:	robot_coordinate.coordinate_transepose.R ---- the rotation matrix, flange ralative to base.
 *			robot_coordinate.coordinate_transepose.d ---- the pose matrix, flange ralative to base.
 *			robot_coordinate.coordinate_transepose.T ---- the Homogeneous transpose matrix, flange ralative to base.
 */
int16 FinB(robot_coordinate_system* p_rcs, double* joint_pose)
{
	Uint8 i, j;
	double theta;// , d;
	double q[AXIS_COORDINATE_NUM];

	coordinate_transformation *p_coort;
	p_coort = &p_rcs->coor_t;

	if (joint_pose == NULL)
	{
		for (i = 0; i < p_rcs->robot_type; i++)
		{
			q[i] = p_rcs->axis[i].q;
		}
	}
	else
	{
		for (i = 0; i < p_rcs->robot_type; i++)
		{
			if ((joint_pose + i) == NULL)
			{
				return -1;
			}
			q[i] = joint_pose[i];
		}
	}
	for (j = 0; j<p_rcs->robot_type; j++)
	{
		if (p_rcs->axis[j].type == 0)
		{
			//theta = q[j] + p_rcs->axis[j].theta;
			//d = p_rcs->axis[j].d;
			rotz(q[j], &p_coort->m44);
		}
		else
		{
			/*
			*	z
			*   |_____x  // joint 2
			*	 _____x  //joint3
			*	|
			*	z
			*/
			//d = -q[j] + p_rcs->axis[j].d;
			//theta = p_rcs->axis[j].theta;
			transz(-q[j], &p_coort->m44);
			//matxx_printf(&p_coort->m44);
		}
		if (j == 0)
		{
			dh2t(p_rcs->axis[j].a, p_rcs->axis[j].alpha, p_rcs->axis[j].d, p_rcs->axis[j].theta, &p_coort->T, 0);
			matxx_multiply(&p_coort->m44,&p_coort->T, &p_coort->T);
			//matxx_printf(&p_coort->T);
		}
		else
		{
			// 0^T_1 * 1^T_2 * 2^T_3 * 3^T_4
			dh2t(p_rcs->axis[j].a, p_rcs->axis[j].alpha, p_rcs->axis[j].d, p_rcs->axis[j].theta, &p_coort->m44_, 0);
			matxx_multiply( &p_coort->m44,&p_coort->m44_, &p_coort->m44_);
			matxx_multiply(&p_coort->T, &p_coort->m44_, &p_coort->T);
		}
	}
	//matxx_printf(&p_coort->T);
	t2rd(&p_coort->T, &p_coort->R, &p_coort->d);
	//matxx_printf(&p_coort->T);

	return 0;
}
/*
 * work in base.
 * tcp in base to flange in base
 * input:
 *		  p_rcs ---- the point to robot_coordinate_system
 *		  pose_in ---- 6 dim, tool's origin(tcp) and posture in base. Unit m/deg
 * output:
 *		  pose_out ---- 6 dim, flange's origin and posture in base. Unit m/deg
 *
 *			robot_coordinate.coordinate_transepose.R ---- the rotation matrix, flange ralative to base.
 *			robot_coordinate.coordinate_transepose.d ---- the pose matrix, flange ralative to base.
 *			robot_coordinate.coordinate_transepose.T ---- the Homogeneous transpose matrix, flange ralative to base.
 */
int16 bPt2bPf(robot_coordinate_system* p_rcs, double* pose_in, double* pose_out)
{
	coordinate_transformation *p_coort		= &p_rcs->coor_t;
	tool_coordinate			*p_tool = &p_rcs->tool[p_rcs->tool_id];
	xyz2r(&pose_in[3], &p_coort->m33, 0);

	// b'T_t ---- Homogeneous transform from base to tool
	p_coort->m31.point[0][0] = pose_in[0];
	p_coort->m31.point[0][1] = pose_in[1];
	p_coort->m31.point[0][2] = pose_in[2];

	rd2t(&p_coort->m33, &p_coort->m31, &p_coort->m44);

	//matxx_printf(&p_coort->m44);
	// (f'T_t)^-1 ---- Homogeneous transform from tool to flange
	xyz2r(p_tool->tcf, &p_coort->m33, 0);

	p_coort->m31.point[0][0] = p_tool->tcp[0];
	p_coort->m31.point[0][1] = p_tool->tcp[1];
	p_coort->m31.point[0][2] = p_tool->tcp[2];
	rd2t(&p_coort->m33, &p_coort->m31, &p_coort->m44_);

	//printf("tool relative flange");
	//matxx_printf(&p_coort->m44_);

	if (Ax_b(&p_coort->m44_, &p_coort->T, 1) == -1)
	{
		printf("matrix can't inverse");
		return -1;
	}
	//printf("flange relative tool");
	//matxx_printf(&p_coort->T);
	// b'T_f = b'T_t * t'T_f
	matxx_multiply(&p_coort->m44, &p_coort->T, &p_coort->T);
	t2rd(&p_coort->T, &p_coort->R, &p_coort->d);

	//printf("flange relative base");
	//matxx_printf(&p_coort->T);

	pose_out[0] = p_coort->d.point[0][0];
	pose_out[1] = p_coort->d.point[0][1];
	pose_out[2] = p_coort->d.point[0][2];

	r2xyz(&p_coort->R, &pose_out[3], 0);

	return 0;
}
int16 bPf2bPt(robot_coordinate_system* p_rcs, double* pose_in, double* pose_out)
{
	coordinate_transformation *p_coort = &p_rcs->coor_t;
	tool_coordinate			*p_tool = &p_rcs->tool[p_rcs->tool_id];
	xyz2r(&pose_in[3], &p_coort->m33, 0);

	// b'T_f ---- Homogeneous transform from base to flange
	p_coort->m31.point[0][0] = pose_in[0];
	p_coort->m31.point[0][1] = pose_in[1];
	p_coort->m31.point[0][2] = pose_in[2];

	rd2t(&p_coort->m33, &p_coort->m31, &p_coort->m44);

	//matxx_printf(&p_coort->m44);
	// f'T_t ---- Homogeneous transform from flange to tool  
	xyz2r(p_tool->tcf, &p_coort->m33, 0);

	p_coort->m31.point[0][0] = p_tool->tcp[0];
	p_coort->m31.point[0][1] = p_tool->tcp[1];
	p_coort->m31.point[0][2] = p_tool->tcp[2];
	rd2t(&p_coort->m33, &p_coort->m31, &p_coort->m44_);

	//printf("tool relative flange");
	//matxx_printf(&p_coort->m44_);

	// b'T_t =b'T_f  * f'T_t
	matxx_multiply(&p_coort->m44, &p_coort->m44_, &p_coort->T);
	t2rd(&p_coort->T, &p_coort->R, &p_coort->d);

	//printf("flange relative base");
	//matxx_printf(&p_coort->T);

	pose_out[0] = p_coort->d.point[0][0];
	pose_out[1] = p_coort->d.point[0][1];
	pose_out[2] = p_coort->d.point[0][2];

	r2xyz(&p_coort->R, &pose_out[3], 0);

	return 0;
}
/*
 * work in tool.
 *
 */
int16 tP2bPf(robot_coordinate_system* p_rcs, double* pose_in, double* pose_out)
{
	coordinate_transformation *p_coort	= &p_rcs->coor_t;
	tool_coordinate			*p_tool		= &p_rcs->tool[p_rcs->tool_id];

	double pos[6];

	// t'T ---- Homogeneous transform from old tool to new tool
	xyz2r(&pose_in[3], &p_coort->m33, 0);
	p_coort->m31.point[0][0] = pose_in[0];
	p_coort->m31.point[0][1] = pose_in[1];
	p_coort->m31.point[0][2] = pose_in[2];
	rd2t(&p_coort->m33, &p_coort->m31, &p_coort->m44s);
	//matxx_printf(&p_coort->m44s);
	// get current tool's Homogeneous transform relative base.
	TinB(p_rcs, NULL);
	printf("TinB  ");
	matxx_printf(&p_coort->T); 
	// b'T_tn = b'T_t * t'T_tn       tn:new tool's position and posture 
	matxx_multiply(&p_coort->T, &p_coort->m44s, &p_coort->T);
	t2rd(&p_coort->T, &p_coort->R, &p_coort->d);
	r2xyz(&p_coort->R, &pos[3], 0);
	pos[0] = p_coort->d.point[0][0];
	pos[1] = p_coort->d.point[0][1];
	pos[2] = p_coort->d.point[0][2];

	// tool relative base to flange relative base.
	bPt2bPf(p_rcs,pos,pose_out);
	return 0;
}
/*
* work in user.
*
*/
int16 uP2bPf(robot_coordinate_system* p_rcs, double* pose_in, double* pose_out)
{
	Uint8 user_id;
	double tmp[6] = { 0,0,0,0,0,0 };

	user_id = p_rcs->user_id;

	xyz2r(&pose_in[3], &p_rcs->user[user_id].m33, 0);

	p_rcs->user[user_id].m31.point[0][0] = pose_in[0];
	p_rcs->user[user_id].m31.point[0][1] = pose_in[1];
	p_rcs->user[user_id].m31.point[0][2] = pose_in[2];
	// point relate to user.
	rd2t(&p_rcs->user[user_id].m33, &p_rcs->user[user_id].m31, &p_rcs->user[user_id].m44);

	//printf("point in user  ");
	//matxx_printf(&p_rcs->user[user_id].m44);

	//printf("user in base ");
	//matxx_printf(&p_rcs->user[user_id].T);

	// point relate to base.
	matxx_multiply(&p_rcs->user[user_id].T, &p_rcs->user[user_id].m44, &p_rcs->user[user_id].m44);

	//printf("point in base  ");
	//matxx_printf(&p_rcs->user[user_id].m44);

	t2rd(&p_rcs->user[user_id].m44, &p_rcs->user[user_id].m33, &p_rcs->user[user_id].m31);
	//matxx_printf(&p_rcs->user[user_id].m33);
	r2xyz(&p_rcs->user[user_id].m33, &tmp[3],0);
	tmp[0] = p_rcs->user[user_id].m31.point[0][0];
	tmp[1] = p_rcs->user[user_id].m31.point[0][1];
	tmp[2] = p_rcs->user[user_id].m31.point[0][2];
	// point in base to flange in base.
	bPt2bPf(p_rcs, tmp, pose_out);

	return 0;
}

int16 uP2bP(robot_coordinate_system* p_rcs, double* pose_in, double* pose_out)
{
	Uint8 user_id;

	user_id = p_rcs->user_id;

	xyz2r(&pose_in[3], &p_rcs->user[user_id].m33, 0);

	p_rcs->user[user_id].m31.point[0][0] = pose_in[0];
	p_rcs->user[user_id].m31.point[0][1] = pose_in[1];
	p_rcs->user[user_id].m31.point[0][2] = pose_in[2];
	// point relate to user.
	rd2t(&p_rcs->user[user_id].m33, &p_rcs->user[user_id].m31, &p_rcs->user[user_id].m44);

	//printf("point in user  ");
	//matxx_printf(&p_rcs->user[user_id].m44);

	//printf("user in base ");
	//matxx_printf(&p_rcs->user[user_id].T);

	// point relate to base.
	matxx_multiply(&p_rcs->user[user_id].T, &p_rcs->user[user_id].m44, &p_rcs->user[user_id].m44);

	//printf("point in base  ");
	//matxx_printf(&p_rcs->user[user_id].m44);

	t2rd(&p_rcs->user[user_id].m44, &p_rcs->user[user_id].m33, &p_rcs->user[user_id].m31);

	r2xyz(&p_rcs->user[user_id].m33, &pose_out[3], 0);
	pose_out[0] = p_rcs->user[user_id].m31.point[0][0];
	pose_out[1] = p_rcs->user[user_id].m31.point[0][1];
	pose_out[2] = p_rcs->user[user_id].m31.point[0][2];

	return 0;
}


// 4 points to calibrate scara's tcp
// method can be found on http://www.rzrobot.hk/index.php?id=213
int16 ScaraTcpCalibration(robot_coordinate_system* p_rcs)
{
	Uint8 i;
//	Uint8 j;
	double theta;

	double a, b, c, d, e, f;
	tool_calibration*  p_toolc;
	p_toolc = &p_rcs->toolc;

	for (i = 0; i<4; i++)
	{
		FinB(p_rcs, p_toolc->p[i]);

		t2rd(&p_rcs->coor_t.T, &p_toolc->R[i], &p_toolc->d[i]);
	//matxx_printf(&p_toolc->m44);
	}
	// if 2~4 has coincidence point in cartesian.
	matxx_copy(&p_toolc->d[2], &p_toolc->m31);
	matxx_k_mac(-1, &p_toolc->d[1], &p_toolc->m31);
	matxx_copy(&p_toolc->d[3], &p_rcs->coor_t.m31);
	matxx_k_mac(-1, &p_toolc->d[2], &p_rcs->coor_t.m31);
	if (matxx_EuclideanNorm2(&p_toolc->m31) < 1E-10|| matxx_EuclideanNorm2(&p_rcs->coor_t.m31) < 1E-10)
	{
		p_rcs->tool[p_rcs->tool_id].tcp[0] = p_toolc->tcp.point[0][0] = 0;
		p_rcs->tool[p_rcs->tool_id].tcp[0] = p_toolc->tcp.point[0][1] = 0;
		p_rcs->tool[p_rcs->tool_id].tcp[2] = p_toolc->tcp.point[0][2] = (p_toolc->d[1].point[0][2] + p_toolc->d[2].point[0][2] + \
			p_toolc->d[3].point[0][2]) / 3.0 - p_toolc->d[0].point[0][2];
		return 0;
	}

	// save for use
	matxx_copy(&p_rcs->coor_t.T, &p_toolc->m44);

	a = p_toolc->d[2].point[0][0] - p_toolc->d[1].point[0][0];
	b = p_toolc->d[2].point[0][1] - p_toolc->d[1].point[0][1];
	c = (pow(p_toolc->d[2].point[0][0], 2) - pow(p_toolc->d[1].point[0][0], 2) + \
		pow(p_toolc->d[2].point[0][1], 2) - pow(p_toolc->d[1].point[0][1], 2)) / 2.0;
	d = p_toolc->d[3].point[0][0] - p_toolc->d[1].point[0][0];
	e = p_toolc->d[3].point[0][1] - p_toolc->d[1].point[0][1];
	f = (pow(p_toolc->d[3].point[0][0], 2) - pow(p_toolc->d[1].point[0][0], 2) + \
		pow(p_toolc->d[3].point[0][1], 2) - pow(p_toolc->d[1].point[0][1], 2)) / 2.0;

	if (fabs(a*e - b*d) < 1E-10)
	{
		printf("error point:the 2~4 three points can't form a circle");
		p_toolc->tcp.point[0][0] = 0;
		p_toolc->tcp.point[0][1] = 0;

		return -1;
	}
	else
	{	// x,y in base frame.
		p_toolc->tcp.point[0][0] = (c*e - b*f) / (a*e - b*d);
		p_toolc->tcp.point[0][1] = (a*f - c*d) / (a*e - b*d);
	}
	// z in base frame.
	p_toolc->tcp.point[0][2] = p_toolc->d[3].point[0][2];
	//matxx_printf(&p_rcs->toolc.tcp);
	// T:base relative to flange
	if (Ax_b(&p_toolc->m44, &p_toolc->m44_, 1) == -1)
	{
		printf("the matrix T can't inverse");
		return -1;
	}

	pb2pa(&p_toolc->m44_, &p_toolc->tcp, &p_toolc->m31);

	matxx_copy(&p_toolc->m31, &p_toolc->tcp);
	//matxx_printf(&p_toolc->tcp);
	// z in flange frame.
	p_toolc->tcp.point[0][2] = (p_toolc->d[1].point[0][2] +p_toolc->d[2].point[0][2] +\
								p_toolc->d[3].point[0][2]) / 3.0 - p_toolc->d[0].point[0][2];
	p_rcs->tool[p_rcs->tool_id].tcp[0] = p_toolc->tcp.point[0][0];
	p_rcs->tool[p_rcs->tool_id].tcp[1] = p_toolc->tcp.point[0][1];
	p_rcs->tool[p_rcs->tool_id].tcp[2] = p_toolc->tcp.point[0][2];

	return 0;
}
/*
* tool center point calibration. 
* calibrate tool coordinate origin relative to flange.
* 4 point's least square method
* Ax=b    A:m*n (m>n)
* the least square method
* A'*Ax=A'*b
* Least squares solution = (A'*A)^-1*A'*b
*
* method can be found on http://www.doc88.com/p-9999383491968.html or 
* https://wenku.baidu.com/view/da7ee096551810a6f52486e8.html
*
*/
int16 TcpCalibration(robot_coordinate_system* p_rcs)
{
	Uint8 i;
//	Uint8 j;
	double theta;
//	double d;
	tool_calibration*  p_toolc;
	p_toolc = &p_rcs->toolc;

	for (i = 0; i<4; i++)
	{
		// flange's origin pos in base.
		FinB(p_rcs, p_toolc->p[i]);

		t2rd(&p_rcs->coor_t.T, &p_toolc->R[i], &p_toolc->d[i]);
	}

	// A=[R1-R2;R1-R3;R1-R4]
	matxx_reshape(&p_toolc->m93, &p_toolc->R[0], &p_toolc->R[0], &p_toolc->R[0]);
	matxx_reshape(&p_toolc->m93_, &p_toolc->R[1], &p_toolc->R[2], &p_toolc->R[3]);

	matxx_k_mac(-1, &p_toolc->m93_, &p_toolc->m93);
	//matxx_printf(&p_toolc->m93);
	// A'
	matxx_transpose(&p_toolc->m93, &p_toolc->m39);

	// A'*A
	matxx_multiply(&p_toolc->m39, &p_toolc->m93, &p_toolc->m33);


	// (A'*A)^-1
	if (Ax_b(&p_toolc->m33, &p_toolc->tmp33, 1) == -1)
	{
		printf("\n the matrix T can't inverse\n");
		return -1;
	}
	// (A'*A)^-1*A'
	matxx_multiply(&p_toolc->tmp33, &p_toolc->m39, &p_toolc->m39);

	// b=[d2-d1;d3-d1;d4-d1]
	matxx_reshape(&p_toolc->m91_, &p_toolc->d[0], &p_toolc->d[0], &p_toolc->d[0]);

	matxx_reshape(&p_toolc->m91, &p_toolc->d[1], &p_toolc->d[2], &p_toolc->d[3]);

	matxx_k_mac(-1, &p_toolc->m91_, &p_toolc->m91);

	// (A'*A)^-1*A'*b
	matxx_multiply(&p_toolc->m39, &p_toolc->m91, &p_toolc->tcp);

	p_rcs->tool[p_rcs->tool_id].origin[0] = p_toolc->tcp.point[0][0];
	p_rcs->tool[p_rcs->tool_id].origin[1] = p_toolc->tcp.point[0][1];
	p_rcs->tool[p_rcs->tool_id].origin[2] = p_toolc->tcp.point[0][2];

	return 0;
}

// calibrate the posture of tool ralative to flange.
int16 TcfCalibration(robot_coordinate_system* p_rcs)
{
	double k;
	double norm;
	Uint8 i;
	tool_calibration*  p_toolc;
	p_toolc = &p_rcs->toolc;

	double  p1[3], p2[3], p3[3];
	double r[3];
	double tmp[3];

	TinB(p_rcs, p_rcs->toolc.p[3]);
	p1[0] = p_rcs->coor_t.d.point[0][0];
	p1[1] = p_rcs->coor_t.d.point[0][1];
	p1[2] = p_rcs->coor_t.d.point[0][2];
	TinB(p_rcs, p_rcs->toolc.p[4]);
	p2[0] = p_rcs->coor_t.d.point[0][0];
	p2[1] = p_rcs->coor_t.d.point[0][1];
	p2[2] = p_rcs->coor_t.d.point[0][2];
	TinB(p_rcs, p_rcs->toolc.p[5]);
	p3[0] = p_rcs->coor_t.d.point[0][0];
	p3[1] = p_rcs->coor_t.d.point[0][1];
	p3[2] = p_rcs->coor_t.d.point[0][2];

	// d=p1+(p1-p2)*k
	// k=solve((p2-p1)'*(d-p3)==0)
	// k=-((p1(1) - p2(1))*(p1(1) - p3(1)) + (p1(2) - p2(2))*(p1(2) - p3(2)) + (p1(3) -p2(3))*(p1(3)...
	// - p3(3)))/((p1(1) - p2(1))^2 + (p1(2) - p2(2))^2 + (p1(3) - p2(3))^2)
	k = -((p1[0] - p2[0])*(p1[0] - p3[0]) + (p1[1] - p2[1])*(p1[1] - p3[1]) + \
		(p1[2] - p2[2])*(p1[2] - p3[2])) / (pow((p1[0] - p2[0]), 2) + \
			pow((p1[1] - p2[1]), 2) + pow((p1[2] - p2[2]), 2));
	
	// d=p1+(p1-p2)*k
	for (i = 0; i < 3; i++)
	{
		r[i]	= p1[i] + (p1[i] - p2[i])*k;
		// p2-p1
		tmp[i]	= p2[i] - p1[i];
	}
	// norm(p2 - p1)
	norm = pow(pow(tmp[0], 2) + pow(tmp[1], 2) + pow(tmp[2], 2), 0.5);
	// R = [x; y;z ]  rotation matrix ,tool's posture relative to base.
	for (i = 0; i < 3; i++)
	{
		// x=(p2-p1)/norm(p2-p1)
		p_toolc->tcf.point[0][i] = tmp[i] / norm;
		// p3-d
		tmp[i] = p3[i] - r[i];
	}
	// norm(p3 - d)
	norm = pow(pow(tmp[0], 2) + pow(tmp[1], 2) + pow(tmp[2], 2), 0.5);
	for (i = 0; i < 3; i++)
	{
		// z=(p3-d)/norm(p3-d)
		p_toolc->tcf.point[2][i] = tmp[i] / norm;
	}
	// y=z X x = [z1*x2 - x1*z2, -(z0*x2 - x0*z2), z0*x1 - x0*z1]
	p_toolc->tcf.point[1][0] = p_toolc->tcf.point[2][1] * p_toolc->tcf.point[0][2] - \
							   p_toolc->tcf.point[0][1] * p_toolc->tcf.point[2][2];
	p_toolc->tcf.point[1][1] = -p_toolc->tcf.point[2][0] * p_toolc->tcf.point[0][2] + \
							   p_toolc->tcf.point[0][0] * p_toolc->tcf.point[2][2];
	p_toolc->tcf.point[1][2] = p_toolc->tcf.point[2][0] * p_toolc->tcf.point[0][1] - \
							   p_toolc->tcf.point[0][0] * p_toolc->tcf.point[2][1];
	// flange's posture in world.
	FinB(p_rcs, p_rcs->toolc.p[5]);
	matxx_transpose(&p_rcs->coor_t.R, &p_toolc->m33);
	//matxx_printf(&p_toolc->m33);
	matxx_multiply(&p_toolc->m33, &p_toolc->tcf, &p_toolc->tcf);
	//matxx_printf(&p_toolc->tcf);
	r2xyz(&p_toolc->tcf, p_rcs->tool[p_rcs->tool_id].tcf, 0);
	return 0;
}
int16 ToolCalibration(robot_coordinate_system* p_rcs)
{
	// scara
	if (p_rcs->robot_type == 4)
	{
		if (ScaraTcpCalibration(p_rcs) == -1) {
			return -1;
		}
	}
	else
	{
		if (TcpCalibration(p_rcs) == -1) {
			return -1;
		}
	}
	if (TcfCalibration(p_rcs) == -1) {
		return -1;
	}
	return 0;
}
// robot teaching
/*              p2 <--x axis
* 				|
* 				|
* 				p1 <----origin
* 				|
* 				d--------p3 <---z axis
*
*/
int16 UserCalibration(robot_coordinate_system* p_rcs)
{
	Uint8 i;
	double k,norm;
	double r[3];
	double tmp[3];
	double p1[3], p2[3], p3[3];

	TinB(p_rcs, p_rcs->userc.p[0]);
	p1[0] = p_rcs->coor_t.d.point[0][0];
	p1[1] = p_rcs->coor_t.d.point[0][1];
	p1[2] = p_rcs->coor_t.d.point[0][2];
	TinB(p_rcs, p_rcs->userc.p[1]);
	p2[0] = p_rcs->coor_t.d.point[0][0];
	p2[1] = p_rcs->coor_t.d.point[0][1];
	p2[2] = p_rcs->coor_t.d.point[0][2];
	TinB(p_rcs, p_rcs->userc.p[2]);
	p3[0] = p_rcs->coor_t.d.point[0][0];
	p3[1] = p_rcs->coor_t.d.point[0][1];
	p3[2] = p_rcs->coor_t.d.point[0][2];

	// d=p1+(p1-p2)*k
	// k=solve((p2-p1)'*(d-p3)==0)
	// k=-((p1(1) - p2(1))*(p1(1) - p3(1)) + (p1(2) - p2(2))*(p1(2) - p3(2)) + (p1(3) -p2(3))*(p1(3)...
	// - p3(3)))/((p1(1) - p2(1))^2 + (p1(2) - p2(2))^2 + (p1(3) - p2(3))^2)
	k = -((p1[0] - p2[0])*(p1[0] - p3[0]) + (p1[1] - p2[1])*(p1[1] - p3[1]) + \
		(p1[2] - p2[2])*(p1[2] - p3[2])) / (pow((p1[0] - p2[0]), 2) + \
			pow((p1[1] - p2[1]), 2) + pow((p1[2] - p2[2]), 2));

	// d=p1+(p1-p2)*k
	for (i = 0; i < 3; i++)
	{
		r[i] = p1[i] + (p1[i] - p2[i])*k;
		// p2-p1
		tmp[i] = p2[i] - p1[i];
	}

	// norm(p2 - p1)
	norm = pow(pow(tmp[0], 2) + pow(tmp[1], 2) + pow(tmp[2], 2), 0.5);
	// R = [x; y;z ]  rotation matrix ,tool's posture relative to base.
	for (i = 0; i < 3; i++)
	{
		// x=(p2-p1)/norm(p2-p1)
		p_rcs->userc.R.point[0][i] = tmp[i] / norm;
		// p3-d
		tmp[i] = p3[i] - r[i];
	}
	// norm(p3 - d)
	norm = pow(pow(tmp[0], 2) + pow(tmp[1], 2) + pow(tmp[2], 2), 0.5);
	for (i = 0; i < 3; i++)
	{
		// z=(p3-d)/norm(p3-d)
		p_rcs->userc.R.point[2][i] = tmp[i] / norm;
	}
	// y=z X x = [z1*x2 - x1*z2, -(z0*x2 - x0*z2), z0*x1 - x0*z1]
	p_rcs->userc.R.point[1][0] = p_rcs->userc.R.point[2][1] * p_rcs->userc.R.point[0][2] - \
		p_rcs->userc.R.point[0][1] * p_rcs->userc.R.point[2][2];
	p_rcs->userc.R.point[1][1] = -p_rcs->userc.R.point[2][0] * p_rcs->userc.R.point[0][2] + \
		p_rcs->userc.R.point[0][0] * p_rcs->userc.R.point[2][2];
	p_rcs->userc.R.point[1][2] = p_rcs->userc.R.point[2][0] * p_rcs->userc.R.point[0][1] - \
		p_rcs->userc.R.point[0][0] * p_rcs->userc.R.point[2][1];

	// save data.
	matxx_copy(&p_rcs->userc.R, &p_rcs->user[p_rcs->user_id].R);
	p_rcs->user[p_rcs->user_id].origin[0] = p1[0];
	p_rcs->user[p_rcs->user_id].origin[1] = p1[1];
	p_rcs->user[p_rcs->user_id].origin[2] = p1[2];

	p_rcs->user[p_rcs->user_id].d.point[0][0] = p1[0];
	p_rcs->user[p_rcs->user_id].d.point[0][1] = p1[1];
	p_rcs->user[p_rcs->user_id].d.point[0][2] = p1[2];

	rd2t(&p_rcs->user[p_rcs->user_id].R, &p_rcs->user[p_rcs->user_id].d, &p_rcs->user[p_rcs->user_id].T);
	return 0;
}


void RobotCoordinateISR()
{
	// get joint fd,and save it to axis coordinate.




	extern robot_coordinate_system robot_cs;
	FinB(&robot_cs, NULL);
	matxx_copy(&robot_cs.coor_t.T, &robot_cs.flange[0].T);

	TinB(&robot_cs, NULL);
	matxx_copy(&robot_cs.coor_t.T, &robot_cs.tool[robot_cs.tool_id].T);


	return;
}


