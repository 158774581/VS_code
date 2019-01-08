#pragma once
/*
* robot_coordinate_system.h
*
*  Created on: Oct 17, 2018
*      Author: hqi
*/

#ifndef ROBOT_CONTROL_ROBOT_COORDINATE_SYSTEM_INCLUDE__ROBOT_COORDINATE_SYSTEM_H_
#define ROBOT_CONTROL_ROBOT_COORDINATE_SYSTEM_INCLUDE__ROBOT_COORDINATE_SYSTEM_H_

#include	"matrix.h"
#include	"math.h"


#define		AXIS_COORDINATE_NUM				8
#define		TOOL_COORDINATE_NUM				16
#define		USER_COORDINATE_NUM				16
#define		BASE_COORDINATE_NUM				16



typedef struct TOOL_CALIBRATION
{
	//Uint8 	toolc_id;  // the tool coordinate's id need to be calibrated.
	double	p[6][AXIS_COORDINATE_NUM];  // 6 points(4+2) can calibration tool center point and tool's posture.  Unit deg/mm
	matxx	tcp;  // 3*1 matrix, tool center point relative to flange coordinate.
	matxx	tcf;  // 3*1 matrix, tool posture ralative to flange.
	matxx	R[4];  // 3*3 matrix,flange's posture relative to base. in 4 points calibration method.
	matxx	d[4];  // 3*1 matrix,flange's position relative to base. in 4 points calibration method.

	matxx	m44;
	matxx	m44_;
	matxx	m93;
	matxx	m93_;
	matxx	m91;
	matxx	m91_;
	matxx	m39;
	matxx	m33;
	matxx	m31;
	matxx	tmp33;

}tool_calibration;

typedef struct USER_CALIBRATION
{
	double	p[3][AXIS_COORDINATE_NUM];  // 3 point. the first one is origin,the secend is on x frame, the third is on y frame. relative to base coordinate

	matxx	d;
	matxx	m31;
	matxx	m31_;
	matxx	R;  // rotation matrix relative base coordinate.
	double  xyz[3];  // posture of user coordinate relative to base coordinate.  in xyz angle.


}user_calibration;

typedef struct BASE_COORDINATE
{
	matxx	R[4];  // 3*3 matrix,flange's posture relative to base. in 4 points calibration method.
	matxx	d[4];  // 3*1 matrix,flange's position relative to base. in 4 points calibration method.




}base_coordinate;

typedef struct WORLD_COORDINATE
{
	matxx	R[4];  // 3*3 matrix,flange's posture relative to base. in 4 points calibration method.
	matxx	d[4];  // 3*1 matrix,flange's position relative to base. in 4 points calibration method.
	



}world_coordinate;

typedef struct AXIS_COORDINATE
{
	Uint8 		type;  // the type of axis .0 for rotation and 1 for prismatic,
	double 		q;  // the val of the axis,come from servo fd.
	double		offset;

	// the D-H
	double		a;  // Unit mm
	double		alpha;  // Unit deg
	double		d;	// Unit mm
	double		theta;  // Unit deg


}axis_coordinate;

typedef struct FLANGE_COORDINATE
{
	double					origin[3];  // origin of flange coordinate relative to base coordinate.
	double					xyz[3];  // posture of flange coordinate relative to base coordinate.  in xyz angle.
	matxx					T;  // 4 X 4 matrix.Homogeneous transpose of flange coordinate relative to base coordinate.


}flange_coordinate;


typedef struct TOOL_COORDINATE
{
	double					origin[3];  // origin of tool coordinate relative to base coordinate.
	double					xyz[3];  // posture of tool coordinate relative to base coordinate.  in xyz angle.
	matxx					T;  // 4 X 4 matrix.Homogeneous transpose of tool coordinate relative to base coordinate.

	double					tcp[3];  // tool center point relative to flange coordinate.
	double					tcf[3];  // tool posture relative to flange coordinate in xyz angle.Unit deg

}tool_coordinate;

typedef struct USER_COORDINATE
{
	double					origin[3];  // origin of tool coordinate relative to base coordinate.
	double					xyz[3];  // posture of tool coordinate relative to base coordinate.  in xyz angle.
	matxx					R;  // the rotation ,user coordinate relative base coordinate.
	matxx					d;

	matxx					T; // 4 X 4 matrix.Homogeneous transform of user coordinate relative to base coordinate.
	matxx					m33;
	matxx					m31;
	matxx					m44;

}user_coordinate;
typedef struct COORDINATE_TRANSFORMATION
{
	matxx	T; //4 X 4 matrix.Homogeneous transpose.
	matxx	R;  // 3*3 matrix
	matxx	d;  // 3*1 matrix

	matxx	m44;
	matxx	m44_;

	matxx	m44s;
	matxx	m44_s;

	matxx	m33;
	matxx	m31;

}coordinate_transformation;

typedef struct ROBOT_COORDINATE_SYSTEM
{
	Uint8					robot_type;  // 4 :4 axis
	Uint8 					tool_id;  // current tool coordinate  0~TOOL_COORDINATE_NUM-1
	Uint8					user_id;  // current user coordinate  0~USER_COORDINATE_NUM-1

	base_coordinate			base[BASE_COORDINATE_NUM];
	flange_coordinate		flange[BASE_COORDINATE_NUM];
	world_coordinate		world;
	axis_coordinate			axis[AXIS_COORDINATE_NUM];
	tool_coordinate			tool[TOOL_COORDINATE_NUM];
	user_coordinate			user[USER_COORDINATE_NUM];

	coordinate_transformation		coor_t;

	tool_calibration		toolc;
	user_calibration		userc;

	int16(*pfTcpCalibration)(struct ROBOT_COORDINATE_SYSTEM* p_robotcs);

	int16(*pfScaraTcpCalibration)(struct ROBOT_COORDINATE_SYSTEM* p_rcs);

	int16(*pfTcfCalibration)(struct ROBOT_COORDINATE_SYSTEM* p_rcs);

	int16(*pfUserCalibration)(struct ROBOT_COORDINATE_SYSTEM* p_rcs);

	int16(*pfInitRobotCoordinateSystem)(struct ROBOT_COORDINATE_SYSTEM* p_robotcs);

	int16(*pfFinB)(struct ROBOT_COORDINATE_SYSTEM* p_rcs, double* joint_pose);

	int16(*pfTinB)(struct ROBOT_COORDINATE_SYSTEM* p_robotcs, double* joint_pose);

	int16(*pfbPt2bPf)(struct ROBOT_COORDINATE_SYSTEM* p_rcs, double* pose_in, double* pose_out);

	int16(*pftP2bPf)(struct ROBOT_COORDINATE_SYSTEM* p_rcs, double* pose_in, double* pose_out);

	int16(*pfuP2bPf)(struct ROBOT_COORDINATE_SYSTEM* p_rcs, double* pose_in, double* pose_out);

}robot_coordinate_system;

int16 InitAxisCoordinate(axis_coordinate* p_axis);

int16 InitToolCoordinate(tool_coordinate* p_tool);

int16 InitUserCoordinate(user_coordinate* p_user);

int16 InitCoordinateTrans(coordinate_transformation* p_coort);

int16 InitRobotCoordinateSystem(robot_coordinate_system* p_robotcs);

int16 ScaraTcpCalibration(robot_coordinate_system* p_rcs);

int16 TcpCalibration(robot_coordinate_system* p_rcs);

int16 TcfCalibration(robot_coordinate_system* p_rcs);

int16 UserCalibration(robot_coordinate_system* p_rcs);


int16 FinB(robot_coordinate_system* p_rcs, double* joint_pose);

int16 TinB(robot_coordinate_system* p_robotcs, double* joint_pose);

int16 bPt2bPf(robot_coordinate_system* p_rcs, double* pose_in, double* pose_out);

int16 bPf2bPt(robot_coordinate_system* p_rcs, double* pose_in, double* pose_out);

int16 tP2bPf(robot_coordinate_system* p_rcs, double* pose_in, double* pose_out);

int16 uP2bPf(robot_coordinate_system* p_rcs, double* pose_in, double* pose_out);

int16 uP2bP(robot_coordinate_system* p_rcs, double* pose_in, double* pose_out);

void RobotCoordinateISR();

robot_coordinate_system		robot_cs;

#endif // !ROBOT_CONTROL_ROBOT_COORDINATE_SYSTEM_INCLUDE__ROBOT_COORDINATE_SYSTEM_H_
