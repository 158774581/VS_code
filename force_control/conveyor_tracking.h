/*
* conveyor_tracking.h
*
*  Created on: Nov 26, 2018
*      Author: hqi
*/
#ifndef CONVEYOR_TRACKING_H_
#define CONVEYOR_TRACKING_H_

#include "type_def.h"
#include "robot_config.h"
#include "joint_module.h"
#include "cartesian_module.h"
#include "kinematics.h"
#include "trajectory_generator.h"
#include "motion_planning_3rd.h"
#include "robot_coordinate_system.h"
#include "robot_module.h"
//#include "conveyor_tracking.h"

//#include "robot_device.h"
//#include "robot_app.h"


#define		MAX_ROBOR_NUM				16
#define		MAX_CONVEYOR_NUM			16
#define		MAX_WOBJ_STREAM_NUM			8
#define		MAX_WOBJ_NUM				1000
#define		MAX_ROBOT_JOINT_DIM			8
#define		MAX_CART_DIM			    6

#define     PLACEMENT_DIM				6

// 系统参数
#define		CONVEYOR_VEL_AMPLITUDE		0.05  //传送带速度波动
#define		CONVEYOR_VEL				500
#define		CONVEYOR_COUNTER_PER_METER  10000

#define     WOBJ_STATE_IDLE				0
#define     WOBJ_STATE_USED				1
#define		WOBJ_STATE_RUNNING			2

#define     CNV_TRCK_SIMPLE_TIME		1  //ms


typedef struct TIME
{
	int32		year;
	Uint8		month; 
	Uint8		day;
	Uint8		hour;
	Uint8		minute;
	Uint8		second;
	Uint32		ticks;  // millisecond
	Uint32		cur_ticks;  // var  Unit ms
}time;


typedef enum 
{
	OVER_FLOW,
	BY_PASS,
	AREA_DISTRIBUTE,
	PART_TYPE


}sort_strategy;

typedef enum 
{
	PRECISION_FIRST,
	SPEED_FIRST			// no PID


}pickup_mode;

typedef enum
{
	IDLE,
	PICK_READY,
	PICK_INTERCEPT,
	PICK_TRACKING,
	PICK_CHASE,
	PLACE_READY,
	PLACE_INTERCEPT,
	PLACE_TRACKING
}conveyor_robot_state;

typedef enum
{
	INTERCEPT_IDLE=0,
	INTERCEPT_LIFT=1,
	INTERCEPT_APPROACH=2,
	INTERCEPT_LAYDOWN=3
}conveyor_robot_intercept_state;

typedef	struct CONVEYOR_CONTROL
{
	double  spd_change_t;
	double  acc_max;
	double  vel_max;

	// Var

}conveyor_control;

typedef	struct WOBJ
{
	Uint16			id;
	Uint8           state;
	Uint8			type;  // type of work object,default 0.
	Uint8			robot_id;  // the robot will pick up this work object.
	int32			init_mot_pos;  // the mot pos while photographing. Unit 1 pulse
	int32			init_mot_vel;  // the mot vel while photographing.
	double			init_y;  // the y in conveyor frame while photographing.
	time			init_time;  // the time while photographing.
}wobj;

typedef	struct WOBJ_QUEUE
{
	Uint16					wobj_num;
	wobj					wobj_buff[MAX_WOBJ_NUM];
	wobj*					insrt_wobj;  // the wobj who is being insert calculating
	wobj*					cal_wobj;  // the wobj who is being calculating
	wobj*					trck_wobj;  // the wobj who is being tracking
}wobj_queue;

typedef	struct PYRAMID
{
	Uint32							max_iter;
	double							eps_pyr;  //  accuracy of pyramid iteration.  Unit mm
	double							eps2_pyr;  // second level accuracy of pyramid iteration.  Unit mm
	Uint32							up_down_time;  // motion lift tool , lay down tool and pick up 's time interval.  Unit ms.
	Uint32							comp_time;  // conpsate robot's iteration time. Unit ms  
}pyramid;

typedef struct PID
{
	double							eps_pid;
	double							err[3];  // pid control in x,y.
	double							err_max;  // max err.
	double							m[3];  // pid control in x,y.
	double							m_max;  // Unit mm

	double							Ierr[3];  // the integral of error.
	double							Ierr_max;  // the integral of error.
	double							Derr[3];  // the last error.

	double							Kp;
	double							Ti;
	double							Td;
	Uint32							prm_trck_time;
	Uint32							cur_trck_time;
}pid;

// 用凸轮曲线让机器人相对工件做跟随运动
typedef struct CAM
{
	double							d;  // 凸轮距离，用户给定，Unit mm
	double							c0;  // 凸轮曲线系数
	double							c1;  // 凸轮曲线系数
	double							c2;  // 凸轮曲线系数
	double							c3;  // 凸轮曲线系数
	double							c4;  // 凸轮曲线系数
	int32							init_cam_pos;  // 凸轮咬合时工件的位置。

}cam;

/*
 *   sensor               upper limit								lower limit
 *      |                      |										|
 *		|<--queue_trck_dist--->|<---start_win_width----->|				|
 *   ___|______________________|_________________________|______________|___
 *      |                      ^y                        |				|
 *      |                      |                         |				|
 *      |                      |						 |				|
 *      |         conveyor_base|_____>x					 |				|----> conveyor belt direction
 *      |                      |						 |				|
 *   ___|______________________|_________________________|______________|___
 *					^y
 *					|								  ^y
 *            x<____|world					          |
 *                                              x<____|robot1  ...
 *
 */
// conveyor belt is in servo vel loop
typedef	struct CONVEYOR
{
	Uint8					id;  
	Uint8					type;  // 0 line;1 circle
	Uint8					direction;  // 0 :forward ,same as x; 1:backward,same as -x.
	sort_strategy			sort_strategy;  // the strategy to sort
	Uint8					last_conveyor;  // upstream conveyor
	Uint8					next_conveyor;  // downstream conveyor

	double					counts_per_meter;  // Change in the number of encoders per metre of conveyor belt
	double					pulse_2_mm;
	Uint32					queue_trck_dist;   // Unit pulse
	int32					start_win_width;  // Unit pulse
	conveyor_control		conveyor_ctl;

	// prm 
	pyramid							pyramid;
	cam								cam;
	pid								pid;

	Uint8							tracking_delay;  // follow work objecct sometime.

	double							down_height;  // Unit mm.
												  
	// upper and lower limit 
	double							U1[2];  // upper limit in conveyor;x,y
	double							U2[2];
	double							L1[2];
	double							L2[2];

	conveyor_robot_state			state;
	conveyor_robot_intercept_state	intercept_sts;
	Uint8							tool_num;  // multiply tool in flange.
	double							tcp[4];
	double							place_point[PLACEMENT_DIM];  // placement point ralete to robot base,the flange ralete to robot base,[x,y,z,roll,pitch,yaw].---teaching
	double							lift_point[PLACEMENT_DIM];  // lift motion
	double							approach_point[PLACEMENT_DIM];  // approach motion,placement point ralete to robot base,[x,y,z,roll,pitch,yaw].
	double							laydown_point[PLACEMENT_DIM];  // laydown motion
	double							pick_pose[3];  // the pose of tool ralete to robot base.

	// gServo   一个伺服模块
	// Var -->synchronize every 1ms 
	int32					mot_pos_fd;  // Unit pulse ---feedback
	int32					mot_vel_fd;  // Unit pulse/s --- feedback
	int32					mot_acc_fd;  // Unit pulse/s^2 --- feedback
	double					mot_pos_cmd;  // Unit pulse ---cmd
	double					mot_vel_cmd;  // Unit pulse/s --- cmd
	double					mot_acc_cmd;  // Unit pulse/s^2 --- cmd
	double					spd_change_t;
	double				    acc_max;
	double					vel_max;
	double					vel_new;  //Unit pulse/s

	// work object queue -->get from net work
	wobj_queue				queue;

}conveyor;


typedef	struct CONVEYOR_ROBOT
{
	  // profile

	//---------------------------------------------------------------
    // input
	double							joint_pos_fd[MAX_ROBOT_JOINT_DIM];
	double							cart_pos_fd[MAX_CART_DIM];
	// output
	double							joint_pos_cmd[MAX_ROBOT_JOINT_DIM];
	double							cart_pos_cmd[MAX_CART_DIM];


}conveyor_robot;

typedef	struct CONVEYOR_SENSOR
{
	Uint32			min_latch_cnt;  // encoder




}conveyor_sensor;

typedef	struct CONVEYOR_CALIBRATION
{
	Uint8			CountsPerMeter;
	Uint8			robot_num;




}conveyor_calibration;

typedef	struct CONVEYOR_TRACKING
{
	Uint8					NrOfConveyor;
	Uint8					NrOfRobot;

	conveyor				conveyor[MAX_CONVEYOR_NUM];
	conveyor_robot			conv_rbt[MAX_ROBOR_NUM];
	conveyor_sensor			conv_sensor;

	conveyor_calibration	conv_calib;

}conveyor_tracking;


int16 InitConveyor(conveyor* cnv, Uint8 id,Uint8 robot_id);

int16 InitConveyorTracking(conveyor_tracking* cnv_tra,Uint8 robot_id);

int16 QueueIn(wobj_queue* queue, Uint8 type, Uint8 robot_id, int32 init_mot_pos, int32 init_mot_vel, double init_y, time* init_time);

int16 CalUpDownTime(conveyor* cnv, conveyor_robot* rbt, robot_config_module* cfg, double up_height, double down_height);

int16 PyramidOptimization(conveyor* cnv, conveyor_robot* rbt, robot_config_module* cfg, robot_coordinate_system* p_cs, double* intercept);

int16 InsertInterceptBlocks(trajectory_module* tra, robot_config_module* cfg, conveyor* cnv, double* start_pos, double* end_pos, double* aux_pos);

int16 conveyor_tracking_mode(conveyor_tracking* p_cnv_trck, robot_config_module* p_config, trajectory_module* m_traj, \
							 robot_coordinate_system* p_cs, cartesian_module* p_cart, joint_module* p_joint, Uint8 id);

int16 PIDTrack(conveyor* cnv, cartesian_module* p_cart, joint_module* p_joint, robot_config_module* cfg);

int16 cnv_rbt_traj_gen(trajectory_module* p_trajectory, cartesian_module* p_cart, joint_module* p_joint, robot_config_module* p_config, double* err, Uint8 id);
#endif // !CONVEYOR_TRACKING_H_

