/*
* trajectory_generator.h
*
*  Created on: mar 15, 2018
*      Author: root
*/

#ifndef ROBOT_CONTROL_TRAJECTORY_GENERATOR_INCLUDE_TRAJECTORY_GENERATOR_H
#define ROBOT_CONTROL_TRAJECTORY_GENERATOR_INCLUDE_TRAJECTORY_GENERATOR_H
//--------------------------------------------------------------------------
#include "type_def.h"
#include "matrix.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "math.h"


//--------------------------------------------------------------------------
//define the motion planning coordinate type
typedef enum
{
	COORD_TYPE_JOINT = 0,
	COORD_TYPE_CART_TRANS = 1,
	COORD_TYPE_CART_ROT = 2,
	COORD_TYPE_CART = 3,
	REV_PRF_COORD_TYPE = 0x8000
}prfcoord_type;
//--------------------------------------------------------------------------
//define the motion planning jointp2p sync method
typedef enum
{
	JOINTP2P_TYPE_TIME_SCALE = 0,
	JOINTP2P_TYPE_TIME_SAME = 1,
	REV_JOINTP2P_TYPE = 0x8000
}jointp2p_sync;

//--------------------------------------------------------------------------
//define the motion planning cartline sync method
typedef enum
{
	CARTLINE_TYPE_SYNC_TIME_SCALE = 0,
	CARTLINE_TYPE_SYNC_TIME_SAME = 1,
	REV_CARTLINE_TYPE_SYNC = 0x8000
}cartline_sync;
//--------------------------------------------------------------------------
//define the motion planning cartline sync method
typedef enum
{
	CARTCIRCLE_TYPE_SYNC_TIME_SCALE = 0,
	CARTCIRCLE_TYPE_SYNC_TIME_SAME = 1,
	REV_CARTCIRCLE_TYPE_SYNC = 0x8000
}cartcircle_sync;
//--------------------------------------------------------------------------
//define the motion planning cart line posture calculate method
typedef enum
{
	CARTLINE_TYPE_AXIS_ANGLE = 0,
	CARTLINE_TYPE_ROT_MATRIX = 1,
	REV_CARTLINE_TYPE = 0x8000
}cartLine_method;
//--------------------------------------------------------------------------
//define the motion planning cart circle posture calculate method
typedef enum
{
	CARTCIRCLE_TYPE_AXIS_ANGLE = 0,
	CARTCIRCLE_TYPE_ROT_MATRIX = 1,
	REV_CARTCIRCLE_TYPE = 0x8000
}cartCircle_method;


//---------------------------------------------------------------------------
#pragma pack(4)
typedef struct
{
	double profile_vel; 				// max vel, 1/s
	double profile_acc; 				// max acc, 1/s^2
	double profile_tacc; 				// s
	double profile_dec; 				// max dec, 1/s^2
	double profile_tdec; 				// s
	double profile_sample_time; 		// s
}motion_profile;
#pragma pack()
//---------------------------------------------------------------------------
#pragma pack(4)
typedef struct
{
	double pos;							// pos
	double vel;							// vel
	double acc;							// acc
	double dec;							// dec
	double jerk_acc; 					// unit: 1/s^3
	double jerk_dec; 					// unit: 1/s^3
	double total_interval; 				// total = tacc + t1 + tacc + t3 + tdec + t2 + tdec
										/*
										* ACC:[0,Tacc]
										* -ACC:[Tacc+T1,Tacc+T1+Tacc]
										* -DEC:[Tacc+T1+Tacc+T3, Tacc+T1+Tacc+T3+Tdec]
										* DEC:[Tacc+T1+Tacc+T3+Tdec+T2, Tacc+T1+Tacc+T3+Tdec+T2+Tdec]
										*/
	Uint32 time_series[8];
	double jerk_series[8];
	double acc_series[8];
	double vel_series[8];
	double pos_series[8];
}motion_trajectory;
#pragma pack()

//---------------------------------------------------------------------------
#pragma pack(4)
typedef struct
{
	/*type
	* 0: none
	* 1: auto
	* 2: percentage of interval
	* 4: distance, millimeter
	* */
	Uint16 type;
	Uint16 is_ready_blending;
	Uint16 is_need_blending;
	double parameters;
	Uint32 blending_start_time;
	Uint32 blending_time_length;
	double blending_time_length_inv;

}motion_blending;
#pragma pack()
//---------------------------------------------------------------------------
#pragma pack(4)
typedef struct
{
	Uint16   type;
}motion_trigger;
#pragma pack()

//---------------------------------------------------------------------------
#pragma pack(4)
typedef struct
{
	/* Type,
	* 0: World;
	* 1: Base;
	* 2: Tool;
	* 4: RobotRoot;
	* 8: Frange;
	* */
	Uint16   frame_type;
	Uint16   axis_angle_flag;							//use for judge whether the axis is existed
														//--------------------------------------------
														// Pose
														// position (mm) + orientation: Roll, Pitch, Yaw, (r-p-y, in rad)(x,y,z)(fix frame,x first)
	double    pose_start[6];
	double    pose_end[6];
	double    pose_aux1[6];								//use for circle
	double    pose_aux2[6];								//use for circle
	double    pose_unit_vector[6];						//unit vector
	double    arc_length;								//path lenth(line or arc)
	double 	  arc_lenth_trans;
	//--------------------------------------------
	//axis angle presentation
	matxx	 axis_vector;								//axis presentation 3*1 matrix,use for pose ,angle-axis presentation
	double 	 angle_lenth;								//angle presentation:unit :rad

														//--------------------------------------------
														//rotate matrix presentation
	matxx    rotation_matxx;							//rotate matrix presentation
	matxx    d_rotation_matxx;							//rotate matrix presentation'
	matxx    rot_start;
	matxx    rot_end;
	matxx 	 rot_tmp;
	matxx    rot_tmp1;

	matxx 	 tmp31;
	matxx 	 P0;
	matxx    P0P1;
	matxx    P0P2;
	matxx    u;
	matxx	 v;
	matxx    w;

	//--------------------------------------------
	//rotation axis unit vector for Circle Motion
	matxx    unit_vector;								//use for angle-axis presentation's axis,used for circle

														//--------------------------------------------
														// Joint (Axis) in degree
	matxx    joint_start_vector;
	matxx    joint_end_vector;
	matxx    joint_unit_vector;
	matxx    joint_temp_vector;
	matxx    joint_pos;

	//--------------------------------------------
	matxx    center_vector;    							//the vector of circle center to xyz original
	matxx    cartesian_xyz;								//circle cartesian pos
	matxx 	 cartesian_xyz_vel;							//circle cartesian vel
	double   circle_radius;
	double   circle_radius_inv;

}motion_pose_information;
#pragma pack()
//---------------------------------------------------------------------------
#pragma pack(4)
typedef struct
{
	/* state:
	* 0: unused or finish motion;
	* 1: used;
	* 2: ready to run;
	* 3: running;
	* 255: unavailable;
	* */
	Uint16 						state;
	/* type:
	* 0: none
	* 1: P2P
	* 2: LIN
	* 4: CIRC
	* */
	Uint16 						motion_type;
	Uint16                      temp_type;										//use for save the resume motion block type ,resume need change to p2p,
	Uint16 						motion_block_id;
	prfcoord_type				prfcoord_type;

	//--------------------------------------------------------------------
	jointp2p_sync 				jointp2p_sync;									//0: use time scale method, 1: use time equal method
	cartline_sync			    cartline_sync;									//0: use time scale method, 1: use time equal method
	cartcircle_sync			    cartcircle_sync;								//0: use time scale method, 1: use time equal method

																				//--------------------------------------------------------------------
	cartLine_method				cartline_method;								//0: use axis angle,1: use rotate matrix
	cartCircle_method			cartcircle_method;								//0: use axis angle,1: use rotate matrix

																				//--------------------------------------------------------------------
	motion_pose_information    	pose;

	motion_profile    			profile;										//this used for Cartesian translation motion profile
	motion_profile				prf_cart_rot;								    //this used for Cartesian rotate motion profile
	motion_profile				prf_joint[JOINT_COORDINATE_NUM];				//this used for every joint motion profile

	motion_blending    			blending;
	motion_blending    			blending_cart_rot;
	//motion_trigger    		trigger;

	motion_trajectory   	 	trajectory;  									//this used for  cartesian translation trajectory
	motion_trajectory   	 	traj_cart_rot;  								//this used for  cartesian rotate trajectory
	motion_trajectory			traj_joint[JOINT_COORDINATE_NUM];				//this used for every joint trajectory

																				//---------------------------------------------------------------------
	Uint8						delay_enable;  									// 1: enable, 0:disable ,the delay is before this block run.
																				// if this block need blending ,disable delay and let delay_enable=0
	Uint32						delay_time;										// Unit ms.  in 4ms interrupt ,Unit 4ms

	Uint8						arc_flag;										// 1:total circle ;0 arc

} motion_block;
#pragma pack()
//---------------------------------------------------------------------------
#pragma pack(4)
typedef struct TRAJECTORY_MODULE
{
	Uint32    		 current_time;
	Uint32 			 abs_time;
	motion_block     motion_block_buffer[MOTION_MODULE_DEFAULT_MOTION_BLOCK_SIZE];

	//pointer for buffer,which is convenience for programming
	motion_block*    motion_block_execute;
	motion_block*    motion_block_new_insert;
	motion_block*    motion_block_calculate;

	motion_block	 motion_block_tmp;

	double 			 motion_profile[6];
	double			 prf_cart_rot[6];
	//---------------------------------------------------------------
	// function pointer define
	int16(*pfInitTrajectoryModule)(struct TRAJECTORY_MODULE * m_traj, robot_config_module *p_config, Uint8 id);	//trajectory module initialization

	int16(*pfTrajSynCmdWithFd)(struct TRAJECTORY_MODULE * m_traj, robot_config_module *p_config, \
		cartesian_module* p_cart, joint_module* p_joint);									//sync trajectory pos cmd using pos feedback

	int16(*pfCalcMotionProfile)(struct TRAJECTORY_MODULE * m_traj, robot_config_module *m_cfg);					//calc motion profile

	int16(*pfInsertNewMotionBlock)(struct TRAJECTORY_MODULE *m_traj, robot_config_module* p_config, \
		double* pose_end, double* pose_aux, double* profile, double* blend, Uint8 motion_type, Uint8 id);

	int16(*pfTrajectoryGenerator)(struct TRAJECTORY_MODULE* p_trajectory, cartesian_module* p_cart, joint_module* p_joint, \
		robot_config_module* p_config, Uint8 id);
}trajectory_module;
#pragma pack()

//---------------------------------------------------------------------------
int16 InitTrajectoryModule(trajectory_module *m_traj, robot_config_module *p_config, Uint8 id);

int16 TrajSynCmdWithFd(trajectory_module * m_traj, robot_config_module *p_config, cartesian_module* p_cart, joint_module* p_joint);

int16 CalcMotionProfile(trajectory_module * m_traj, robot_config_module *m_cfg);

void calculate_motion_parameters(robot_config_module* m_cfg, motion_block* p_motion);

int16 InsertNewMotionBlock(trajectory_module *m_traj, robot_config_module* p_config,double* pose_start, double* pose_end, \
	double* pose_aux, double* profile, double* blend, Uint8 motion_type, Uint8 id);

int16  TrajectoryGenerator(trajectory_module* p_trajectory, cartesian_module* p_cart, joint_module* p_joint, \
	robot_config_module* p_config, Uint8 id);

void calculate_acceleration_from_profile(double t, double jerk, double acc_init, double* acc_t);
void calculate_velocity_from_profile(double t, double jerk, double acc_init, double vel_init, double* vel_t);
void calculate_position_from_profile(double t, double jerk, double acc_init, double vel_init, double pos_init, double* pos_t);
//---------------------------------------------------------------------------

#endif /* TRAJECTORY_GENERATOR_H_ */
