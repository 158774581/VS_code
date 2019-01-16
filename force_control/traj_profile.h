#pragma once
/*
* profile.h
*
*  Created on: Nov 28, 2018
*      Author: root
*/

#ifndef ROBOT_CONTROL_ROBOT_MODULE_ROBOT_PROFILE_TRAJECTORY_GENERATOR_INCLUDE_ROBOT_PROFILE_H_
#define ROBOT_CONTROL_ROBOT_MODULE_ROBOT_PROFILE_TRAJECTORY_GENERATOR_INCLUDE_ROBOT_PROFILE_H_
//----------------------------------------------------------------------------
#include "type_def.h"
#include "robot_macro_def.h"
//----------------------------------------------------------------------------
// 点到点运动采用的方式
//----------------------------------------------------------------------------
typedef enum
{
	P2P_JOINT_INTEPOLATE_METHOD = 0,								//点到点采用关节插补，不关心末端走的路径是否是直线。
	P2P_CARTESIAN_INTEPOLATE_LINE_METHOD = 1									//点到点采用空间直线插补
}P2P_STRAIGHT;
//----------------------------------------------------------------------------
// 运动到位类型
//----------------------------------------------------------------------------
typedef enum
{
	INRANGE_TYPE_NO_STOP = 0,								//不停止
	INRANGE_TYPE_STOP_CMD_REACHED = 1,								//命令到达，不考虑位置误差限制
	INRANGE_TYPE_STOP_WITH_CONSTRAINT = 2								    //考虑位置误差限制
}INRANGE_TYPE;
//----------------------------------------------------------------------------
//blending的类型
//----------------------------------------------------------------------------
typedef enum
{
	BLENDING_TYPE_NONE = 0,														//没有blending
	BLENDING_TYPE_AUTO = 1,														//自动blending，第一段的减速段和第二段的加速段进行blending
	BLENDING_TYPE_TIME = 2,														//时间blending，根据给定的时间，
	BLENDING_TYPE_DISTANCE = 3,														//距离blending，根据给定的距离
	BLENDING_TYPE_ROUND_CORNER = 4															//圆角blending，根据给定的中间点到blending路径上最近点的距离
}BLENDING_TYPE;

//----------------------------------------------------------------------------
//速度，加速度，加速度斜坡时间等可以通过参数相互耦合，自动变化
//百分比变化都从1---100，一般不超出100，当然也可以超出，具体问题具体分析
//这些参数都有默认值
//使用s型曲线可以减少共振，减少稳定的调整时间，但是会增加运动的时间，因为平均加速度没有梯形大
//----------------------------------------------------------------------------
#pragma pack(4)
typedef struct PROFILE_PRM
{
	double vel; 						// vel, 			 % ,用于平移速度和旋转速度，假如vel2是0的时候
	double vel2; 						// vel for rotation, % ,用于旋转速度，假如vel2不是零的时候，一般用于有一个比较快的旋转部分，可以将vel2设置的低一些
	double acc; 						// acc, 			 %  加速度
	double accramp; 					// 加速度斜坡时间		 s  设为0是梯形，不为0是s型
	double dec; 						// dec, 			 %  减速度
	double decramp; 					// 减速度斜坡时间       s  设为0是梯形，不为0是s型
	Uint16 straight;					// 直线标志 			 no unit，假如走空间直线，设置为1，否则走关节的插补类似于PTP，默认false
	Uint16 inrange;						// 到位的位置误差设置，  负数表示不停，0 表示指令到达，大于0表示收到误差限制才认为是到位， 100表示严格的误差限制，小数字表示不那么严格的误差限制，
										// 目前不清楚误差限制和该数值的关系。
}profile_prm;
#pragma pack(0)
//----------------------------------------------------------------------------
//profile的某些值的限制值，设置的规划会和这些限制值进行比较，不能超出这些限制值，默认是100%
//----------------------------------------------------------------------------
#pragma pack(4)
typedef struct PROFILE_PRM_MAX
{
	double vel_max; 						// vel, 			 % 最大限制速度百分比，可以超过百分比，比如某些轻载的情况下，是可以设置超过100%的
	double acc_max; 						// acc, 			 % 最大限制加速度百分比，
	double dec_max; 						// dec,              % 最大限制减速度百分比，
}profile_prm_max;
#pragma pack(0)

//----------------------------------------------------------------------------
//profile的基准值，是物理单位的值，其他的百分比是基于这些值的，对于笛卡尔来说，index=0 表示平移，index=1表示旋转
//----------------------------------------------------------------------------
#pragma pack(4)
typedef struct PROFILE_PRM_BASE
{
	double joint_vel_base[ROBOT_MAX_AXIS]; 					// 关节基准速度, 也就是100%对应的速度	        mm/s or degree/s
	double cart_vel_base[ROBOT_MAX_CART]; 					// 笛卡尔基准速度, 也就是100%对应的速度		mm/s or degree/s
															// 第一个参数是平移运动的笛卡尔速度，	从第二参数开始是根据具体的运动学模型不同而不同，一般第二个可以是旋转的速度
	double joint_acc_base[ROBOT_MAX_AXIS]; 					// 关节基准加速度, 也就是100%对应的加速度	    mm/s^2 or degree/s^2
	double joint_dec_base[ROBOT_MAX_AXIS]; 					// 关节基准减速度, 也就是100%对应的减速度	    mm/s^2 or degree/s^2
	double cart_acc_base[ROBOT_MAX_CART]; 					// 笛卡尔基准加速度, 也就是100%对应的加速度		mm/s^2 or degree/s^2
															// 第一个参数是平移运动的笛卡尔加速度，	从第二参数开始是根据具体的运动学模型不同而不同，一般第二个可以是旋转的加速度
	double cart_dec_base[ROBOT_MAX_CART]; 					// 笛卡尔基准减速度, 也就是100%对应的减速度	mm/s^2 or degree/s^2
															// 第一个参数是平移运动的笛卡尔减速度，	从第二参数开始是根据具体的运动学模型不同而不同，一般第二个可以是旋转的减速度
}profile_prm_base;
#pragma pack(0)

//---------------------------------------------------------------------------
//定义blending参数的联合体,使用不同类型的时候，对应输入的参数不同，有的类型不需要输入参数，比如auto
//---------------------------------------------------------------------------
typedef union
{
	double	 			time_per;							//时间百分比			%
	double	 			distance;							//距离    			mm
	double				corner_distance;					//圆角距离 			mm
}blend_prm_union;
//---------------------------------------------------------------------------
//定义blending参数
//---------------------------------------------------------------------------
#pragma pack(4)
typedef struct	BLEND_PRM
{
	BLENDING_TYPE 		type;								//类型
	blend_prm_union		value;
}blend_prm;
#pragma pack()
//---------------------------------------------------------------------------
#pragma pack(4)
typedef struct PROFILE_BLENDING
{
	//-----------------------------------------------------------------------
	blend_prm			prm;								//blending参数
															//-----------------------------------------------------------------------
	Uint16 				needed_flag;						//需要blending标志
	Uint16				ready_flag;							//准备好标志
	Uint16				force_close_flag;					//某些情况下需要强制关闭blending
															//-----------------------------------------------------------------------
	Uint32 				start_t;							//blending开始时间
	Uint32 				length_t;							//blending时间长度
	double 				length_t_inv;						//时间倒数

}profile_blending;
#pragma pack()
//----------------------------------------------------------------------------
#pragma pack(4)
typedef struct ROBOT_PROFILE
{
	profile_prm_max     max;						//最大百分比
	profile_prm     	def;						//默认百分比
	profile_prm_base    base;						//基准值，物理量
	INRANGE_TYPE 		inrange_type;				//到位类型
	Uint16 				couple_spd_flag;			//速度耦合标志
	double				couple_spd_per;				//速度耦合百分比 % 设置为0，不耦合，设置为100%或者更大，因为设置为小于100%，容易误解，速度变大，加速度，减速度，斜坡时间都对应减小，
													//这看起来很奇怪。设置为200%，速度增加一倍的时候，其他都double，速度减小一半的时候，其他的也变为一半
	double		     	r_spd_percent;				//机器人速度百分比（系统速度百分比，机器人速度百分比，运动块百分比）
													//-------------------------------------------------------------------------------------------------
													// function pointer define
													//-------------------------------------------------------------------------------------------------
	int16(*pfInitRobotProfile)(struct ROBOT_PROFILE * m_prf, Uint8 id);	// robot profile path initialization
}robot_profile;
#pragma pack(0)


int16 InitRobotProfile(robot_profile* m_prf, Uint8 id);

void calculate_acceleration_from_profile(double t, double jerk, double acc_init, double* acc_t);
void calculate_velocity_from_profile(double t, double jerk, double acc_init, double vel_init, double* vel_t);
void calculate_position_from_profile(double t, double jerk, double acc_init, double vel_init, double pos_init, double* pos_t);

void ProfileCheckLimit(profile_prm_max* max, profile_prm* prf);

//----------------------------------------------------------------------------
#endif /* ROBOT_CONTROL_ROBOT_MODULE_ROBOT_PROFILE_TRAJECTORY_GENERATOR_INCLUDE_ROBOT_PROFILE_H_ */
