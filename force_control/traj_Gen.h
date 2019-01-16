#pragma once
/*
* traj_Gen.h
*
*  Created on: mar 15, 2018
*      Author: root
*/

#ifndef ROBOT_CONTROL_TRAJECTORY_GENERATOR_INCLUDE_TRAJECTORY_GENERATOR_H_1
#define ROBOT_CONTROL_TRAJECTORY_GENERATOR_INCLUDE_TRAJECTORY_GENERATOR_H_1
//--------------------------------------------------------------------------
#include "traj_profile.h"
#include "type_def.h"
#include "robot_macro_def.h"
#include "location.h"

//插入运动块时需要传入的数据
//---------------------------------------------------------------------------
#pragma pack(4)
typedef struct MB_INSERT
{
	//-------------------------------------------------------------------------------
	Uint8 		type;			//插入模块的运动类型
	//-------------------------------------------------------------------------------
	//该段运动和结束点定义
	//-------------------------------------------------------------------------------
	robot_location    		loc_end;
	//-------------------------------------------------------------------------------
	// 辅助点定义，用于圆弧和整圆
	//-------------------------------------------------------------------------------
	robot_location    		loc_aux;
	robot_location    		loc_aux1;
	//-------------------------------------------------------------------------------
	//定义运动规划参数，包括blending
	//-------------------------------------------------------------------------------
	profile_prm				prf;
	blend_prm				bld;
	double					start_vel[6];
	double					end_vel[6];
	//-------------------------------------------------------------------------------
	//指定开始位置
	//-------------------------------------------------------------------------------
	Uint8					pos_start_assigned;  
	robot_location    		loc_start;
}mb_insert;


#endif /* TRAJECTORY_GENERATOR_H_1_ */
