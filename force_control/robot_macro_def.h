#pragma once
/*
* robot_macro_def.h
*
*  Created on: Nov 28, 2018
*      Author: root
*/

#ifndef ROBOT_CONTROL_UTILITY_INCLUDE_ROBOT_MACRO_DEF_H_
#define ROBOT_CONTROL_UTILITY_INCLUDE_ROBOT_MACRO_DEF_H_


//-------------------------------------------------------------------

#define 		ROBOT_MAX_AXIS   									12								//一个机器人最大的轴数
#define 		ROBOT_MAX_CART   									ROBOT_MAX_AXIS - 2 				//一个机器人最大的笛卡尔数，默认是最大轴数减2，xyz平移运动看做一个cartesian，每个
#define			ROBOT_MAX_MOTOR										ROBOT_MAX_AXIS					//一个机器人的最大电机数，默认和轴数一致。

#define			LOCATION_CART_DIMENSION_MAX							9								//笛卡尔下点位最大维数，加上三个额外的轴，一共是9
//-------------------------------------------------------------------

#define 		PRM_FROM_EX_MEM_EN  								ENABLE							//参数从外部存储设备进行初始化，读取外部的参数文件

//-------------------------------------------------------------------
#define 		MOTION_MODULE_CONSTANT_MIN_POSITIVE 				((double)1.0E-10)				//定义最小的数，比这个数小认为是0
//-------------------------------------------------------------------
#define 		TRAJECTORY_MODULE_DIMENSION_OF_XYZ_FRAME 			((Uint8) 3)						//笛卡尔位置的维数
#define 		TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME 			((Uint8) 3)						//笛卡尔姿态的维数
#endif /* ROBOT_CONTROL_UTILITY_INCLUDE_ROBOT_MACRO_DEF_H_ */
