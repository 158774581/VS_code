#pragma once
/*
* location.h
*
*  Created on: Nov 29, 2018
*      Author: root
*/

#ifndef ROBOT_CONTROL_LOCATION_INCLUDE_LOCATION_H_
#define ROBOT_CONTROL_LOCATION_INCLUDE_LOCATION_H_
//----------------------------------------------------------------------------
#include "type_def.h"
#include "robot_macro_def.h"
//----------------------------------------------------------------------------

#define			CONFIGURE_RIGHT				0x01
#define			CONFIGURE_LEFT				0x02
#define			CONFIGURE_ABOVE				0x04
#define			CONFIGURE_BELOW				0x08
#define			CONFIGURE_FLIP				0x10
#define			CONFIGURE_NOFLIP			0x20
#define			CONFIGURE_SINGLE			0x1000			//只有这个在笛卡尔运动过程中是合法的。
//----------------------------------------------------------------------------
// 定义点位的类型，默认是笛卡尔方式
//----------------------------------------------------------------------------
typedef enum
{
	LOCATION_TYPE_CARTESIAN = 0,								//笛卡尔坐标系的位置和姿态
	LOCATION_TYPE_ANGLES = 1,								//角度
}LOCATION_TYPE;
//----------------------------------------------------------------------------
// 转化错误是否报异常问题，两种类型相互转化，并不是单方向的
//----------------------------------------------------------------------------
typedef enum
{
	CONVERT_MODE_EXCEPTION = 0,								//如果为0，产生一个异常。默认是0
	CONVERT_MODE_IGNORE = 1,								//如果为1，那么忽略转化的报错.
}CONVERT_MODE;

//----------------------------------------------------------------------------
// 定义zclearance的方向，是根据world坐标系的z，还是工具坐标系的z轴
//----------------------------------------------------------------------------
typedef enum
{
	ZCLEARANCE_TYPE_TOOL = 0,								//根据工具坐标系的z轴 ，负z轴
	ZCLEARANCE_TYPE_WORLD = 1,								//根据world坐标系的z
}ZCLEARANCE_TYPE;
//----------------------------------------------------------------------------
// 定义画整圆的时候，末端点的姿态如何变化，是保持和起始一致，还是根据辅助点1作为末端点
//----------------------------------------------------------------------------
typedef enum
{
	CIRCLE_POSE_SAME = 0,								//根据起点的姿态不变化
	CIRCLE_POSE_AUX1 = 1,								//根据辅助点1的姿态作为最终的姿态
}CIRCLE_POSE_TYPE;

//----------------------------------------------------------------------------
// 定义笛卡尔下的点位 其中每个值都是相对于参考坐标系的值
// 固定角，pitch 绕x轴，yaw 绕y轴，roll绕z轴 固定角，就是按顺序根据固定的坐标系进行旋转，
// 先绕x，再绕y，再绕z
//----------------------------------------------------------------------------
#pragma pack(4)
typedef struct LOCATION_CART
{
	double									x;									// x mm
	double 									y;									// y mm
	double									z;									// x mm
	double 									pitch;								// rotate about x deg
	double									yaw;								// rotate about y deg
	double 									roll;								// rotate about z deg
	double									ext1;								// 额外轴1 mm/deg
	double									ext2;								// 额外轴2 mm/deg
	double									ext3;								// 额外轴3 mm/deg
}location_cart;
#pragma pack(0)
//----------------------------------------------------------------------------
//定义笛卡尔下的联合体，可以使用数组的方式方便循环赋值，并且可以单独使用其中的某个值，比较方便
//----------------------------------------------------------------------------
typedef union
{
	location_cart		single;
	double				array[LOCATION_CART_DIMENSION_MAX];
}location_union;

//----------------------------------------------------------------------------
// location POS 包括笛卡尔坐标和angle,因为后续可能有只改动pos，而不影响location中其他配置的要求
//----------------------------------------------------------------------------
#pragma pack(4)
typedef struct LOCATION_POS
{
	double 									axis[ROBOT_MAX_AXIS];				//定义轴的点位， mm 或者 deg
	location_union							cart;								//定义笛卡尔下的点位
}location_pos;
#pragma pack(0)
//-------------------------------------------------------------------------------------
//定义config，不同的机器人有不同的配置，不同的配置导致在同一个空间位置上有不同的关节角度可以对应
//-------------------------------------------------------------------------------------
typedef struct	LOCATION_CONFIG
{
	Uint32		RIGHT : 1;												// 右肩配置 shoulder
	Uint32		LEFT : 1;												// 右肩配置 shoulder
	Uint32		ABOVE : 1;												// 肘在腕的上方 elbow above twist
	Uint32		BELOW : 1;												// 肘在腕的下方

	Uint32		FLIP : 1;												// 手腕翘起 wrist
	Uint32		NOFLIP : 1;												// 手腕放下
	Uint32		SINGLE : 1;												// 限制手腕轴在正负180度之间，而不是整个运动范围内

	Uint32		rsvd : 25;												// rsvd
}location_config;


typedef	union
{
	Uint32					all;
	location_config			bit;
}loc_cfg;

//----------------------------------------------------------------------------
// 定义点位 轴的点位根据实际机器人类型使用
//----------------------------------------------------------------------------
#pragma pack(4)
typedef struct ROBOT_LOCATION
{
	LOCATION_TYPE 							type;								//点位类型变量
	ZCLEARANCE_TYPE 						zWorld;								//间隔高度依赖的坐标系
	CIRCLE_POSE_TYPE						cirPoseType;						//走整圆时候的终点姿态
	Uint16									kinesol;							//根据该标志判断如何处理，1 忽略正逆解中的转化错误，比如关节超限等，0：如果出错就报错
	loc_cfg									config;								//配置
	double									zclearance;							//z方向间隔距离 mm
	location_pos							loc_pos;							//location信息，包括笛卡尔和关节
	location_cart							ref_frame;							//笛卡尔坐标系下点位的参考坐标系
}robot_location;
#pragma pack(0)
#endif /* ROBOT_CONTROL_LOCATION_INCLUDE_LOCATION_H_ */
