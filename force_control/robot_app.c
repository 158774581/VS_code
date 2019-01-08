#include "robot_app.h"
robot gRobot;
int16	InitRobot(robot * m_robot)
{
	//-----------------------------------------------------
	Uint8	i;

	//-----------------------------------------------------
	//init all the robot device
	//-----------------------------------------------------
	for (i = 0; i < ROBOT_DEVICE_MAX_NUM; ++i)
	{
		InitRobotDevice(&(m_robot->robot_dev[i]), i);
	}

	//-----------------------------------------------------
	m_robot->device_num = ROBOT_DEVICE_MAX_NUM;

	//-----------------------------------------------------
	m_robot->pfInitRobot = InitRobot;

	//-----------------------------------------------------
	return 0;

}