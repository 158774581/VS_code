
#include "robot_device.h"

//-------------------------------------------------------------------------------------------
int16	InitRobotDevice(robot_device * m_robot_dev, Uint8 id)
{
	m_robot_dev->cfg.prm.robot_type	= MOTION_MODULE_DEFAULT_ROBOT_TYPE_SCARA;
	//robot_mgr* tmp = &m_robot_dev->mgr;
	//---------------------------------------------------------
	//initial robot configure module
	InitRobotConfigModule(&(m_robot_dev->cfg), id);

	//-------------------------------------------------------------------------------------
	//if (SEV_PRM_FROM_EX_MEM_EN == ENABLE)
	{

	}
	//else
	{
		m_robot_dev->prm.enable = 1;											//default enable
		m_robot_dev->prm.dev_mode = ROBOT_DEV_MODE_RUN;  							//default run mode

	}

	m_robot_dev->id = id;
	m_robot_dev->type = m_robot_dev->cfg.prm.robot_type;				//get type from config module
	m_robot_dev->status = ROBOT_DEV_STS_IDLE;							//default idle status.
	m_robot_dev->home_flag = ROBOT_NO_HOME;								//init no home
	m_robot_dev->station_id = id;											//default rtn station id is equal to the id

																			//---------------------------------------------------------
	m_robot_dev->pfInitRobotDevice = InitRobotDevice;
	m_robot_dev->pfRobotDeviceStatus = RobotDeviceStatus;
	//---------------------------------------------------------
	//init robot manager
	//InitRobotMgr(&tmp, id);

	//---------------------------------------------------------
	//init robot module
	InitRobotModule(&(m_robot_dev->module), &(m_robot_dev->cfg), id);

	//----------------------------------------------------------------
	//inti robot mode
	//InitRobotMode(&m_robot_dev->mode, id);

	//init conveyor tracking
	InitConveyorTracking(&m_robot_dev->cnv_trck,id);

	return 0;
}
//--------------------------------------------------------------------------------------------------------------------
int16 RobotDeviceStatus(robot_device * m_robot, Uint8 id)
{
	Uint8 index = 0;
	//------------------------------------------------------------------
	//check id
	if (id >= ROBOT_DEVICE_MAX_NUM || id < 0)
	{
		printf("robot id %d is illegal!\n", id);
		return -1;
	}

	//-------------------------------------------------------------------------------
	// Show staus Information
	printf("********Robot %d States********\n", id);
	//printf("robot mode ID: %d;\n", m_robot->mode.mode);
	printf("Homed: %d;\n", m_robot->home_flag);
	printf(".........................................\n");
	//-------------------------------------------------------------------------------
	// Show Cartesian Information
	printf("********Cartesian Information********\n");
	printf("Pose CMD = [");

	for (index = 0; index < m_robot->cfg.cart_dim; index++)
	{
		printf("%f ", m_robot->module.cart.cart_pos_cmd[index]);
	}
	printf("];\n");
	//-------------------------------------------------------------------------------
	printf("Pose FB  = [");
	for (index = 0; index < m_robot->cfg.cart_dim; index++)
	{
		printf("%f ", m_robot->module.cart.cart_pos_fd[index]);
	}
	printf("];\n");
	printf("........................................ \n");
	//-------------------------------------------------------------------------------

	// Show Joint Information
	printf("********Joint Information********\n");
	printf("Joint CMD = [");

	for (index = 0; index < m_robot->cfg.joint_dim; index++) {
		printf("%f ", m_robot->module.joint[index].joint_pos_cmd);
	}
	printf("];\n");
	//-------------------------------------------------------------------------------
	printf("Joint FB  = [");
	for (index = 0; index < m_robot->cfg.joint_dim; index++) {
		printf("%f ", m_robot->module.joint[index].joint_pos_fd);
	}
	printf("];\n");
	printf("...\n");
	//-------------------------------------------------------------------------------
	return 0;
}