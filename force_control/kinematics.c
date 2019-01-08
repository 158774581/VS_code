/*
* kinematics_cfg.c
*
*  Created on: Mar 22, 2018
*      Author: root
*/

#include "type_def.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "matrix.h"
#include "robot_ctl_utility.h"
#include "robot_config.h"
#include "cartesian_module.h"
#include "joint_module.h"
#include "kinematics.h"
#include "lib_scara.h"
//---------------------------------------------------------------
//init kinematics function
//---------------------------------------------------------------
int16 InitKinematics(kinematics * handle, robot_config_module* handle_config, Uint8 id)
{
	Uint8 i;
	handle_config->prm.robot_type = MOTION_MODULE_DEFAULT_ROBOT_TYPE_SCARA;
	//-------------------------------------------------------------------------------------
	//if (SEV_PRM_FROM_EX_MEM_EN == ENABLE)
	{

	}
	//else
	{
		for (i = 0; i<JOINT_COORDINATE_NUM; ++i)
		{
			handle->prm.encoder_lineNum[i] = 131072;
			handle->prm.gear_ratio[i] = 1;
			handle->prm.lenth_of_joint[i] = 250;
			handle->prm.screw_pitch[i] = 25;
			handle->prm.dh_param_a[i] = 0.0;
			handle->prm.dh_param_d[i] = 0.0;
			handle->prm.dh_param_alpha[i] = 0.0;
			handle->prm.dh_param_theta[i] = 0.0;

			handle->prm.mech_couple[i] = 0.0;
		}
	}

	//-------------------------------------------------------------------------------------
	//assign spmatrix
	switch (handle_config->prm.robot_type)
	{
	case MOTION_MODULE_DEFAULT_ROBOT_TYPE_SCARA:
	case MOTION_MODULE_DEFAULT_ROBOT_TYPE_SCARAJOINT:
	{
		//-----------------------------------
		//matrix is as follow for scara
		//[ G1
		//		G2
		//          G3  K
		//              G4 ]
		//-----------------------------------

		//		encoder_lineNum *gear_ratio
		//G = ----------------------------
		// 					360.0

		//for G3 ,this is special,because joint3 is translation move

		//		encoder_lineNum*gear_ratio
		//G3 = ----------------------------
		//				screw_pitch

		handle->spg[0] = handle->prm.dir[0] * handle->prm.encoder_lineNum[0] * handle->prm.gear_ratio[0] / 360.0;
		handle->spg[1] = handle->prm.dir[1] * handle->prm.encoder_lineNum[1] * handle->prm.gear_ratio[1] / 360.0;
		handle->spg[2] = handle->prm.dir[2] * handle->prm.encoder_lineNum[2] * handle->prm.gear_ratio[2] / handle->prm.screw_pitch[2];
		handle->spg[3] = handle->prm.dir[3] * handle->prm.encoder_lineNum[3] * handle->prm.gear_ratio[3] / 360.0;

		//			encoder_lineNum[3]*gear_ratio[2]
		//k = ---------------------------------------
		//						360.0
		//use encoder_lineNum[3] means joint4 couple with joint 3, joint4 angle change to pulse and multiply joint3's gear ratio
		handle->spk[0] = -handle->prm.dir[3] * handle->prm.encoder_lineNum[3] * handle->prm.gear_ratio[2] / 360.0;

		//-----------------------------------
		//inverse matrix is as follow for scara
		//[ 1/G1
		//		1/G2
		//          1/G3  -K/(G3*G4)
		//                  1/G4       ]
		//-----------------------------------
		for (i = 0; i< handle_config->joint_dim; ++i)
		{
			if (fabs(handle->spg[i]) > MOTION_MODULE_CONSTANT_MIN_POSITIVE)
			{
				handle->inv_spg[i] = 1.0 / handle->spg[i];
			}
			else
			{
				//default
				handle->inv_spg[i] = 1;
				printf("the inverse of g[%d] is not ok! \n", i);
				return -1;
			}
		}

		handle->inv_spk[0] = -handle->spk[0] * handle->inv_spg[2] * handle->inv_spg[3];

		//------------------------------------------------------------------------
		handle->spmatxx.row = 4;
		handle->spmatxx.column = 4;
		handle->spmatxx.non_zero = 5;

		//this four is gear ratio
		handle->spmatxx.data[0].row = 0;
		handle->spmatxx.data[0].column = 0;
		handle->spmatxx.data[0].value = handle->spg[0];

		handle->spmatxx.data[1].row = 1;
		handle->spmatxx.data[1].column = 1;
		handle->spmatxx.data[1].value = handle->spg[1];

		handle->spmatxx.data[2].row = 2;
		handle->spmatxx.data[2].column = 2;
		handle->spmatxx.data[2].value = handle->spg[2];

		handle->spmatxx.data[3].row = 3;
		handle->spmatxx.data[3].column = 3;
		handle->spmatxx.data[3].value = handle->spg[3];

		//couple item
		handle->spmatxx.data[4].row = 2;
		handle->spmatxx.data[4].column = 3;
		handle->spmatxx.data[4].value = handle->spk[0];


		//------------------------------------------------------------------------
		handle->inv_spmatxx.row = 4;
		handle->inv_spmatxx.column = 4;
		handle->inv_spmatxx.non_zero = 5;

		//this four is gear ratio
		handle->inv_spmatxx.data[0].row = 0;
		handle->inv_spmatxx.data[0].column = 0;
		handle->inv_spmatxx.data[0].value = handle->inv_spg[0];

		handle->inv_spmatxx.data[1].row = 1;
		handle->inv_spmatxx.data[1].column = 1;
		handle->inv_spmatxx.data[1].value = handle->inv_spg[1];

		handle->inv_spmatxx.data[2].row = 2;
		handle->inv_spmatxx.data[2].column = 2;
		handle->inv_spmatxx.data[2].value = handle->inv_spg[2];

		handle->inv_spmatxx.data[3].row = 3;
		handle->inv_spmatxx.data[3].column = 3;
		handle->inv_spmatxx.data[3].value = handle->inv_spg[3];

		//couple item
		handle->inv_spmatxx.data[4].row = 2;
		handle->inv_spmatxx.data[4].column = 3;
		handle->inv_spmatxx.data[4].value = handle->inv_spk[0];

		break;
	}
	default:
	{
		break;
	}
	}

	//-------------------------------------------------------------------------------------
	handle->pfFK = NULL;
	handle->pfIK = NULL;
	handle->pfFJ = NULL;
	handle->pfIJ = NULL;
	handle->pfJ2M = NULL;
	handle->pfM2J = NULL;

	handle->pfInitKinematics = InitKinematics;
	handle->pfConfigKinematics = configKinematics;
	//-------------------------------------------------------------------------------------
	return 0;
}

//---------------------------------------------------------------
//configure kinematics function according to the robot type
//---------------------------------------------------------------
int16 configKinematics(kinematics* handle, robot_config_module* handle_config, Uint8 id)
{
	//---------------------------------------------------------------
	switch (handle_config->prm.robot_type)
	{
	case MOTION_MODULE_DEFAULT_ROBOT_TYPE_SCARA:
		handle->pfFK = ScaraFK;
		handle->pfIK = ScaraIK;
		handle->pfFJ = ScaraFJ;
		handle->pfIJ = ScaraIJ;
		handle->pfM2J = ScaraM2J;
		handle->pfJ2M = ScaraJ2M;
		break;
	case MOTION_MODULE_DEFAULT_ROBOT_TYPE_SCARAJOINT:
	default:
		handle->pfM2J = ScaraM2J;
		handle->pfJ2M = ScaraJ2M;
		break;
	}
	//---------------------------------------------------------------

	return 0;
}

