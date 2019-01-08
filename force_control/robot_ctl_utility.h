#pragma once
/*
* robot_ctl_utility.h
*
*  Created on: Mar 15, 2018
*      Author: root
*/

#ifndef ROBOT_CONTROL_UTILITY_INCLUDE_ROBOT_CTL_UTILITY_H_
#define ROBOT_CONTROL_UTILITY_INCLUDE_ROBOT_CTL_UTILITY_H_

//-------------------------------------------------------------------
/*
* debug information
* */
#ifdef QKM_MOTION_DEBUG
#define MOTION_DBG_MSG(txt) printk(txt)
#define MOTION_DBG_PRINTF(txt,...) printk(txt,__VA_ARGS__)
#else
#define MOTION_DBG_MSG(txt)
#define MOTION_DBG_PRINTF(txt,...)
#endif

#define MOTION_DBG_FUNC_IN() \
	MOTION_DBG_PRINTF(">> Entering function: >> %s \n",__func__)
#define MOTION_DBG_FUNC_OUT() \
	MOTION_DBG_PRINTF("<< Exiting function: << %s \n",__func__)
#define MOTION_DBG_FUNC_LINE_FUNC() \
	MOTION_DBG_PRINTF("At line:%d in function:%s\n",__LINE__,__func__)

//--------------------------------------------------------------------
#define	ROBOT_DEVICE_MAX_NUM								2				//define robot controller can controlled max robot device. for example 2
//--------------------------------------------------------------------
//---------------------------------------------------------------------------------

#define	CART_COORDINATE_DIM									6									//cartesian coordinate dimension
#define	JOINT_COORDINATE_NUM								8									//joint coordinate max number
#define	TRAJ_MAX_NUM										JOINT_COORDINATE_NUM + 1			//which support 8 joint and 1 cartesian

//---------------------------------------------------------------------------------

#define ROBOT_CONFIG_MODULE_DEFAULT_JOINT_DIM    			((Uint8) 2)
#define ROBOT_CONFIG_MODULE_DEFAULT_JOINT_DIM_OF_SCARA    	((Uint8) 4)
#define ROBOT_CONFIG_MODULE_CARTESIAN_6_DIM    				((Uint8) 6)
#define ROBOT_CONFIG_MODULE_CARTESIAN_TRANS_3_DIM    		((Uint8) 3)
#define ROBOT_CONFIG_MODULE_CARTESIAN_ROTATE_3_DIM   	 	((Uint8) 3)

//---------------------------------------------------------------------------------
// Inverse Kinematics Configuration
//---------------------------------------------------------------------------------
#define		KINEMATICS_LIBRARY_CONFIG_RIGHTY		(0x0001)
#define		KINEMATICS_LIBRARY_CONFIG_LEFTY			(0x0002)

//---------------------------------------------------------------------------------
// trajectory module Configuration
//---------------------------------------------------------------------------------
#define TRAJECTORY_MODULE_MOTION_BLOCK_STATE_IDL 					((Uint8) 0)
#define TRAJECTORY_MODULE_MOTION_BLOCK_STATE_USED 					((Uint8) 1)
#define TRAJECTORY_MODULE_MOTION_BLOCK_STATE_SOFT_ESTOP 			((Uint8) 2)
#define TRAJECTORY_MODULE_MOTION_BLOCK_STATE_HARD_ESTOP 			((Uint8) 3)
#define TRAJECTORY_MODULE_MOTION_BLOCK_STATE_RUNNING 				((Uint8) 4)


#define TRAJECTORY_MODULE_MOTION_TYPE_NONE 							((Uint8) 0)
#define TRAJECTORY_MODULE_MOTION_TYPE_P2P 							((Uint8) 1)
#define TRAJECTORY_MODULE_MOTION_TYPE_LIN 							((Uint8) 2)
#define TRAJECTORY_MODULE_MOTION_TYPE_CIRCL 						((Uint8) 4)

#define TRAJECTORY_MODULE_BLENDING_NONE 							((Uint8) 0)
#define TRAJECTORY_MODULE_BLENDING_AUTO 							((Uint8) 1)
#define TRAJECTORY_MODULE_BLENDING_PERCENTAGE 						((Uint8) 2)
#define TRAJECTORY_MODULE_BLENDING_DISTANCE 						((Uint8) 4)

#define TRAJECTORY_MODULE_DIMENSION_OF_XYZ_FRAME 					((Uint8) 3)
#define TRAJECTORY_MODULE_DIMENSION_OF_ROT_FRAME 					((Uint8) 3)
//-------------------------------------------------------------------
#define MOTION_MODULE_CONSTANT_MIN_POSITIVE 		((double)1.0E-10)
#define MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD  	((double)0.017453292519943)
#define MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE  	((double)57.295779513082323)
#define MOTION_MODULE_CONSTANT_PI  					((double)3.1415926535897931)
#define MOTION_MODULE_CONSTANT_PERCENTAGE 			((double)0.01)

//-------------------------------------------------------------------
#define MOTION_MODULE_DEFAULT_ROBOT_TYPE_SCARAJOINT    0
#define MOTION_MODULE_DEFAULT_ROBOT_TYPE_SCARA    	   4
//-------------------------------------------------------------------

#define MOTION_MODULE_SAMPLE_TIME    				((double) 0.001) /* unit: s  */
#define MOTION_MODULE_SAMPLE_FREQUENCY    			((double) 250)  /* unit: Hz */

#define MOTION_MODULE_DEFAULT_MOTION_BLOCK_SIZE    3

//----------------------------------------------------------------------

/*
* error information
* */
#define MOTION_MODULE_EXECUTE_OK 						0x0000
#define MOTION_MODULE_EXECUTE_INTERNAL_ERR 				0x000f

#define MOTION_MODULE_ERROR_NULL_INPUIT_HANDLE    		1UL
#define MOTION_MODULE_ERROR_INVALID_ID    				2UL
#define MOTION_MODULE_ERROR_COMPUTE_PROFILE_ERR    		3UL
#define MOTION_MODULE_ERROR_LAST_MOTION_UNFINISHED    	4UL
#define MOTION_MODULE_ERROR_FAILED_INITIALISATION    	5UL
#define MOTION_MODULE_ERROR_MEMORY_ALLOCATE    			6UL
#define MOTION_MODULE_ERROR_PARAMETERS    				7UL
#define MOTION_MODULE_ERROR_INVERSE_KINEMATICS_FAILED   8UL
#define MOTION_MODULE_ERROR_MOITION_IN_JOINT_OVERLIMIT  9UL

//----------------------------------------------------------------------
#define MOTION_MODULE_ERROR_BIT_SHFIT 					16

#define CARTESIAN_MODULE_ERROR_ID						(0x0001 << MOTION_MODULE_ERROR_BIT_SHFIT)
#define JOINT_MODULE_ERROR_ID							(0x0002 << MOTION_MODULE_ERROR_BIT_SHFIT)
#define ROBOT_MODULE_ERROR_ID							(0x0003 << MOTION_MODULE_ERROR_BIT_SHFIT)
#define TRAJECTORY_MODULE_ERROR_ID  					(0x0004 << MOTION_MODULE_ERROR_BIT_SHFIT)


//----------------------------------------------------------------------

#endif /* ROBOT_CONTROL_UTILITY_INCLUDE_ROBOT_CTL_UTILITY_H_ */
