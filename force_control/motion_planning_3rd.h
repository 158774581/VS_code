#pragma once
/*

* motion_planning_3rd.h

*

*  Created on: 2018.3.9

*      Author: bwang

*/

//----------------------------------------------------------------------------------------------------

#ifndef ROBOT_CONTROL_MOTION_PLANNING_INCLUDE_MOTION_PLANNING_3RD_H_

#define ROBOT_CONTROL_MOTION_PLANNING_INCLUDE_MOTION_PLANNING_3RD_H_



//-------------------------------------------------------------------

#include "type_def.h"

#include <math.h>

//----------------------------------------------------------------------------------------------------



//----------------------------------------------------------------------------------------------------

typedef struct motion_planning_prm

{

	double pos;							//position				unit: mm		degree

	double spd_max;						//max speed				unit: mm/s		degree/s

	double acc_max;						//max acceleration		unit: mm/s^2    degree/s^2

	double t_acc;						//sample time			unit: s

	double dec_max;						//max_deceleration      unit: mm/s^2    degree/s^2

	double t_dec;						//sameple time			unit: s

}MOTION_PLANNING_PRM;

typedef struct motion_planning_nonzero_prm
{
	double pos;							//position				unit: mm		degree

	double spd_max;						//max speed				unit: mm/s		degree/s

	double acc_max;						//max acceleration		unit: mm/s^2    degree/s^2

	double t_acc;						//sample time			unit: s

	double vs;							//init vel				unit: mm/s^2    degree/s^2

	double ve;							//terminal vel			unit: mm/s^2    degree/s^2

	double scale;						//time scale			

	double secant_eps;					//calculate eps of NewTon(secant) iteration 

	int16 max_itera_time;				//max ierate time

}MOTION_PLANNING_NON_ZERO_PRM;

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
}motion_trajectory_nonzero;
//----------------------------------------------------------------------------------------------------

//define struct for 3 order motion planning,

//because the jerk is constant,so the varibles are acc, vel ,pos

typedef struct motion_planning_3rd

{

	MOTION_PLANNING_PRM prm;				//input parameter define

	double accMulTacc;					//acc multiply t_acc,

	double decMulTdec;					//dec multiply t_dec,this two part use for compare which bigger,

										//default calculate use accMulTacc > decMulTdec

										//-------------------------------------

	double boundary_condition_1;			//six boundary condition

	double boundary_condition_2;

	double boundary_condition_3;

	double boundary_condition_4;

	double boundary_condition_5;

	double boundary_condition_6;



	//-------------------------------------

	double t_acc_ceil_d;					//recalculate the time using n * sample time ,which is ceiled real time

	double t_dec_ceil_d;					//recalculate the time using n * sample time ,which is ceiled real time

	double t_acc_ceil;					//ceil(real time/sample time) = n

	double t_dec_ceil;					//ceil(real time/sample time) = n





	double t1_ceil_d;						//recalculate the time using n * sample time ,which is ceiled real time

	double t2_ceil_d;						//recalculate the time using n * sample time ,which is ceiled real time

	double t1_ceil;						//ceil(real time/sample time) = n

	double t2_ceil;						//ceil(real time/sample time) = n



	double t3_ceil_d;						//recalculate the time using n * sample time ,which is ceiled real time

	double t3_ceil;                       //ceil(real time/sample time) = n



	double t_all_d;                       //real time

	double t_all;                         //real time/sample



	double Jacc;                          //jerk acc 

	double Jdec;                          //jerk dec



	double vmax_tmp;						//used for after ceil 

	double acc_tmp;						//used for after ceil 

	double dec_tmp;						//used for after ceil



} MOTION_PLANNING_3RD;

//----------------------------------------------------------------------------------------------------

//input used for external input. these parameters are saved in the MOTION_PLANNING_3RD struct

int16 motion_planning_3rd(MOTION_PLANNING_PRM* input, MOTION_PLANNING_3RD* m_mp3rd, double sample_time);

double DisplacemenInDeltaV(double v, double dv, double Acc, double J);

int16 NewTonSecantMethodToFindRoot(double x0, double x1, double s, double vs, double ve, double Acc, double J, double eps, int16 max_iter, double* root);

int16 motion_planning_nonzero(MOTION_PLANNING_NON_ZERO_PRM* input, MOTION_PLANNING_3RD* m_mp3rd, motion_trajectory_nonzero* motion_tra, double sample_time);
//----------------------------------------------------------------------------------------------------

#endif /* ROBOT_CONTROL_MOTION_PLANNING_INCLUDE_MOTION_PLANNING_3RD_H_ */


