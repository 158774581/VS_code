#pragma once
/*
* solver.h
*
*  Created on: Mar 13, 2018
*      Author: root
*/
#ifndef ROBOT_CONTROL_MATH_INCLUDE_SOLVER_H_
#define ROBOT_CONTROL_MATH_INCLUDE_SOLVER_H_

#include "type_def.h"


//------------------------------------------------------------
enum quadratic_root_type
{
	QUADRATIC_NO_ROOTS = 0,
	QUADRATIC_TWO_SAME_ROOTS,
	QUADRATIC_TWO_DIFF_ROOTS
}QUADRATIC_ROOT_TYPE;

//------------------------------------------------------------
//quadratic equation define
//------------------------------------------------------------

typedef struct quadratic_solver
{
	double c0;
	double c1;
	double r0;
	double r1;
	int16  root_type;

}QUADRATIC_SOLVER;

//------------------------------------------------------------
void quadratic_solver(QUADRATIC_SOLVER* m_qs);
//------------------------------------------------------------

enum cubic_root_type
{
	CUBIC_NO_REAL_ROOTS = 0,
	CUBIC_THREE_SAME_REAL_SPECIAL_EQU0_ROOTS,
	CUBIC_THREE_SAME_REAL_SPECIAL_BIG0_ROOTS,
	CUBIC_THREE_SAME_REAL_SPECIAL_LESS0_ROOTS,
	CUBIC_ONLY_ONE_REAL_SPECIAL_R0_ROOTS,
	CUBIC_TWO_SAME_ONE_DIFF_REAL_SPECIAL_R0_ROOTS,
	CUBIC_TWO_SAME_ONE_DIFF_REAL_SPECIAL_R1_ROOTS,
	CUBIC_TWO_SAME_ONE_DIFF_REAL_SPECIAL_BIGGER_ROOTS,
	CUBIC_TWO_SAME_ONE_DIFF_REAL_R0_ROOTS,
	CUBIC_TWO_SAME_ONE_DIFF_REAL_R1_ROOTS,
	CUBIC_ONE_REAL_TWO_COMPLEX_ROOTS,
	CUBIC_THREE_DIFF_REAL_R0_ROOTS,
	CUBIC_THREE_DIFF_REAL_R1_ROOTS,
	CUBIC_THREE_DIFF_REAL_R2_ROOTS
}CUBIC_ROOT_TYPE;


//------------------------------------------------------------
//cubic equation define
//------------------------------------------------------------

typedef struct cubic_solver
{
	double c0; //b/a
	double c1; //
	double c2;
	double r0;
	double r1;
	double r2;
	int16  root_type;

}CUBIC_SOLVER;
//------------------------------------------------------------
int16 cubic_solver(CUBIC_SOLVER* m_cs);
//------------------------------------------------------------

enum quartic_root_type
{
	QUARTIC_NO_REAL_ROOTS = 0,
	QUARTIC_SPECIAL_REAL_ROOT_R0,
	QUARTIC_SPECIAL_REAL_ROOT_R2,
	QUARTIC_SPECIAL_REAL_ROOT_BIGGER,
	QUARTIC_TWO_SAME_01_REAL_ROOTS,
	QUARTIC_TWO_SAME_23_REAL_ROOTS,
	QUARTIC_TWO_DIFF_01_REAL_ROOTS,
	QUARTIC_TWO_DIFF_23_REAL_ROOTS,
	QUARTIC_TWO_SAME_TWO_SAME_REAL_ROOTS,
	QUARTIC_TWO_SAME_TWO_DIFF_REAL_ROOTS,
	QUARTIC_TWO_DIFF_TWO_SAME_REAL_ROOTS,
	QUARTIC_TWO_DIFF_TWO_DIFF_REAL_ROOTS
}QUARTIC_ROOT_TYPE;


//------------------------------------------------------------
//quartic equation define
//------------------------------------------------------------

typedef struct quartic_solver
{
	double c0; //b/a
	double c1; //
	double c2;
	double c3;
	double r0;
	double r1;
	double r2;
	double r3;
	int16  root_type;

}QUARTIC_SOLVER;
//------------------------------------------------------------
int16 quartic_solver(QUARTIC_SOLVER* m_qs);
//------------------------------------------------------------


#endif /* ROBOT_CONTROL_MATH_INCLUDE_SOLVER_H_ */

