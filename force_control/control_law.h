#pragma once
/*
* control_law.h
*
*  Created on: Jul 27, 2018
*      Author: hqi
*/

#ifndef FORCE_CONTROL_CONTROL_LAW_INCLUDE_CONTROL_LAW_H_
#define FORCE_CONTROL_CONTROL_LAW_INCLUDE_CONTROL_LAW_H_
//----------------------------------------------------------------
#include "robot_ctl_utility.h"
#include "matrix.h"
#include "dynamic_profile.h"
//----------------------------------------------------------------
#pragma pack(4)
typedef struct CARTESIAN_IMPEDANCE_CONTROL_LAW
{
	matxx		Md;  //desired inertia matrix.
	matxx		Bd;  //desired damping matrix.
	matxx		Kd;  //desired stiffness matrix.
	matxx		Fext;  //cartesian extra force for environment.

}cartsian_impedance_control_law;
#pragma pack()

#pragma pack(4)
typedef struct JOINT_IMPEDANCE_CONTROL_LAW
{
	matxx		Kp;  //proportion gain
	matxx		Kd;  //differential gain.
	matxx		Text;  //extra torque.

}joint_impedance_control_law;
#pragma pack()

#pragma pack(4)
typedef struct ROBUST
{
	Uint8		robust_enable;
	matxx		P;  //symmetric positive definite matrix satisfying the Lyapunov equation A^T*P + P*A = −Q
	matxx		Q;  //symmetric positive definite matrix satisfying the Lyapunov equation A^T*P + P*A = −Q
	matxx		B;  //
	matxx		delta;  //robust compensation 
	matxx		e;  //tracking error

	matxx		tmp24;  //
	matxx		tmp21;  //

	double      max_e;
}robust;
#pragma pack()

typedef enum
{
	CARTSIAN_IMPEDANCE_CONTROL_LAW = 0,
	JOINT_IMPEDANCE_CONTROL_LAW = 1


}law_type;


#pragma pack(4)
typedef struct CONTROL_LAW
{
	law_type							law_type;  //the type of control law.
	cartsian_impedance_control_law 		cart_law;  //impedance control in cartesian.
	joint_impedance_control_law			joint_law;  //impedance control in joint.

	robust								robust;  //robust control.

	int16(*pfInitControlLaw)(struct CONTROL_LAW* p_law);

}control_law;
#pragma pack()


int16 InitControlLaw(control_law* p_law);

int16 robust_control(control_law* ctl_law, joint_profile* p_joint);

//extern dynamic_module gDymMod;

#endif /* FORCE_CONTROL_CONTROL_LAW_INCLUDE_CONTROL_LAW_H_ */
