/*
* dynamic_module.h
*
*  Created on: Sep 3, 2018
*      Author: hqi
*/

#include "stdafx.h"
#include "robot_ctl_utility.h"
#include "type_def.h"
#include "matrix.h"
#include "DynamicProfile.h"
#include "dynamic2.h"
#include "DynamicModule.h"

int16  InitDynamics(Dynamics* p_dyn)
{
	Uint8 i;
	double zeros[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

	for (i = 0; i < 4; i++)
	{
		matxx_malloc(&p_dyn->pstar[i], 3, 1);
		matxx_malloc(&p_dyn->R[i], 3, 3);
		matxx_malloc(&p_dyn->Rt[i], 3, 3);
		matxx_malloc(&p_dyn->Fm[i], 3, 1);
		matxx_malloc(&p_dyn->Nm[i], 3, 1);
	}
	matxx_malloc(&p_dyn->R[4], 3, 3);
	matxx_malloc(&p_dyn->Rt[4], 3, 3);

	matxx_malloc(&p_dyn->torque, 4, 1);
	matxx_malloc(&p_dyn->wd, 3, 1);
	matxx_malloc(&p_dyn->w, 3, 1);
	matxx_malloc(&p_dyn->vd, 3, 1);
	matxx_malloc(&p_dyn->vcd, 3, 1);
	matxx_malloc(&p_dyn->fext, 3, 1);
	matxx_malloc(&p_dyn->Text, 3, 1);
	matxx_malloc(&p_dyn->f, 3, 1);
	matxx_malloc(&p_dyn->T, 3, 1);

	matxx_malloc(&p_dyn->A, 4, 4);
	matxx_malloc(&p_dyn->U1, 4, 4);
	matxx_malloc(&p_dyn->U2, 4, 4);
	matxx_assign(&p_dyn->A, zeros, 16);
	matxx_assign(&p_dyn->U1, zeros, 16);
	matxx_assign(&p_dyn->U2, zeros, 16);
	(*(p_dyn->U1.point))[0]	    = 1;
	(*(p_dyn->U1.point + 1))[1] = 1;
	(*(p_dyn->U1.point + 2))[2] = 1;
	(*(p_dyn->U1.point + 3))[3] = 1;
	(*(p_dyn->U2.point))[0]	    = 1;
	(*(p_dyn->U2.point + 1))[1] = 1;
	(*(p_dyn->U2.point + 2))[2] = 1;
	(*(p_dyn->U2.point + 3))[3] = 1;

	(*(p_dyn->w.point))[0] = 0;
	(*(p_dyn->w.point))[1] = 0;
	(*(p_dyn->w.point))[2] = 0;
	(*(p_dyn->wd.point))[0] = 0;
	(*(p_dyn->wd.point))[1] = 0;
	(*(p_dyn->wd.point))[2] = 0;
	(*(p_dyn->vd.point))[0] = 0;
	(*(p_dyn->vd.point))[1] = 0;
	(*(p_dyn->vd.point))[2] = 9.81;
	(*(p_dyn->vcd.point))[0] = 0;
	(*(p_dyn->vcd.point))[1] = 0;
	(*(p_dyn->vcd.point))[2] = 0;
	(*(p_dyn->torque.point))[0] = 0;
	(*(p_dyn->torque.point))[1] = 0;
	(*(p_dyn->torque.point))[2] = 0;
	(*(p_dyn->fext.point))[0] = 0;
	(*(p_dyn->fext.point))[1] = 0;
	(*(p_dyn->fext.point))[2] = 0;
	(*(p_dyn->Text.point))[0] = 0;
	(*(p_dyn->Text.point))[1] = 0;
	(*(p_dyn->Text.point))[2] = 0;
	(*(p_dyn->f.point))[0] = 0;
	(*(p_dyn->f.point))[1] = 0;
	(*(p_dyn->f.point))[2] = 0;
	(*(p_dyn->T.point))[0] = 0;
	(*(p_dyn->T.point))[1] = 0;
	(*(p_dyn->T.point))[2] = 0;

	matxx_malloc(&p_dyn->tmp31, 3, 1);
	matxx_malloc(&p_dyn->temp31, 3, 1);
	matxx_malloc(&p_dyn->tp31, 3, 1);

	return 0;
};
