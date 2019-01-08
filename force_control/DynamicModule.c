/*
* dynamic_module.h
*
*  Created on: Sep 3, 2018
*      Author: hqi
*/

#include "stdafx.h"
#include "math.h"
#include "DynamicModule.h"

int16 initDynamicMdl(DynamicMdl* m_Dmdl)
{
	int8 i;
	//init dynamic prm
	m_Dmdl->dprm.robot_type			= 4;
	m_Dmdl->dprm.link_num			= 4;
	matxx_malloc(&m_Dmdl->dprm.gravity, 3, 1);
	(*(m_Dmdl->dprm.gravity.point))[0] = 0;
	(*(m_Dmdl->dprm.gravity.point))[1] = 0;
	(*(m_Dmdl->dprm.gravity.point))[2] = 9.81;

	for (i = 0; i < m_Dmdl->dprm.link_num;i++)
	{
		m_Dmdl->dprm.Kv[i]=2.0;
		m_Dmdl->dprm.Kd[i]=1.0;
	}
	//link 1
	m_Dmdl->dprm.links[0].type		= 0;
	m_Dmdl->dprm.links[0].a			= 0.25;
	m_Dmdl->dprm.links[0].alpha		= 0;
	m_Dmdl->dprm.links[0].d			= 0;
	m_Dmdl->dprm.links[0].theta		= 0;

	matxx_malloc(&m_Dmdl->dprm.links[0].I, 3, 3);
	(*(m_Dmdl->dprm.links[0].I.point+0))[0] = 0;
	(*(m_Dmdl->dprm.links[0].I.point+1))[0] = 0;
	(*(m_Dmdl->dprm.links[0].I.point+2))[0] = 0;
	(*(m_Dmdl->dprm.links[0].I.point))[1]	= 0;
	(*(m_Dmdl->dprm.links[0].I.point+1))[1] = 0;
	(*(m_Dmdl->dprm.links[0].I.point+2))[1] = 0;
	(*(m_Dmdl->dprm.links[0].I.point))[2]	= 0;
	(*(m_Dmdl->dprm.links[0].I.point+1))[2] = 0;
	(*(m_Dmdl->dprm.links[0].I.point+2))[2] = 0.029008;

	matxx_malloc(&m_Dmdl->dprm.links[0].r, 3, 1);
	(*(m_Dmdl->dprm.links[0].r.point + 0))[0] = 0.11652-0.25;
	(*(m_Dmdl->dprm.links[0].r.point + 0))[1] = 0;
	(*(m_Dmdl->dprm.links[0].r.point + 0))[2] = -0.02891;

	m_Dmdl->dprm.links[0].m = 2.247;
	m_Dmdl->dprm.links[0].Jm = 0.11652e-6;
	m_Dmdl->dprm.links[0].G = 80;
	m_Dmdl->dprm.links[0].B = 1.48e-3;
	m_Dmdl->dprm.links[0].Tc[0] = 0.395;
	m_Dmdl->dprm.links[0].Tc[1] = -0.435;
	m_Dmdl->dprm.links[0].qlim[0] = -MOTION_MODULE_CONSTANT_PI/2;
	m_Dmdl->dprm.links[0].qlim[1] = MOTION_MODULE_CONSTANT_PI / 2;

	//link 2
	m_Dmdl->dprm.links[1].type = 0;
	m_Dmdl->dprm.links[1].a = 0.25;
	m_Dmdl->dprm.links[1].alpha = MOTION_MODULE_CONSTANT_PI;
	m_Dmdl->dprm.links[1].d = 0;
	m_Dmdl->dprm.links[1].theta = 0;

	matxx_malloc(&m_Dmdl->dprm.links[1].I, 3, 3);
	(*(m_Dmdl->dprm.links[1].I.point + 0))[0] = 0;
	(*(m_Dmdl->dprm.links[1].I.point + 1))[0] = 0;
	(*(m_Dmdl->dprm.links[1].I.point + 2))[0] = 0;
	(*(m_Dmdl->dprm.links[1].I.point))[1] = 0;
	(*(m_Dmdl->dprm.links[1].I.point + 1))[1] = 0;
	(*(m_Dmdl->dprm.links[1].I.point + 2))[1] = 0;
	(*(m_Dmdl->dprm.links[1].I.point))[2] = 0;
	(*(m_Dmdl->dprm.links[1].I.point + 1))[2] = 0;
	(*(m_Dmdl->dprm.links[1].I.point + 2))[2] = 0.079803;

	matxx_malloc(&m_Dmdl->dprm.links[1].r, 3, 1);
	(*(m_Dmdl->dprm.links[1].r.point + 0))[0] = 0.11661 - 0.25;
	(*(m_Dmdl->dprm.links[1].r.point + 0))[1] = 0;
	(*(m_Dmdl->dprm.links[1].r.point + 0))[2] = -0.08375;

	m_Dmdl->dprm.links[1].m = 11.078;
	m_Dmdl->dprm.links[1].Jm = 104.4268e-6;
	m_Dmdl->dprm.links[1].G = 50;
	m_Dmdl->dprm.links[1].B = 0.817e-3;
	m_Dmdl->dprm.links[1].Tc[0] = 0.126;
	m_Dmdl->dprm.links[1].Tc[1] = -0.071;
	m_Dmdl->dprm.links[1].qlim[0] = -MOTION_MODULE_CONSTANT_PI / 2;
	m_Dmdl->dprm.links[1].qlim[1] = MOTION_MODULE_CONSTANT_PI / 2;

	//link 3
	m_Dmdl->dprm.links[2].type = 1;
	m_Dmdl->dprm.links[2].a = 0;
	m_Dmdl->dprm.links[2].alpha = 0;
	m_Dmdl->dprm.links[2].d = 0;
	m_Dmdl->dprm.links[2].theta = 0;

	matxx_malloc(&m_Dmdl->dprm.links[2].I, 3, 3);
	(*(m_Dmdl->dprm.links[2].I.point + 0))[0] = 0;
	(*(m_Dmdl->dprm.links[2].I.point + 1))[0] = 0;
	(*(m_Dmdl->dprm.links[2].I.point + 2))[0] = 0;
	(*(m_Dmdl->dprm.links[2].I.point))[1]	  = 0;
	(*(m_Dmdl->dprm.links[2].I.point + 1))[1] = 0;
	(*(m_Dmdl->dprm.links[2].I.point + 2))[1] = 0;
	(*(m_Dmdl->dprm.links[2].I.point))[2]	  = 0;
	(*(m_Dmdl->dprm.links[2].I.point + 1))[2] = 0;
	(*(m_Dmdl->dprm.links[2].I.point + 2))[2] = 0;

	matxx_malloc(&m_Dmdl->dprm.links[2].r, 3, 1);
	(*(m_Dmdl->dprm.links[2].r.point + 0))[0] = 0;
	(*(m_Dmdl->dprm.links[2].r.point + 0))[1] = 0;
	(*(m_Dmdl->dprm.links[2].r.point + 0))[2] = 0;

	m_Dmdl->dprm.links[2].m = 0.664;
	m_Dmdl->dprm.links[2].Jm = 21.9e-6;
	m_Dmdl->dprm.links[2].G = 1.8;
	m_Dmdl->dprm.links[2].B = 0.817e-3;
	m_Dmdl->dprm.links[2].Tc[0] = 0.126;
	m_Dmdl->dprm.links[2].Tc[1] = -0.071;
	m_Dmdl->dprm.links[2].qlim[0] = -0.5;
	m_Dmdl->dprm.links[2].qlim[1] = 0.5;

	//link 4
	m_Dmdl->dprm.links[3].type = 0;
	m_Dmdl->dprm.links[3].a = 0;
	m_Dmdl->dprm.links[3].alpha = 0;
	m_Dmdl->dprm.links[3].d = 0;
	m_Dmdl->dprm.links[3].theta = 0;

	matxx_malloc(&m_Dmdl->dprm.links[3].I, 3, 3);
	(*(m_Dmdl->dprm.links[3].I.point + 0))[0] = 0;
	(*(m_Dmdl->dprm.links[3].I.point + 1))[0] = 0;
	(*(m_Dmdl->dprm.links[3].I.point + 2))[0] = 0;
	(*(m_Dmdl->dprm.links[3].I.point))[1] = 0;
	(*(m_Dmdl->dprm.links[3].I.point + 1))[1] = 0;
	(*(m_Dmdl->dprm.links[3].I.point + 2))[1] = 0;
	(*(m_Dmdl->dprm.links[3].I.point))[2] = 0;
	(*(m_Dmdl->dprm.links[3].I.point + 1))[2] = 0;
	(*(m_Dmdl->dprm.links[3].I.point + 2))[2] = 9.37e-4;

	matxx_malloc(&m_Dmdl->dprm.links[3].r, 3, 1);
	(*(m_Dmdl->dprm.links[3].r.point + 0))[0] = 0;
	(*(m_Dmdl->dprm.links[3].r.point + 0))[1] = 0;
	(*(m_Dmdl->dprm.links[3].r.point + 0))[2] = 0;

	m_Dmdl->dprm.links[3].m = 0;
	m_Dmdl->dprm.links[3].Jm = 6.27e-6;
	m_Dmdl->dprm.links[3].G = 50;
	m_Dmdl->dprm.links[3].B = 0.817e-3;
	m_Dmdl->dprm.links[3].Tc[0] = 0.126;
	m_Dmdl->dprm.links[3].Tc[1] = -0.071;
	m_Dmdl->dprm.links[3].qlim[0] = -MOTION_MODULE_CONSTANT_PI / 2;
	m_Dmdl->dprm.links[3].qlim[1] = MOTION_MODULE_CONSTANT_PI / 2;

	m_Dmdl->PfinitDynamicMdl = initDynamicMdl;

	InitDynamics(&m_Dmdl->Dyn);
	return 0;
};

int16 NewtonEuler(Dynamics * p_dyn, DynamicPrf* p_prf, DynamicPrm* p_prm)
{
	int8 i;
	double d;
	double st, ct,sa,ca;
	double zero[3] = { 0, 0, 0};
	double gravity[3] = { 0, 0, 9.81 };
	double fc;
	matxx	q, qd, qdd;

	matxx_malloc(&q, 4, 1);
	matxx_malloc(&qd, 4, 1);
	matxx_malloc(&qdd, 4, 1);

	(*(q.point))[0] = 0;
	(*(q.point))[1] = 0.5516;
	(*(q.point))[2] = 0;
	(*(q.point))[3] = 1.4867;

	(*(qd.point))[0] = 0;
	(*(qd.point))[1] = -1.0001;
	(*(qd.point))[2] = 0;
	(*(qd.point))[3] = 3.0004;

	(*(qdd.point))[0] = 0;
	(*(qdd.point))[1] = -0.1167;
	(*(qdd.point))[2] = 0;
	(*(qdd.point))[3] = 0.3502;

	//init some variables, compute the link rotation matrices
	matxx_assign(&p_dyn->w, zero, 3);
	matxx_assign(&p_dyn->wd, zero, 3);
	matxx_assign(&p_dyn->vd, gravity, 3);

	for (i = 0; i < p_prm->link_num; i++)
	{
		sa			= sin(p_prm->links[i].alpha);
		ca			= cos(p_prm->links[i].alpha);
		//rotation
		if (p_prm->links[i].type == 0)
		{
			d		= p_prm->links[i].d;
			st		= sin((*(q.point))[i]);
			ct		= cos((*(q.point))[i]);
		}
		//prismatic
		else
		{
			d		= (*(q.point))[i];
			st		= sin(p_prm->links[i].theta);
			ct		= cos(p_prm->links[i].theta);
		}
		(*(p_dyn->pstar[i].point))[0]		= p_prm->links[i].a;
		(*(p_dyn->pstar[i].point))[1]		= d*sin(p_prm->links[i].alpha);
		(*(p_dyn->pstar[i].point))[2]		= d*cos(p_prm->links[i].alpha);
		
		(*(p_dyn->R[i].point))[0]			= ct;
		(*(p_dyn->R[i].point+1))[0]			= -st*ca;
		(*(p_dyn->R[i].point+2))[0]			= st*sa;
		(*(p_dyn->R[i].point))[1]			= st;
		(*(p_dyn->R[i].point+1))[1]			= ct*ca;
		(*(p_dyn->R[i].point+2))[1]			= -ca*sa;
		(*(p_dyn->R[i].point))[2]			= 0;
		(*(p_dyn->R[i].point+1))[2]			= sa;
		(*(p_dyn->R[i].point+2))[2]			= ca;

		(*(p_dyn->Rt[i].point))[0]			= (*(p_dyn->R[i].point))[0];
		(*(p_dyn->Rt[i].point + 1))[0]		= (*(p_dyn->R[i].point))[1];
		(*(p_dyn->Rt[i].point + 2))[0]		= (*(p_dyn->R[i].point))[2];
		(*(p_dyn->Rt[i].point))[1]			= (*(p_dyn->R[i].point+1))[0];
		(*(p_dyn->Rt[i].point + 1))[1]		= (*(p_dyn->R[i].point+1))[1];
		(*(p_dyn->Rt[i].point + 2))[1]		= (*(p_dyn->R[i].point+1))[2];
		(*(p_dyn->Rt[i].point))[2]			= (*(p_dyn->R[i].point+2))[0];
		(*(p_dyn->Rt[i].point + 1))[2]		= (*(p_dyn->R[i].point+2))[1];
		(*(p_dyn->Rt[i].point + 2))[2]		= (*(p_dyn->R[i].point+2))[2];

	}
	(*(p_dyn->R[p_prm->link_num].point))[0]		= (*(p_dyn->Rt[p_prm->link_num].point))[0]		= 1;
	(*(p_dyn->R[p_prm->link_num].point + 1))[0] = (*(p_dyn->Rt[p_prm->link_num].point + 1))[0]	= 0;
	(*(p_dyn->R[p_prm->link_num].point + 2))[0] = (*(p_dyn->Rt[p_prm->link_num].point + 2))[0]	= 0;
	(*(p_dyn->R[p_prm->link_num].point))[1]		= (*(p_dyn->Rt[p_prm->link_num].point))[1]		= 0;
	(*(p_dyn->R[p_prm->link_num].point + 1))[1] = (*(p_dyn->Rt[p_prm->link_num].point + 1))[1]	= 1;
	(*(p_dyn->R[p_prm->link_num].point + 2))[1] = (*(p_dyn->Rt[p_prm->link_num].point + 2))[1]	= 0;
	(*(p_dyn->R[p_prm->link_num].point))[2]		= (*(p_dyn->Rt[p_prm->link_num].point))[2]		= 0;
	(*(p_dyn->R[p_prm->link_num].point + 1))[2] = (*(p_dyn->Rt[p_prm->link_num].point + 1))[2]	= 0;
	(*(p_dyn->R[p_prm->link_num].point + 2))[2] = (*(p_dyn->Rt[p_prm->link_num].point + 2))[2]	= 1;
	//the forward recursion
	for (i = 0; i < p_prm->link_num; i++)
	{
		if (p_prm->links[i].type == 0)
		{
			//w X (z0*qd)
			(*(p_dyn->tp31.point))[0] = 0;
			(*(p_dyn->tp31.point))[1] = 0;
			(*(p_dyn->tp31.point))[2] = (*(qd.point))[i];
			matxx_cross(&p_dyn->w, &p_dyn->tp31, &p_dyn->tmp31);

			matxx_k_mac(1.0, &p_dyn->w, &p_dyn->tp31);
			//wd + z0*qdd
			(*(p_dyn->temp31.point))[0] = 0;
			(*(p_dyn->temp31.point))[1] = 0;
			(*(p_dyn->temp31.point))[2] = (*(qdd.point))[i];
			matxx_k_mac(1.0, &p_dyn->wd, &p_dyn->temp31);

			//wd=Rt*(w X (z0*qd) + wd+zo*qdd)
			matxx_k_mac(1.0, &p_dyn->temp31, &p_dyn->tmp31);
			matxx_multiply(&p_dyn->Rt[i], &p_dyn->tmp31, &p_dyn->wd);

			//w=Rt*(w + zo*qd)
			matxx_multiply(&p_dyn->Rt[i], &p_dyn->tp31, &p_dyn->w);

			//vd=wd X pstar + w X (w X pstar) + Rt*vd.
			matxx_multiply(&p_dyn->Rt[i], &p_dyn->vd, &p_dyn->tp31);
			matxx_cross(&p_dyn->w, &p_dyn->pstar[i], &p_dyn->temp31);
			matxx_cross(&p_dyn->w, &p_dyn->temp31, &p_dyn->tmp31);
			matxx_k_mac(1.0, &p_dyn->tmp31, &p_dyn->tp31);
			matxx_cross(&p_dyn->wd, &p_dyn->pstar[i], &p_dyn->vd);
			matxx_k_mac(1.0, &p_dyn->tp31, &p_dyn->vd);
		}
		else
		{
			//w = Rt*w
			matxx_multiply(&p_dyn->Rt[i], &p_dyn->w, &p_dyn->tmp31);
			matxx_copy(&p_dyn->tmp31, &p_dyn->w);

			//wd = Rt*wd
			matxx_multiply(&p_dyn->Rt[i], &p_dyn->wd, &p_dyn->tmp31);
			matxx_copy(&p_dyn->tmp31, &p_dyn->wd);

			//Rt*(z0*qdd + vd)
			(*(p_dyn->tmp31.point))[0] = 0;
			(*(p_dyn->tmp31.point))[1] = 0;
			(*(p_dyn->tmp31.point))[2] = (*(qdd.point))[i];
			matxx_k_mac(1.0, &p_dyn->vd, &p_dyn->tmp31);
			matxx_multiply(&p_dyn->Rt[i], &p_dyn->tmp31, &p_dyn->vd);

			//wd*pstar + Rt*(z0*qdd + vd)
			matxx_cross(&p_dyn->wd, &p_dyn->pstar[i], &p_dyn->temp31);
			matxx_k_mac(1.0, &p_dyn->temp31, &p_dyn->vd);

			//wd*pstar + Rt*(z0*qdd + vd) + 2*w X (Rt*z0*qd)
			(*(p_dyn->tmp31.point))[0] = 0;
			(*(p_dyn->tmp31.point))[1] = 0;
			(*(p_dyn->tmp31.point))[2] = (*(qd.point))[i];
			matxx_multiply(&p_dyn->Rt[i], &p_dyn->tmp31, &p_dyn->temp31);
			matxx_cross(&p_dyn->w, &p_dyn->temp31, &p_dyn->tmp31);
			matxx_k_mac(2.0, &p_dyn->tmp31, &p_dyn->vd);

			//vd = wd*pstar + Rt*(z0*qdd + vd) + 2*w X (Rt*z0*qd)+w X (w X pstar)
			matxx_cross(&p_dyn->w, &p_dyn->pstar[i], &p_dyn->tmp31);
			matxx_cross(&p_dyn->w, &p_dyn->tmp31, &p_dyn->temp31);
			matxx_k_mac(1.0, &p_dyn->temp31, &p_dyn->vd);
		}
		//vcd = wd X r + w X (w X r) + vd 
		matxx_cross(&p_dyn->wd, &p_prm->links[i].r, &p_dyn->vcd);
		matxx_k_mac(1.0, &p_dyn->vd, &p_dyn->vcd);
		matxx_cross(&p_dyn->w, &p_prm->links[i].r, &p_dyn->temp31);
		matxx_cross(&p_dyn->w, &p_dyn->temp31, &p_dyn->tmp31);
		matxx_k_mac(1.0, &p_dyn->tmp31, &p_dyn->vcd);

		//F = m*vcd
		matxx_k_mac(p_prm->links[i].m, &p_dyn->vcd, &p_dyn->Fm[i]);

		//N = I*wd + w X (I*w)
		matxx_multiply(&p_prm->links[i].I, &p_dyn->wd, &p_dyn->Nm[i]);
		matxx_multiply(&p_prm->links[i].I, &p_dyn->w, &p_dyn->tmp31);
		matxx_cross(&p_dyn->w, &p_dyn->tmp31, &p_dyn->temp31);
		matxx_k_mac(1.0, &p_dyn->temp31, &p_dyn->Nm[i]);

	}
	matxx_copy(&p_dyn->fext, &p_dyn->f);
	matxx_copy(&p_dyn->Text, &p_dyn->T);

	for (i = p_prm->link_num - 1; i >= 0; i--)
	{
		//R*(T + (R'*pstar) X f)
		matxx_multiply(&p_dyn->Rt[i + 1], &p_dyn->pstar[i], &p_dyn->tmp31);
		matxx_cross(&p_dyn->tmp31, &p_dyn->f, &p_dyn->temp31);
		matxx_k_mac(1.0, &p_dyn->T, &p_dyn->temp31);
		matxx_multiply(&p_dyn->R[i + 1], &p_dyn->temp31, &p_dyn->T);

		//(pstar+r) X Fm + Nm
		matxx_copy(&p_prm->links[i].r, &p_dyn->tmp31);
		matxx_k_mac(1.0, &p_dyn->pstar[i], &p_dyn->tmp31);
		matxx_cross(&p_dyn->tmp31, &p_dyn->Fm[i], &p_dyn->temp31);
		matxx_k_mac(1.0, &p_dyn->Nm[i], &p_dyn->temp31);

		//T = R*(T + (R'*pstar) X f) + (pstar+r) X Fm + Nm
		matxx_k_mac(1.0, &p_dyn->temp31, &p_dyn->T);

		//f = R*f + Fm
		matxx_multiply(&p_dyn->R[i + 1], &p_dyn->f, &p_dyn->tmp31);
		matxx_k_mac(1.0, &p_dyn->Fm[i], &p_dyn->tmp31);
		matxx_copy(&p_dyn->tmp31, &p_dyn->f);

		//friction
		if ((*(qd.point))[i] > MOTION_MODULE_CONSTANT_MIN_POSITIVE)
		{
			fc = p_prm->links[i].B*fabs(p_prm->links[i].G)*(*(qd.point))[i] + p_prm->links[i].Tc[0];
		}
		else if ((*(qd.point))[i] < -MOTION_MODULE_CONSTANT_MIN_POSITIVE)
		{
			fc = p_prm->links[i].B*fabs(p_prm->links[i].G)*(*(qd.point))[i] + p_prm->links[i].Tc[1];
		}
		else
		{
			fc = 0;
		}
		fc = -fabs(p_prm->links[i].G)*fc;

		if (p_prm->links[i].type == 0)
		{
			//rotation
			(*(p_dyn->torque.point))[i] = (*(p_dyn->T.point))[0] * (*(p_dyn->Rt[i].point + 2))[0] + \
										  (*(p_dyn->T.point))[1] * (*(p_dyn->Rt[i].point + 2))[1] + \
										  (*(p_dyn->T.point))[2] * (*(p_dyn->Rt[i].point + 2))[2] + \
										  pow(p_prm->links[i].G, 2)*p_prm->links[i].Jm*(*(qdd.point))[i]-fc;
		}
		else
		{
			//prismatic
			(*(p_dyn->torque.point))[i] = (*(p_dyn->f.point))[0] * (*(p_dyn->Rt[i].point + 2))[0] + \
										  (*(p_dyn->f.point))[1] * (*(p_dyn->Rt[i].point + 2))[1] + \
										  (*(p_dyn->f.point))[2] * (*(p_dyn->Rt[i].point + 2))[2] + \
										  pow(p_prm->links[i].G, 2)*p_prm->links[i].Jm*(*(qdd.point))[i] - fc;
		}
		matxx_printf(&p_dyn->torque);
	}



	return 0;
};

int16 TorqueGenerator(Dynamics * p_dyn, DynamicPrf* p_prf, DynamicPrm* p_prm)
{
	int8 i;
	double ct, st, ca, sa,/* a,*/ d;
	double eyes[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
	if (p_prm->law_type == 0)  //joint impedance control
	{
		for (i = 0; i < p_prm->link_num; i++)
		{
			p_prf->aq[i] = p_prf->joint_acc_d[i] + p_prm->Kv[i] * (p_prf->joint_vel_d[i] - p_prf->joint_vel[i]) + \
						   p_prm->Kd[i] * (p_prf->joint_pos_d[i] - p_prf->joint_pos[i]);
		}
	}
	else if (p_prm->law_type == 1)  //cart impedance control
	{
		matxx_assign(&p_dyn->U1, eyes, 16);
		for (i = p_prm->link_num - 1; i >= 0; i--)
		{
			sa = sin(p_prm->links[i].alpha);
			ca = cos(p_prm->links[i].alpha);
			//rotation
			if (p_prm->links[i].type == 0)
			{
				d = p_prm->links[i].d;
				st = sin(p_prf->joint_pos_d[i]);
				ct = cos(p_prf->joint_pos_d[i]);
			}
			//prismatic
			else
			{
				d = p_prf->joint_pos_d[i];
				st = sin(p_prm->links[i].theta);
				ct = cos(p_prm->links[i].theta);
			}
			(*(p_dyn->A.point))[0]	   = ct;
			(*(p_dyn->A.point))[1]	   = st;
			(*(p_dyn->A.point))[2]	   = 0;
			(*(p_dyn->A.point))[3]	   = 0;
			(*(p_dyn->A.point + 1))[0] = -st*ca;
			(*(p_dyn->A.point + 1))[1] = ct*ca;
			(*(p_dyn->A.point + 1))[2] = sa;
			(*(p_dyn->A.point + 1))[3] = 0;
			(*(p_dyn->A.point + 2))[0] = st*sa;
			(*(p_dyn->A.point + 2))[1] = -ct*sa;
			(*(p_dyn->A.point + 2))[2] = ca;
			(*(p_dyn->A.point + 2))[3] = 0;
			(*(p_dyn->A.point + 3))[0] = p_prm->links[i].a*ct;
			(*(p_dyn->A.point + 3))[1] = p_prm->links[i].a*st;
			(*(p_dyn->A.point + 3))[2] = 0;
			(*(p_dyn->A.point + 3))[3] = 1;



		}


	}






	return 0;
};

