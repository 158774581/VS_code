#include "stdafx.h"
/*

* motion_planning_3rd.c

*

*  Created on: 2018.3.9

*      Author: bwang

*/

#include "type_def.h"

#include <stdio.h>

#include <stdlib.h>

#include <float.h>

#include <math.h>

#include "motion_planning_3rd.h"

#include "solver.h"



//--------------------------------------------------------------------------

//3 order motion planning design



/*   there are 8 time point for profile:point0 ~ point7  */

/*        Tacc             Tdec  */

/*        0 1              6 7   */

/*        .-.                    */

/*        | |T1    T3    T2.-.   */

/*        | |  2 3    4 5  | |   */

/*        '-'--.-.----.-.--' '   */

/*             | |    | |        */

/*             | |    '-'        */

/*             '-'               */

/*			 Tacc   Tdec	     */



//you can see that,there are two same Tacc time and two same Tdec time

//--------------------------------------------------------------------------

int16 motion_planning_3rd(MOTION_PLANNING_PRM* input, MOTION_PLANNING_3RD* m_mp3rd, double sample_time)
{

	//---------------------------

	Uint8 spe_case = 0;

	//---------------------------

	double tmp;

	double tmp1, tmp3;

	int16 rtn;

	//---------------------------

	QUADRATIC_SOLVER qs;

	QUARTIC_SOLVER qs1;



	m_mp3rd->boundary_condition_1 = 0;

	m_mp3rd->boundary_condition_2 = 0;

	m_mp3rd->boundary_condition_3 = 0;

	m_mp3rd->boundary_condition_4 = 0;

	m_mp3rd->boundary_condition_5 = 0;

	m_mp3rd->boundary_condition_6 = 0;



	//----------------------------------------------

	//get the input parameter,set by the user,save into internal struct

	m_mp3rd->prm.pos = input->pos;

	m_mp3rd->prm.spd_max = fabs(input->spd_max);

	m_mp3rd->prm.acc_max = fabs(input->acc_max);

	m_mp3rd->prm.dec_max = fabs(input->dec_max);

	m_mp3rd->prm.t_acc = fabs(input->t_acc);

	m_mp3rd->prm.t_dec = fabs(input->t_dec);

	//----------------------------------------------

	//calculate the acc part and dec part ,default we think acc part bigger than dec part,

	m_mp3rd->accMulTacc = m_mp3rd->prm.acc_max * m_mp3rd->prm.t_acc;

	m_mp3rd->decMulTdec = m_mp3rd->prm.dec_max * m_mp3rd->prm.t_dec;



	//if dec part bigger than acc part than we should change the acc parameter and the dec parameter

	//because we just want to calculate one condition.so we change to the default condition,which is acc part bigger than dec part



	if (m_mp3rd->accMulTacc < m_mp3rd->decMulTdec)

	{

		//special case 1 bit0

		spe_case = 1;



		//exchange acc and dec parameter

		tmp = m_mp3rd->prm.acc_max;

		m_mp3rd->prm.acc_max = m_mp3rd->prm.dec_max;

		m_mp3rd->prm.dec_max = tmp;

		//exchange t_acc and t_dec parameter

		tmp = m_mp3rd->prm.t_acc;

		m_mp3rd->prm.t_acc = m_mp3rd->prm.t_dec;

		m_mp3rd->prm.t_dec = tmp;



		m_mp3rd->accMulTacc = m_mp3rd->prm.acc_max * m_mp3rd->prm.t_acc;

		m_mp3rd->decMulTdec = m_mp3rd->prm.dec_max * m_mp3rd->prm.t_dec;

	}

	//----------------------------------------------

	//special case 2 bit1

	if (input->pos < 0)

	{

		m_mp3rd->prm.pos = -input->pos;

		spe_case |= 0x2;

	}

	//----------------------------------------------

	//calculate jerk  jerk is a step.

	m_mp3rd->Jacc = m_mp3rd->prm.acc_max / m_mp3rd->prm.t_acc;

	m_mp3rd->Jdec = m_mp3rd->prm.dec_max / m_mp3rd->prm.t_dec;





	//boundary condition define

	//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// spd_max >= t_acc *acc_max >= t_dec * dec_max  ,in this situation

	//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	if ((m_mp3rd->prm.spd_max >= m_mp3rd->accMulTacc) && (m_mp3rd->accMulTacc >= m_mp3rd->decMulTdec))

	{

		//--------------------------------------------------------------------------------------------------------------------------------------------

		// Boundary Condition 1:

		// T3 = 0, T1 > 0, T2 > 0   no even part 

		// conditions :pos, spd_max, t_dec, dec_max, t_acc, acc_max, all satisfied; 

		// when s > c1, spd_max can arrivate,which has even part,c1 is the Boundary condition for spd_max

		//-------------------------------------------------------------------------------

		//		 spd_max - acc_max * t_acc					spd_max - dec_max * t_dec

		// T1 = ---------------------				T2 = -------------------------------

		//				acc_max										dec_max

		//--------------------------------------------------------------------------------

		//						 spd_max * T1									   spd_max *T2

		// s1 = spd_max *t_acc + --------------				s2 = spd_max *t_dec +---------------

		//						       2												2

		//--------------------------------------------------------------------------------

		//	s = s1+s2 

		//--------------------------------------------------------------------------------

		m_mp3rd->boundary_condition_1 = 0.5*m_mp3rd->prm.spd_max*(m_mp3rd->prm.spd_max / m_mp3rd->prm.acc_max + m_mp3rd->prm.t_acc + \

			m_mp3rd->prm.spd_max / m_mp3rd->prm.dec_max + m_mp3rd->prm.t_dec);



		if (fabs(m_mp3rd->prm.pos - m_mp3rd->boundary_condition_1) < 1.0E-10)

		{

			m_mp3rd->boundary_condition_1 = m_mp3rd->prm.pos;

		}

		//--------------------------------------------------------------------------------------------------------------------------------------------

		// Boundary Condition 2:

		// T3 = 0, T1 = 0, T2 > 0  no even part and no even acc part  

		// pos, t_dec, dec_max, t_acc, acc_max, satisfied; 

		// when c1 > s > c2, spd_max can not arrive, but acc_max can arrive, when s = c2,which means T1 = 0, then acc_max also can not arrive

		// so c2 is the Boundary condition for acc_max

		//-------------------------------------------------------------------------------

		//							acc_max * t_acc - dec_max * t_dec

		// T1 = 0			T2 = -------------------------------------                         (acc_max * t_acc  = this value is which can arrivated max speed and then need to dec)

		//									dec_max

		//--------------------------------------------------------------------------------

		//																 T2

		// s1 = acc_max * t_acc *t_acc				s2 = acc_max*t_acc (---- + t_dec)    (v = acc_max*t_acc )

		//						 										  2

		//--------------------------------------------------------------------------------

		//	s = s1+s2 

		//--------------------------------------------------------------------------------

		m_mp3rd->boundary_condition_2 = 0.5*(m_mp3rd->prm.acc_max*m_mp3rd->prm.t_acc)* \

			((m_mp3rd->prm.acc_max*m_mp3rd->prm.t_acc) / m_mp3rd->prm.dec_max + (m_mp3rd->prm.t_dec + 2 * m_mp3rd->prm.t_acc));



		if (fabs(m_mp3rd->prm.pos - m_mp3rd->boundary_condition_2) < 1.0E-10)

		{

			m_mp3rd->boundary_condition_2 = m_mp3rd->prm.pos;

		}

		//--------------------------------------------------------------------------------------------------------------------------------------------

		//  Boundary Condition 3:

		//  T3 = 0, T1 = 0, T2 = 0  no even part and no even acc part,and no even dec part  

		//  pos, t_dec, dec_max, Jacc = acc_max/t_acc, satisfied; 

		//  when c2 > s > c3,   acc_max can not arrive ,but dec_max can arrive.but the Jacc = acc_max/t_acc, proportion can keep up.

		//  when s <= c3, which means T2 = 0. then dec_max can not arrive

		//  so c3 is the Boundary condition for dec_max

		//-------------------------------------------------------------------------------

		//									

		// T1 = 0	T2 = 0;		t_acc' = t_dec *sqrt(Jdec/Jacc)                  (tacc' = use the speed equal condition to calculate )

		//									  

		//--------------------------------------------------------------------------------

		//																

		// s1 = dec_max * t_dec * t_acc' =  dec_max * t_dec * t_dec *sqrt(Jdec/Jacc) = 	dec_max * t_dec *sqrt(t_dec * Jdec * t_dec /Jacc) = dec_max * t_dec *sqrt(dec_max *t_dec /Jacc)

		//

		// s2 = dec_max * t_dec * t_dec

		//						 										  

		//--------------------------------------------------------------------------------

		//	s = s1+s2 

		//--------------------------------------------------------------------------------



		m_mp3rd->boundary_condition_3 = (m_mp3rd->prm.t_dec *m_mp3rd->prm.dec_max)* \

			(sqrt(m_mp3rd->prm.t_dec *m_mp3rd->prm.dec_max / m_mp3rd->Jacc) + m_mp3rd->prm.t_dec);



		if (fabs(m_mp3rd->prm.pos - m_mp3rd->boundary_condition_3) < 1.0E-10)

		{

			m_mp3rd->boundary_condition_3 = m_mp3rd->prm.pos;

		}

	}

	//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// t_acc *acc_max > spd_max >= t_dec * dec_max  ,in this situation

	//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	if ((m_mp3rd->prm.spd_max < m_mp3rd->accMulTacc) && (m_mp3rd->prm.spd_max >= m_mp3rd->decMulTdec))

	{

		//--------------------------------------------------------------------------------------------------------------------------------------------

		//  Boundary Condition 4:

		//  T3 = 0, T1 = 0, T2 > 0   no even part and no even acc part

		//  pos, spd_max, t_dec, dec_max, Jacc = acc_max/t_acc, satisfied;

		//  because spd_max < acc_max * t_acc, so when s>c4 spd_max can arrived, but acc_max and t_acc can not but the proportion can keep up.

		//  when s>c4, because T1 must be 0 ,otherwise they can get to the acc_max,but this is impossiable,because spd_max < acc_max * t_acc,

		//  acc_max * t_acc is the triangle area,so s>c4 means T3 != 0, has even part.

		//  so c4 is the Boundary condition for spd_max(because T3 =0)

		//-------------------------------------------------------------------------------

		//					spd_max - dec_max * t_dec				

		// T1 = 0	T2 = ------------------------------            

		//						     dec_max			  

		//--------------------------------------------------------------------------------

		// spd_max = Jacc * t_acc'*t_acc'  						

		//

		// s1 = spd_max * t_acc' =  spd_max * sqrt(spd_max / Jacc)

		//

		// s2 = spd_max* (t_dec + 0.5 * T2)

		//						 										  

		//--------------------------------------------------------------------------------

		//	s = s1+s2 

		//--------------------------------------------------------------------------------

		m_mp3rd->boundary_condition_4 = m_mp3rd->prm.spd_max*(sqrt(m_mp3rd->prm.spd_max / m_mp3rd->Jacc) + \

			0.5*(m_mp3rd->prm.spd_max / m_mp3rd->prm.dec_max + m_mp3rd->prm.t_dec));



		if (fabs(m_mp3rd->prm.pos - m_mp3rd->boundary_condition_4) < 1.0E-10)

		{

			m_mp3rd->boundary_condition_4 = m_mp3rd->prm.pos;

		}

		//--------------------------------------------------------------------------------------------------------------------------------------------

		// Boundary Condition 5:

		// T3 = 0, T1 = 0, T2 = 0

		// pos, t_dec, dec_max, Jacc = acc_max/t_acc, satisfied;

		// when c4 > s > c5, dec_max and t_dec can arrived. because T2 != 0, but spd_max can not arrive because T3 = 0 

		// so c5 is the Boundary condition for dec_max and t_dec(because T2=0)

		// when s<c5  dec_max and t_dec also can not arrived

		//-------------------------------------------------------------------------------

		//							

		// T1 = 0	T2 = 0           

		//						  			  

		//--------------------------------------------------------------------------------

		// t_acc' = t_dec * sqrt(Jdec/Jacc)     (using spd equal condition to calculate)						

		//

		// s1 = dec_max * t_dec *sqrt(dec_max *t_dec /Jacc)

		//

		// s2 = dec_max * t_dec * t_dec

		//						 										  

		//--------------------------------------------------------------------------------

		//	s = s1+s2  the formula which is same as condition 3

		//--------------------------------------------------------------------------------

		m_mp3rd->boundary_condition_5 = (m_mp3rd->prm.t_dec *m_mp3rd->prm.dec_max)* \

			(sqrt(m_mp3rd->prm.t_dec *m_mp3rd->prm.dec_max / m_mp3rd->Jacc) + m_mp3rd->prm.t_dec);



		if (fabs(m_mp3rd->prm.pos - m_mp3rd->boundary_condition_5) < 1.0E-10)

		{

			m_mp3rd->boundary_condition_5 = m_mp3rd->prm.pos;

		}

	}

	//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// t_acc *acc_max >= t_dec * dec_max  > spd_max   ,in this situation

	//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	if ((m_mp3rd->accMulTacc >= m_mp3rd->decMulTdec) && (m_mp3rd->decMulTdec > m_mp3rd->prm.spd_max))

	{

		//--------------------------------------------------------------------------------------------------------------------------------------------

		//  Boundary Condition 6:

		//  T3 = 0, T1 = 0, T2 = 0

		//  pos, spd_max, Jacc = acc_max/t_acc,Jacc = dec_max/t_dec,satisfied;

		//  because  the spd_max < t_dec* dec_max,this means T1 = T2 = 0. must be zero 

		//  when s> c6 ,T3!=0 then spd_max can arrived ,but dec_max and t_dec can not 

		//  but Jacc = acc_max/t_acc, Jdec = dec_max/t_dec can keep up

		//  so c6 is the Boundary condition for spd_max 

		//  when s<c6, then T3 = 0 .spd_max also can not arrived

		//-------------------------------------------------------------------------------

		//							

		// T1 = 0	T2 = 0     T3  =0       

		//						  			  

		//--------------------------------------------------------------------------------

		// spd_max = Jacc * t_acc'* t_acc' = Jdec * t_dec'*t_dec'     (using spd equal condition to calculate)						

		//

		// t_acc' = sqrt(spd_max/Jacc),t_dec' = sqrt(spd_max/Jdec) 

		//

		// s1 = spd_max * t_acc' = spd_max * sqrt(spd_max/Jacc)

		//

		// s2 = spd_max * t_dec' = spd_max * sqrt(spd_max/Jdec)

		//						 										  

		//--------------------------------------------------------------------------------

		//	s = s1+s2  

		//--------------------------------------------------------------------------------

		m_mp3rd->boundary_condition_6 = m_mp3rd->prm.spd_max * (sqrt(m_mp3rd->prm.spd_max / m_mp3rd->Jacc) + sqrt(m_mp3rd->prm.spd_max / m_mp3rd->Jdec));



		if (fabs(m_mp3rd->prm.pos - m_mp3rd->boundary_condition_6) < 1.0E-10)

		{

			m_mp3rd->boundary_condition_6 = m_mp3rd->prm.pos;

		}

	}

	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// now we can calculate the different cases ,we need discrete the formulation, calculate the T1, T2, T3

	// because the time we calculate maybe not integer times of the sample time, so we need ceil the times.

	// result time is multiple of sample times,it is integer , for example  100, real time = 100 * sample_time(0.004);

	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// *********************************************************  case 1: **********************************************

	// spd_max >= acc_max * t_acc>= dec_max * t_dec

	// pos >= boundary_condition_1

	// which means T3>0

	// in this situation ,spd_max can arrived

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	if ((m_mp3rd->prm.spd_max >= m_mp3rd->accMulTacc) && (m_mp3rd->accMulTacc >= \

		m_mp3rd->decMulTdec) && (m_mp3rd->prm.pos >= m_mp3rd->boundary_condition_1))

	{

		//---------------------------------------------------------------------

		//get the n of t_acc ,n = ceil(t_acc / sample time)  

		//---------------------------------------------------------------------

		m_mp3rd->t_acc_ceil = ceil(m_mp3rd->prm.t_acc / sample_time);

		m_mp3rd->t_dec_ceil = ceil(m_mp3rd->prm.t_dec / sample_time);



		if ((m_mp3rd->t_acc_ceil == 0) || (m_mp3rd->t_dec_ceil == 0))

		{

			//this means t_acc and t_dec value is illegal,it must bigger than one sample time

			return -1;

		}

		//---------------------------------------------------------------------

		//get the real time of t_acc and t_dec, now ,time is n mutiply sample time

		m_mp3rd->t_acc_ceil_d = sample_time * m_mp3rd->t_acc_ceil;

		m_mp3rd->t_dec_ceil_d = sample_time * m_mp3rd->t_dec_ceil;



		//---------------------------------------------------------------------

		//				spd_max								spd_max

		// T1 = ceil(-------------- - tacc),   T2 = ceil(-------------- - tdec),

		//				t_acc *Jacc							t_dec *Jdec

		//get the n of the t1 and t2 , t1 = n * sample time 

		//---------------------------------------------------------------------

		m_mp3rd->t1_ceil = ceil(((m_mp3rd->prm.spd_max / (m_mp3rd->prm.acc_max)) - m_mp3rd->prm.t_acc) / sample_time);

		m_mp3rd->t2_ceil = ceil(((m_mp3rd->prm.spd_max / (m_mp3rd->prm.dec_max)) - m_mp3rd->prm.t_dec) / sample_time);

		//---------------------------------------------------------------------

		//get the real time after ceil

		m_mp3rd->t1_ceil_d = m_mp3rd->t1_ceil * sample_time;

		m_mp3rd->t2_ceil_d = m_mp3rd->t2_ceil * sample_time;



		//---------------------------------------------------------------------

		//			S-C1

		// T3 = -------------

		//			spd_max

		//---------------------------------------------------------------------

		m_mp3rd->t3_ceil = ceil(((m_mp3rd->prm.pos - m_mp3rd->boundary_condition_1) / m_mp3rd->prm.spd_max) / sample_time);

		m_mp3rd->t3_ceil_d = m_mp3rd->t3_ceil * sample_time;



		//---------------------------------------------------------------------

		//calculate the vmax

		//									s

		//vmax = -----------------------------------------------------------------

		//		t_acc_ceil_d + t_dec_ceil_d + 0.5*(t1_ceil_d+t2_ceil_d) + t3_ceil_d

		//---------------------------------------------------------------------

		m_mp3rd->vmax_tmp = m_mp3rd->prm.pos / (0.5*(m_mp3rd->t1_ceil_d + m_mp3rd->t2_ceil_d) + m_mp3rd->t_acc_ceil_d + m_mp3rd->t_dec_ceil_d + m_mp3rd->t3_ceil_d);



		//---------------------------------------------------------------------

		//calculate the acc_max and dec_max

		//									v_max

		//acc_max = -----------------------------------------------------------------

		//							t_acc_ceil_d + t1_ceil_d

		//---------------------------------------------------------------------

		//									v_max

		//dec_max = -----------------------------------------------------------------

		//							t_dec_ceil_d + t2_ceil_d

		//---------------------------------------------------------------------

		m_mp3rd->acc_tmp = m_mp3rd->vmax_tmp / (m_mp3rd->t1_ceil_d + m_mp3rd->t_acc_ceil_d);

		m_mp3rd->dec_tmp = m_mp3rd->vmax_tmp / (m_mp3rd->t2_ceil_d + m_mp3rd->t_dec_ceil_d);



		//---------------------------------------------------------------------

		//calculate the Jacc and Jdec

		m_mp3rd->Jacc = m_mp3rd->acc_tmp / m_mp3rd->t_acc_ceil_d;

		m_mp3rd->Jdec = m_mp3rd->dec_tmp / m_mp3rd->t_dec_ceil_d;

		//---------------------------------------------------------------------

	}

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// *********************************************************  case 2: **********************************************

	// spd_max >= acc_max * t_acc>= dec_max * t_dec

	// boundary_condition_1 > pos >= boundary_condition_2

	// in this situation we can not arrive at max spd  because T3 = 0  

	// but we can arrive acc_max and t_acc because T1 != 0

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	if ((m_mp3rd->prm.spd_max >= m_mp3rd->accMulTacc) && (m_mp3rd->accMulTacc >= \

		m_mp3rd->decMulTdec) && (m_mp3rd->prm.pos < m_mp3rd->boundary_condition_1) && (m_mp3rd->prm.pos >= m_mp3rd->boundary_condition_2))

	{

		//---------------------------------------------------------------------

		//get the n of t_acc ,n = ceil(t_acc / sample time)  

		//---------------------------------------------------------------------

		m_mp3rd->t_acc_ceil = ceil(m_mp3rd->prm.t_acc / sample_time);

		m_mp3rd->t_dec_ceil = ceil(m_mp3rd->prm.t_dec / sample_time);



		if ((m_mp3rd->t_acc_ceil == 0) || (m_mp3rd->t_dec_ceil == 0))

		{

			//this means t_acc and t_dec value is illegal,it must bigger than one sample time

			return -1;

		}

		//---------------------------------------------------------------------

		//get the real time of t_acc and t_dec, now ,time is n mutiply sample time

		m_mp3rd->t_acc_ceil_d = sample_time * m_mp3rd->t_acc_ceil;

		m_mp3rd->t_dec_ceil_d = sample_time * m_mp3rd->t_dec_ceil;



		//---------------------------------------------------------------------

		//we need calculate v first ,because we can not arrive at max spd

		//

		// S = 0.5*V*T = 0.5*V*(T1 + T2 + 2*t_acc +2*t_dec)

		//

		//				v							v

		// T1 = (-------------- - tacc),   T2 = (-------------- - tdec), substitute T1,T2 into the first formula

		//			acc_max							dec_max

		// we can get 

		//

		//2S = V*(V/acc_max +V/dec_max + t_acc + t_dec) ====>    (1/acc_max + 1/dec_max)V^2 + (t_acc + t_dec)*V -2s = 0

		//

		//now we need calculate the quadratic equation

		//-----------------------------------------------------------------------

		tmp1 = (1 / m_mp3rd->prm.acc_max + 1 / m_mp3rd->prm.dec_max);

		qs.c0 = (m_mp3rd->prm.t_acc + m_mp3rd->prm.t_dec) / tmp1;

		qs.c1 = -2 * m_mp3rd->prm.pos / tmp1;

		quadratic_solver(&qs);



		//because we want to get v,so v>0 ,we choose r0 (which is bigger than zero)

		switch (qs.root_type)

		{

		case QUADRATIC_NO_ROOTS:

			return -1;

			break;

		case QUADRATIC_TWO_SAME_ROOTS:

			//because c0 >0 c1 <0 ,this condition not satisfied

			return -2;

			break;

		case QUADRATIC_TWO_DIFF_ROOTS:

			//now we has the v that we can arrivate in r0.

			//---------------------------------------------------------------------

			//				    v										v

			// T1 = ceil(-------------- - tacc),   T2 = ceil(-------------- - tdec),

			//				acc_max							    dec_max

			//get the n of the t1 and t2 , t1 = n * sample time 

			//---------------------------------------------------------------------

			m_mp3rd->t1_ceil = ceil((qs.r0 / m_mp3rd->prm.acc_max - m_mp3rd->prm.t_acc) / sample_time);

			m_mp3rd->t2_ceil = ceil((qs.r0 / m_mp3rd->prm.dec_max - m_mp3rd->prm.t_dec) / sample_time);



			m_mp3rd->t1_ceil_d = m_mp3rd->t1_ceil*sample_time;

			m_mp3rd->t2_ceil_d = m_mp3rd->t2_ceil*sample_time;

			break;

		default:

			break;

		}



		//---------------------------------------------------------------------

		m_mp3rd->t3_ceil = 0;

		m_mp3rd->t3_ceil_d = 0;



		//---------------------------------------------------------------------

		//calculate vmax

		//									s

		//vmax = -----------------------------------------------------------------

		//		 t_acc_ceil_d + t_dec_ceil_d + 0.5*(t1_ceil_d+t2_ceil_d)

		//---------------------------------------------------------------------

		m_mp3rd->vmax_tmp = m_mp3rd->prm.pos / (0.5*(m_mp3rd->t1_ceil_d + m_mp3rd->t2_ceil_d) + m_mp3rd->t_acc_ceil_d + m_mp3rd->t_dec_ceil_d);



		//---------------------------------------------------------------------

		//calculate the acc_max and dec_max

		//									v_max

		//acc_max = -----------------------------------------------------------------

		//							t_acc_ceil_d + t1_ceil_d

		//---------------------------------------------------------------------

		//									v_max

		//dec_max = -----------------------------------------------------------------

		//							t_dec_ceil_d + t2_ceil_d

		//---------------------------------------------------------------------

		m_mp3rd->acc_tmp = m_mp3rd->vmax_tmp / (m_mp3rd->t1_ceil_d + m_mp3rd->t_acc_ceil_d);

		m_mp3rd->dec_tmp = m_mp3rd->vmax_tmp / (m_mp3rd->t2_ceil_d + m_mp3rd->t_dec_ceil_d);



		//---------------------------------------------------------------------

		//calculate the Jacc and Jdec

		m_mp3rd->Jacc = m_mp3rd->acc_tmp / m_mp3rd->t_acc_ceil_d;

		m_mp3rd->Jdec = m_mp3rd->dec_tmp / m_mp3rd->t_dec_ceil_d;

		//---------------------------------------------------------------------

	}

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// *********************************************************  case 3: **********************************************

	// spd_max >= acc_max * t_acc>= dec_max * t_dec

	// boundary_condition_2 > pos >= boundary_condition_3

	// in this situation we can not arrive at acc_max and t_acc,but t_dec and dec_max can arrived

	// so we need calculate t_acc first

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	if (((m_mp3rd->prm.spd_max >= m_mp3rd->accMulTacc) && (m_mp3rd->accMulTacc >= \

		m_mp3rd->decMulTdec) && (m_mp3rd->prm.pos < m_mp3rd->boundary_condition_2) && (m_mp3rd->prm.pos >= m_mp3rd->boundary_condition_3)) || \

		((m_mp3rd->prm.spd_max < m_mp3rd->accMulTacc) && (m_mp3rd->prm.spd_max >= \

			m_mp3rd->decMulTdec) && (m_mp3rd->prm.pos >= m_mp3rd->boundary_condition_5) && (m_mp3rd->prm.pos < m_mp3rd->boundary_condition_4)))

	{

		//---------------------------------------------------------------------

		//get the n of t_dec ,n = ceil(t_dec / sample time)  

		//---------------------------------------------------------------------

		m_mp3rd->t_dec_ceil = ceil(m_mp3rd->prm.t_dec / sample_time);



		if ((m_mp3rd->t_dec_ceil == 0))

		{

			//this means t_acc and t_dec value is illegal,it must bigger than one sample time

			return -1;

		}

		//---------------------------------------------------------------------

		//get the real time of t_dec, now ,time is n mutiply sample time

		m_mp3rd->t_dec_ceil_d = sample_time * m_mp3rd->t_dec_ceil;



		//---------------------------------------------------------------------

		m_mp3rd->t1_ceil = 0;

		m_mp3rd->t1_ceil_d = 0;



		//---------------------------------------------------------------------

		m_mp3rd->t3_ceil = 0;

		m_mp3rd->t3_ceil_d = 0;



		//---------------------------------------------------------------------

		//we need calculate t_acc first

		// using v equal method

		// 

		// v = Jacc * t_acc'*t_acc' = Jdec*t_dec*(t_dec + T2)            (1)

		//

		//S = 0.5*v*T = 0.5*v*(2*t_acc +T2 +2*t_dec)					 (2)

		//

		// substitute (1) into (2) ,erase T2 and v

		//

		//we get t_acc's formula

		//		 Jacc^2										Jacc*t_dec

		// S = --------------- *t_acc'^4 + Jacc* t_acc'^3 + --------- *t_acc'^2

		//       2*Jdec*t_dec							     2			

		//

		// s =a x^4 + bx^3 +cx^2 + 0 

		//

		//c0 = b/a,c1 = c/a c2 = 0 c3 = -s/a

		//c0 >0

		//c1 >0

		//c2 =0

		//c3 <0



		qs1.c0 = (2 * m_mp3rd->Jdec * m_mp3rd->prm.t_dec) / m_mp3rd->Jacc;

		qs1.c1 = (m_mp3rd->Jdec*m_mp3rd->prm.t_dec*m_mp3rd->prm.t_dec) / m_mp3rd->Jacc;

		qs1.c2 = 0;

		qs1.c3 = (-2 * m_mp3rd->prm.pos*m_mp3rd->Jdec*m_mp3rd->prm.t_dec) / (m_mp3rd->Jacc*m_mp3rd->Jacc);



		rtn = quartic_solver(&qs1);

		if (rtn != 0)

		{

			return -1;

		}



		switch (qs1.root_type)

		{

		case QUARTIC_NO_REAL_ROOTS:

			return -2;

			break;

		case QUARTIC_SPECIAL_REAL_ROOT_R0:
		case QUARTIC_SPECIAL_REAL_ROOT_R2:
		case QUARTIC_SPECIAL_REAL_ROOT_BIGGER:

		case QUARTIC_TWO_SAME_01_REAL_ROOTS:

		case QUARTIC_TWO_DIFF_01_REAL_ROOTS:

		case QUARTIC_TWO_SAME_23_REAL_ROOTS:

		case QUARTIC_TWO_DIFF_23_REAL_ROOTS:

		case QUARTIC_TWO_SAME_TWO_SAME_REAL_ROOTS:

		case QUARTIC_TWO_SAME_TWO_DIFF_REAL_ROOTS:

		case QUARTIC_TWO_DIFF_TWO_SAME_REAL_ROOTS:

		case QUARTIC_TWO_DIFF_TWO_DIFF_REAL_ROOTS:

			tmp1 = qs1.r0;

			break;

		default:

			break;

		}

		//get the number of the ceil

		m_mp3rd->t_acc_ceil = ceil(tmp1 / sample_time);



		if ((m_mp3rd->t_acc_ceil == 0))

		{

			//this means t_acc and t_dec value is illegal,it must bigger than one sample time

			return -1;

		}

		//---------------------------------------------------------------------

		//get the real time of t_acc, now ,time is n mutiply sample time

		m_mp3rd->t_acc_ceil_d = sample_time * m_mp3rd->t_acc_ceil;



		//---------------------------------------------------------------------

		//				Jacc* t_acc^2								

		// T2 = ceil(-------------- - tdec)

		//				Jdec * t_dec							    

		//get the n of the t2 ,t2 = n * sample time 

		//---------------------------------------------------------------------

		m_mp3rd->t2_ceil = ceil(((m_mp3rd->Jacc * tmp1 *tmp1 / (m_mp3rd->Jdec* m_mp3rd->prm.t_dec)) - m_mp3rd->prm.t_dec) / sample_time);

		//---------------------------------------------------------------------

		//get the real time after ceil

		m_mp3rd->t2_ceil_d = m_mp3rd->t2_ceil * sample_time;



		//---------------------------------------------------------------------

		//calculate vmax

		//									s

		//vmax = -----------------------------------------------------------------

		//		 t_acc_ceil_d + t_dec_ceil_d + 0.5*t2_ceil_d

		//---------------------------------------------------------------------

		m_mp3rd->vmax_tmp = m_mp3rd->prm.pos / ((0.5* m_mp3rd->t2_ceil_d) + m_mp3rd->t_acc_ceil_d + m_mp3rd->t_dec_ceil_d);



		//---------------------------------------------------------------------

		//calculate the acc_max and dec_max

		//									vmax

		//acc_max = -----------------------------------------------------------------

		//							     t_acc_ceil_d 

		//---------------------------------------------------------------------

		//									vmax

		//dec_max = -----------------------------------------------------------------

		//							t_dec_ceil_d + t2_ceil_d

		//---------------------------------------------------------------------

		m_mp3rd->acc_tmp = m_mp3rd->vmax_tmp / (m_mp3rd->t_acc_ceil_d);

		m_mp3rd->dec_tmp = m_mp3rd->vmax_tmp / (m_mp3rd->t2_ceil_d + m_mp3rd->t_dec_ceil_d);



		//---------------------------------------------------------------------

		//calculate the Jacc and Jdec

		m_mp3rd->Jacc = m_mp3rd->acc_tmp / m_mp3rd->t_acc_ceil_d;

		m_mp3rd->Jdec = m_mp3rd->dec_tmp / m_mp3rd->t_dec_ceil_d;

		//---------------------------------------------------------------------

	}

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// *********************************************************  case 4: **********************************************

	// spd_max >= acc_max * t_acc>= dec_max * t_dec

	// boundary_condition_3 > pos

	// in this situation we can not arrive at max acc and max dec

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	if (((m_mp3rd->prm.spd_max >= m_mp3rd->accMulTacc) && (m_mp3rd->accMulTacc >= \

		m_mp3rd->decMulTdec) && (m_mp3rd->prm.pos < m_mp3rd->boundary_condition_3)) || \

		((m_mp3rd->prm.spd_max < m_mp3rd->accMulTacc) && (m_mp3rd->prm.spd_max >= \

			m_mp3rd->decMulTdec) && (m_mp3rd->prm.pos < m_mp3rd->boundary_condition_5)) || \

			((m_mp3rd->prm.spd_max < m_mp3rd->decMulTdec) && (m_mp3rd->accMulTacc > \

				m_mp3rd->decMulTdec) && (m_mp3rd->prm.pos < m_mp3rd->boundary_condition_6)))

	{

		tmp1 = pow(m_mp3rd->prm.pos / (m_mp3rd->Jdec*(1 + sqrt(m_mp3rd->Jdec / m_mp3rd->Jacc))), 0.33333333333333333);



		//---------------------------------------------------------------------

		//get the n of t_dec,t_acc ,n = ceil(t_dec / sample time)  

		//---------------------------------------------------------------------

		m_mp3rd->t_dec_ceil = ceil(tmp1 / sample_time);

		m_mp3rd->t_acc_ceil = ceil((sqrt(m_mp3rd->Jdec / m_mp3rd->Jacc)*tmp1) / sample_time);



		if ((m_mp3rd->t_dec_ceil == 0) || (m_mp3rd->t_acc_ceil == 0))

		{

			//this means t_acc and t_dec value is illegal,it must bigger than one sample time

			return -1;

		}

		//---------------------------------------------------------------------

		//get the real time of t_dec, now ,time is n mutiply sample time

		m_mp3rd->t_acc_ceil_d = sample_time * m_mp3rd->t_acc_ceil;

		m_mp3rd->t_dec_ceil_d = sample_time * m_mp3rd->t_dec_ceil;



		m_mp3rd->t1_ceil = 0;

		m_mp3rd->t2_ceil = 0;

		m_mp3rd->t3_ceil = 0;

		m_mp3rd->t1_ceil_d = 0;

		m_mp3rd->t2_ceil_d = 0;

		m_mp3rd->t3_ceil_d = 0;



		//---------------------------------------------------------------------

		//calculate vmax

		//				 s

		//vmax = -----------------------------

		//		 t_acc_ceil_d + t_dec_ceil_d

		//---------------------------------------------------------------------

		m_mp3rd->vmax_tmp = m_mp3rd->prm.pos / (m_mp3rd->t_acc_ceil_d + m_mp3rd->t_dec_ceil_d);



		//---------------------------------------------------------------------

		//calculate the acc_max and dec_max

		//									vmax

		//acc_max = -----------------------------------------------------------------

		//							     t_acc_ceil_d 

		//---------------------------------------------------------------------

		//									vmax

		//dec_max = -----------------------------------------------------------------

		//							     t_dec_ceil_d

		//---------------------------------------------------------------------

		m_mp3rd->acc_tmp = m_mp3rd->vmax_tmp / (m_mp3rd->t_acc_ceil_d);

		m_mp3rd->dec_tmp = m_mp3rd->vmax_tmp / (m_mp3rd->t_dec_ceil_d);





		//---------------------------------------------------------------------

		//calculate the Jacc and Jdec

		m_mp3rd->Jacc = m_mp3rd->acc_tmp / m_mp3rd->t_acc_ceil_d;

		m_mp3rd->Jdec = m_mp3rd->dec_tmp / m_mp3rd->t_dec_ceil_d;

		//---------------------------------------------------------------------

	}

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// *********************************************************  case 5: **********************************************

	// acc_max * t_acc > spd_max>= dec_max * t_dec

	// pos >=boundary_condition_4

	// in this situation we can not arrive at max acc ,but can get to max speed

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	if ((m_mp3rd->prm.spd_max < m_mp3rd->accMulTacc) && (m_mp3rd->prm.spd_max >= \

		m_mp3rd->decMulTdec) && (m_mp3rd->prm.pos >= m_mp3rd->boundary_condition_4))

	{

		//---------------------------------------------------------------------

		//get the n of t_dec,t_acc ,n = ceil(t_dec / sample time)  

		//---------------------------------------------------------------------

		m_mp3rd->t_acc_ceil = ceil(sqrt(m_mp3rd->prm.spd_max / m_mp3rd->Jacc) / sample_time);

		m_mp3rd->t_dec_ceil = ceil(m_mp3rd->prm.t_dec / sample_time); //t_tec can arrive



		if ((m_mp3rd->t_dec_ceil == 0) || (m_mp3rd->t_acc_ceil == 0))

		{

			//this means t_acc and t_dec value is illegal,it must bigger than one sample time

			return -1;

		}

		//---------------------------------------------------------------------

		//get the real time of t_dec, now ,time is n mutiply sample time

		m_mp3rd->t_acc_ceil_d = sample_time * m_mp3rd->t_acc_ceil;

		m_mp3rd->t_dec_ceil_d = sample_time * m_mp3rd->t_dec_ceil;



		m_mp3rd->t1_ceil = 0;

		m_mp3rd->t1_ceil_d = 0;



		//calculate T2

		m_mp3rd->t2_ceil = ceil(((m_mp3rd->prm.spd_max / m_mp3rd->prm.dec_max) - m_mp3rd->prm.t_dec) / sample_time);

		m_mp3rd->t2_ceil_d = m_mp3rd->t2_ceil*sample_time;



		//calulate T3 

		m_mp3rd->t3_ceil = ceil(((m_mp3rd->prm.pos - m_mp3rd->boundary_condition_4) / m_mp3rd->prm.spd_max) / sample_time);

		m_mp3rd->t3_ceil_d = m_mp3rd->t3_ceil * sample_time;



		//---------------------------------------------------------------------

		//calculate vmax

		//									s

		//vmax = -----------------------------------------------------------------

		//		 t_acc_ceil_d + t_dec_ceil_d + 0.5*t2_ceil_d +��t3_ceil_d

		//---------------------------------------------------------------------

		m_mp3rd->vmax_tmp = m_mp3rd->prm.pos / ((0.5* m_mp3rd->t2_ceil_d + m_mp3rd->t3_ceil_d) + m_mp3rd->t_acc_ceil_d + m_mp3rd->t_dec_ceil_d);



		//---------------------------------------------------------------------

		//calculate the acc_max and dec_max

		//									vmax

		//acc_max = -----------------------------------------------------------------

		//							     t_acc_ceil_d 

		//---------------------------------------------------------------------

		//									vmax

		//dec_max = -----------------------------------------------------------------

		//							t_dec_ceil_d + t2_ceil_d

		//---------------------------------------------------------------------

		m_mp3rd->acc_tmp = m_mp3rd->vmax_tmp / (m_mp3rd->t_acc_ceil_d);

		m_mp3rd->dec_tmp = m_mp3rd->vmax_tmp / (m_mp3rd->t2_ceil_d + m_mp3rd->t_dec_ceil_d);



		//---------------------------------------------------------------------

		//calculate the Jacc and Jdec

		m_mp3rd->Jacc = m_mp3rd->acc_tmp / m_mp3rd->t_acc_ceil_d;

		m_mp3rd->Jdec = m_mp3rd->dec_tmp / m_mp3rd->t_dec_ceil_d;

		//---------------------------------------------------------------------

	}

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// *********************************************************  case 6: **********************************************

	// acc_max * t_acc > spd_max>= dec_max * t_dec

	// boundary_condition_4 > pos >=boundary_condition_5

	// in this situation we can not arrive at max acc ,also can not get to max speed, but can get to max dec(t2!=0)

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	//if ((m_mp3rd->prm.spd_max < m_mp3rd->accMulTacc) && (m_mp3rd->prm.spd_max >=

	//		m_mp3rd->decMulTdec) && (m_mp3rd->prm.pos >= m_mp3rd->boundary_condition_5)&& (m_mp3rd->prm.pos < m_mp3rd->boundary_condition_4))

	//{

	//		//same as case 3

	//

	//}

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// *********************************************************  case 7: **********************************************

	// acc_max * t_acc > spd_max>= dec_max * t_dec

	// boundary_condition_5 > pos

	// in this situation we can not arrive at max acc and max dec

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	//if ((m_mp3rd->prm.spd_max < m_mp3rd->accMulTacc) && (m_mp3rd->prm.spd_max >=

	//		m_mp3rd->decMulTdec) && (m_mp3rd->prm.pos < m_mp3rd->boundary_condition_5))

	//{

	//		//same as case 4

	//

	//}

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// *********************************************************  case 8: **********************************************

	// acc_max * t_acc > dec_max * t_dec > spd_max

	// boundary_condition_6 <= pos

	// in this situation we can not arrive at max acc and max dec t3 can has value t3!=0 so spd_max can arrive

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	if ((m_mp3rd->prm.spd_max < m_mp3rd->decMulTdec) && (m_mp3rd->accMulTacc > \

		m_mp3rd->decMulTdec) && (m_mp3rd->prm.pos >= m_mp3rd->boundary_condition_6))

	{

		//get the number of the ceil

		m_mp3rd->t_acc_ceil = ceil(sqrt(m_mp3rd->prm.spd_max / m_mp3rd->Jacc) / sample_time);

		m_mp3rd->t_dec_ceil = ceil(sqrt(m_mp3rd->prm.spd_max / m_mp3rd->Jdec) / sample_time);



		//according the ceil number to get the t_acc and t_dec time after the ceil

		m_mp3rd->t_acc_ceil_d = sample_time * m_mp3rd->t_acc_ceil;

		m_mp3rd->t_dec_ceil_d = sample_time * m_mp3rd->t_dec_ceil;



		m_mp3rd->t1_ceil = 0;

		m_mp3rd->t2_ceil = 0;

		m_mp3rd->t1_ceil_d = 0;

		m_mp3rd->t2_ceil_d = 0;



		//calulate T3

		m_mp3rd->t3_ceil = ceil(((m_mp3rd->prm.pos - m_mp3rd->boundary_condition_6) / m_mp3rd->prm.spd_max) / sample_time);

		m_mp3rd->t3_ceil_d = m_mp3rd->t3_ceil * sample_time;



		//---------------------------------------------------------------------

		//calculate vmax

		//									s

		//vmax = -----------------------------------------------------------------

		//		 t_acc_ceil_d + t_dec_ceil_d +��t3_ceil_d

		//---------------------------------------------------------------------

		m_mp3rd->vmax_tmp = m_mp3rd->prm.pos / (m_mp3rd->t3_ceil_d + m_mp3rd->t_acc_ceil_d + m_mp3rd->t_dec_ceil_d);



		//---------------------------------------------------------------------

		//calculate the acc_max and dec_max

		//									vmax

		//acc_max = -----------------------------------------------------------------

		//							     t_acc_ceil_d 

		//---------------------------------------------------------------------

		//									vmax

		//dec_max = -----------------------------------------------------------------

		//							t_dec_ceil_d

		//---------------------------------------------------------------------

		m_mp3rd->acc_tmp = m_mp3rd->vmax_tmp / (m_mp3rd->t_acc_ceil_d);

		m_mp3rd->dec_tmp = m_mp3rd->vmax_tmp / (m_mp3rd->t_dec_ceil_d);



		//---------------------------------------------------------------------

		//calculate the Jacc and Jdec

		m_mp3rd->Jacc = m_mp3rd->acc_tmp / m_mp3rd->t_acc_ceil_d;

		m_mp3rd->Jdec = m_mp3rd->dec_tmp / m_mp3rd->t_dec_ceil_d;

		//---------------------------------------------------------------------

	}

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// *********************************************************  case 9: **********************************************

	// acc_max * t_acc > dec_max * t_dec > spd_max

	// boundary_condition_6 > pos

	// in this situation we can not arrive at max acc and max dec

	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	//if ((m_mp3rd->prm.spd_max < m_mp3rd->decMulTdec) && (m_mp3rd->accMulTacc >

	//		m_mp3rd->decMulTdec) && (m_mp3rd->prm.pos < m_mp3rd->boundary_condition_6))

	//{

	//		//same as case 4

	//

	//}



	//------------------------------------------------------------------------------------------

	// special situation process

	if ((spe_case & 1))

	{

		tmp3 = m_mp3rd->Jdec;

		m_mp3rd->Jdec = m_mp3rd->Jacc;

		m_mp3rd->Jacc = tmp3;



		tmp3 = m_mp3rd->t_dec_ceil_d;

		m_mp3rd->t_dec_ceil_d = m_mp3rd->t_acc_ceil_d;

		m_mp3rd->t_acc_ceil_d = tmp3;



		tmp3 = m_mp3rd->t2_ceil_d;

		m_mp3rd->t2_ceil_d = m_mp3rd->t1_ceil_d;

		m_mp3rd->t1_ceil_d = tmp;



		tmp3 = m_mp3rd->t_dec_ceil;

		m_mp3rd->t_dec_ceil = m_mp3rd->t_acc_ceil;

		m_mp3rd->t_acc_ceil = tmp3;



		tmp3 = m_mp3rd->t2_ceil;

		m_mp3rd->t2_ceil = m_mp3rd->t1_ceil;

		m_mp3rd->t1_ceil = tmp;

	}



	/*  special case 2: */

	/*  S < 0 */

	if ((spe_case & 2))

	{

		m_mp3rd->Jacc = -m_mp3rd->Jacc;

		m_mp3rd->Jdec = -m_mp3rd->Jdec;

	}



	//calculate the sum of the time

	m_mp3rd->t_all_d = m_mp3rd->t_acc_ceil_d + m_mp3rd->t1_ceil_d + m_mp3rd->t_acc_ceil_d + m_mp3rd->t3_ceil_d + m_mp3rd->t_dec_ceil_d + \

		m_mp3rd->t2_ceil_d + m_mp3rd->t_dec_ceil_d;

	m_mp3rd->t_all = ceil(m_mp3rd->t_all_d / sample_time);



	return 0;



}

void calculate_acceleration_from_profile_(double t,
	double jerk,
	double acc_init,
	double* acc_t)
{
	*acc_t = acc_init;
	*acc_t += jerk * t;
	return;
}
//------------------------------------------------------------------
void calculate_velocity_from_profile_(double t,
	double jerk,
	double acc_init,
	double vel_init,
	double* vel_t)
{
	/*
	* v =  v0 + a0 * t + 0.5 * jerk * t * t
	*   = v0 + (a0  + 0.5 * jerk * t )* t
	* */
	*vel_t = acc_init;
	*vel_t += jerk * t * 0.5;
	*vel_t *= t;
	*vel_t += vel_init;
	return;
}
//------------------------------------------------------------------
void calculate_position_from_profile_(double t,
	double jerk,
	double acc_init,
	double vel_init,
	double pos_init,
	double* pos_t)
{
	/*
	* s
	* =  s0+ v0*t + a0*t*t*1/2  + jerk*t*t*t*1/6
	* =  s0+ (v0 + a0*t*1/2  + jerk*t*t*1/6)*t
	* =  s0+ (v0 + (a0  + jerk*t*1/3)*t*1/2 )*t
	* */
	*pos_t = acc_init;
	*pos_t += jerk * t * 0.333333333333333;
	*pos_t *= (t*0.5);
	*pos_t += vel_init;
	*pos_t *= t;
	*pos_t += pos_init;
	return;
}
//----------------------------------------------------------------------------------------------------------------------------------------
// motion planning with v0 and ve
// Acc=Dec,Tacc=Tdec;对称的梯形规划。某些文献中也叫s型曲线

int16 motion_planning_nonzero(MOTION_PLANNING_NON_ZERO_PRM* input, MOTION_PLANNING_3RD* m_mp3rd, motion_trajectory_nonzero* motion_tra, double sample_time)
{
	int8		error_flag = 0;
	int8		reverse_flag = 0;
	int8		dec_first = 0;
	double		V = input->spd_max;
	double		Acc = input->acc_max;
	double		J = input->acc_max / input->t_acc;
	double		s = input->pos;
	double      vs = input->vs;
	double		ve = input->ve;
	double		Lsvme = 0;  // 能否到达最大速度的临界位移
	double		L_a = 0;  // 从初速度之间加（减）到末速度的位移。

						  //var
	double		Acc2invJ = Acc*Acc / J;
	double		T[8] = { 0,0,0,0,0,0,0 };  // unit s
	double		T_[8] = { 0,0,0,0,0,0,0 };  // unit ms
	double		vm = 0;
	double		J1 = 0;
	double		J2 = 0;
	int8		counter = 0;

	if (s < 0)
	{
		s = -s;
		reverse_flag = 1;
		vs = -vs;
		ve = -ve;
	}
	if (V < vs || V < ve)
	{
		printf("初或末速度大于系统最大速度");
		if (V < vs)
		{
			V = vs;
		}
		if (V < ve)
		{
			V = ve;
		}
	}
	Lsvme = DisplacemenInDeltaV(vs, V - vs, Acc, J) + DisplacemenInDeltaV(ve, V - ve, Acc, J);

	if (s > Lsvme)  // 能够达到系统最大速度
	{
		if (V - vs > Acc2invJ)  // 有匀加速段
		{
			T[2] = T[0] = Acc / J;
			T[1] = (V - vs - Acc2invJ) / Acc;
			T[3] = (s - Lsvme) / V;
		}
		else  // 没有匀加速段
		{
			T[2] = T[0] = pow((V - vs) / J, 0.5);
			T[1] = 0;
			T[3] = (s - Lsvme) / V;
		}
		if (V - ve>Acc2invJ)  // 有匀减速段
		{
			T[6] = T[4] = Acc / J;
			T[5] = (V - ve - Acc2invJ) / Acc;

		}
		else  // 没有匀减速段
		{
			T[6] = T[4] = pow((V - ve) / J, 0.5);
			T[5] = 0;
		}
	}
	else  // 不能达到系统最大速度
	{
		if (vs < ve)
		{
			L_a = DisplacemenInDeltaV(vs, ve - vs, Acc, J);  // 直接加速所需位移，加速度曲线一个梯形或三角形
			if (s - L_a < input->secant_eps)
			{
				dec_first = 1;  // 需要先减速后加速
				vm = ve;
				if (s - L_a < -input->secant_eps)
				{
					printf("error 1:位移过小，需要先减速后加速");
				}
			}
		}
		else
		{
			L_a = DisplacemenInDeltaV(ve, vs - ve, Acc, J);  // 直接减速所需位移，加速度曲线一个梯形或三角形
			if (s - L_a < input->secant_eps)
			{
				dec_first = 2;  // 需要先减速后加速
				vm = vs;
				if (s - L_a < -input->secant_eps)
				{
					printf("2:位移过小，需要先减速后加速");
				}
			}
		}
		if (dec_first == 0)  // 先加速后减速
		{
			// 寻找能够达到的最大速度
			if (NewTonSecantMethodToFindRoot(vs, V, s, vs, ve, Acc, J, input->secant_eps, input->max_itera_time, &vm) == -1)
			{
				printf("迭代超过最大次数");
				return -1;
			}
			if (vm - vs > Acc2invJ)  // 有匀加速段
			{
				T[0] = T[2] = Acc / J;
				T[1] = (vm - vs - Acc2invJ) / Acc;
				T[3] = 0;
			}
			else  // 没有匀加速段
			{
				if (vm - vs > 0)
				{
					T[0] = pow((vm - vs) / J, 0.5);
				}
				else
				{
					T[0] = 0;
				}
				T[2] = T[0];
				T[1] = T[3] = 0;
			}
			if (vm - ve > Acc2invJ)  // 有匀减速段
			{
				T[6] = T[4] = Acc / J;
				T[5] = (vm - ve - Acc2invJ) / Acc;
			}
			else  // 没有匀减速段
			{
				if (vm - ve > 0)
				{
					T[4] = pow((vm - ve) / J, 0.5);
				}
				else
				{
					T[4] = 0;
				}
			}
		}
		else  // 需要先减速后加速
		{
			// 寻找能够达到的中间速度
			NewTonSecantMethodToFindRoot(-V, V, s, vs, ve, Acc, J, input->secant_eps, input->max_itera_time, &vm);
			if (vs - vm > Acc*Acc / J)  // 有匀减速段
			{
				T[2] = T[0] = Acc / J;
				T[1] = (vs - vm - Acc2invJ) / Acc;
				T[3] = 0;
			}
			else  // 没有匀减速段
			{
				if (vs - vm > 0)
				{
					T[0] = pow((vs - vm) / J, 0.5);
				}
				else
				{
					T[0] = 0;
				}
				T[2] = T[0];
				T[1] = 0;
				T[3] = 0;
			}
			if (ve - vm > Acc2invJ)  // 有匀加速段
			{
				T[6] = T[4] = Acc / J;
				T[5] = (ve - vm - Acc2invJ) / Acc;
			}
			else  // 没有匀加速段
			{
				if (ve - vm > 0)
				{
					T[4] = pow((ve - vm) / J, 0.5);
				}
				else
				{
					T[4] = 0;
				}
				T[6] = T[4];
				T[5] = 0;
			}
			J = -J;  // 需要把J反向
		}
	}

	// 时间缩放，时间取整
	T[0] = floor(input->scale*T[0] / sample_time)*sample_time;
	T[1] = floor(input->scale*T[1] / sample_time)*sample_time;
	T[2] = floor(input->scale*T[2] / sample_time)*sample_time;
	T[3] = floor(input->scale*T[3] / sample_time)*sample_time;
	T[4] = floor(input->scale*T[4] / sample_time)*sample_time;
	T[5] = floor(input->scale*T[5] / sample_time)*sample_time;
	T[6] = floor(input->scale*T[6] / sample_time)*sample_time;
	T[7] = floor(input->scale*T[7] / sample_time)*sample_time;

	// 时间取整后的补偿
	if (T[0] + T[1] + T[2] + T[3] + T[4] + T[5] + T[6] + T[7] < 1E-10)
	{
		return -1;
	}
	vm = (s - ve*(0.5*T[5] + T[4]) - vs*(T[0] + 0.5*T[1])) / (T[0] + 0.5*T[1] + T[3] + T[4] + 0.5*T[5]);
	if (T[0] < 1E-10)
	{
		J1 = 0;
	}
	else
	{
		J1 = (vm - vs) / (T[0] * T[0] + T[0] * T[1]);
	}
	if (T[4] < 1E-10)
	{
		J2 = 0;
	}
	else
	{
		J2 = (vm - ve) / (T[4] * T[4] + T[4] * T[5]);
	}

	motion_tra->jerk_series[0] = J1;
	motion_tra->jerk_series[1] = 0;
	motion_tra->jerk_series[2] = -J1;
	motion_tra->jerk_series[3] = 0;
	motion_tra->jerk_series[4] = -J2;
	motion_tra->jerk_series[5] = 0;
	motion_tra->jerk_series[6] = J2;
	motion_tra->jerk_series[7] = 0;
	motion_tra->acc_series[0] = 0;
	motion_tra->vel_series[0] = vs;
	motion_tra->pos_series[0] = 0;

	for (counter = 1; counter < 8; counter++)
	{
		// acc
		calculate_acceleration_from_profile_(T[counter - 1], \
			motion_tra->jerk_series[counter - 1], \
			motion_tra->acc_series[counter - 1], \
			&motion_tra->acc_series[counter]);
		// vel
		calculate_velocity_from_profile_(T[counter - 1], \
			motion_tra->jerk_series[counter - 1], \
			motion_tra->acc_series[counter - 1], \
			motion_tra->vel_series[counter - 1], \
			&motion_tra->vel_series[counter]);
		// pos
		calculate_position_from_profile_(T[counter - 1], \
			motion_tra->jerk_series[counter - 1], \
			motion_tra->acc_series[counter - 1], \
			motion_tra->vel_series[counter - 1], \
			motion_tra->pos_series[counter - 1], \
			&motion_tra->pos_series[counter]);
	}


	if (reverse_flag == 1)
	{
		for (counter = 0; counter < 8; counter++)
		{
			motion_tra->jerk_series[counter] = -motion_tra->jerk_series[counter];
			motion_tra->acc_series[counter] = -motion_tra->acc_series[counter];
			motion_tra->vel_series[counter] = -motion_tra->vel_series[counter];
			motion_tra->pos_series[counter] = -motion_tra->pos_series[counter];
		}
	}


	return 0;
}
// 根据加速度为梯形，速度变化dv后的位移。
double DisplacemenInDeltaV(double v,double dv,double Acc,double J)
{
	double invJ = 1.0 / J;
	double abs_dv = fabs(dv);
	if (abs_dv < Acc*Acc*invJ)
	{
		return (2 * v + dv)*pow(abs_dv*invJ, 0.5);
	}
	else
	{
		return (2 * v + dv)*(abs_dv + Acc*Acc*invJ) / (2 *Acc);
	}
	return 0;
}

int16 NewTonSecantMethodToFindRoot(double x0,double x1,double s,double vs,double ve,double Acc,double J,double eps,int16 max_iter,double* root)
{
	int16 ita_time = 0;
	double fk0 = 0;
	double fk1 = 0;
	double fk2 = 0;
	double xk0 = 0;
	double xk1 = 0;
	double xk2 = 0;

	fk0 = DisplacemenInDeltaV(vs, x0 - vs, Acc, J) + DisplacemenInDeltaV(ve, x0 - ve, Acc, J) - s;
	if (fabs(fk0) < eps)
	{
		*root = x0;
		return 0;
	}
	fk1 = DisplacemenInDeltaV(vs, x1 - vs, Acc, J) + DisplacemenInDeltaV(ve, x1 - ve, Acc, J) - s;
	if (fabs(fk1) < eps)
	{
		*root = x1;
		return 0;
	}
	xk1 = x1;
	xk0 = x0;

	while (1)
	{
		ita_time++;
		xk2 = xk1 - fk1 / (fk1 - fk0)*(xk1 - xk0);
		xk0 = xk1;
		xk1 = xk2;
		fk0 = fk1;
		fk1 = DisplacemenInDeltaV(vs, xk1 - vs, Acc, J) + DisplacemenInDeltaV(ve, xk1 - ve, Acc, J) - s;
		if (fabs(fk1) < eps)
		{
			*root = xk1;
			break;
		}
		if (ita_time > max_iter)
		{
			printf("迭代超过最大次数%d\n", max_iter);
			return -1;
		}
	}

	return 0;
}
