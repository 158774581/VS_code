#include "stdafx.h"
/*
* solver.c
*
*  Created on: Mar 13, 2018
*  Author: root
*  http://www.mathemania.com/lesson/cardanos-formula-solving-cubic-equations/
*  https://en.wikipedia.org/wiki/Cubic_function#math_1
*  https://en.wikipedia.org/wiki/Quartic_function#Ferrari's_solution
*
*/

#include <math.h>
#include "solver.h"
//-----------------------------------------------------------
//quadratic equation solver
//
//x^2 + c0 x + c1 =0
//
//discriminant =0.25*c0^2-c1;
//
//if discriminant < 0 no root
//
//if discriminant > 0 has two different roots
//
//if discriminant = 0 has twp same roots
//-----------------------------------------------------------

void quadratic_solver(QUADRATIC_SOLVER* m_qs)
{

	double discriminant;
	double tmp;

	discriminant = 0.25 * m_qs->c0 * m_qs->c0 - m_qs->c1;

	if (fabs(discriminant) < 1.0E-10)
	{
		//has same roots
		m_qs->r0 = -0.5 * m_qs->c0;
		m_qs->r1 = m_qs->r0;

		m_qs->root_type = QUADRATIC_TWO_SAME_ROOTS;
	}
	else
	{
		if (discriminant < 0.0)
		{
			m_qs->r0 = 0;
			m_qs->r1 = 0;
			m_qs->root_type = QUADRATIC_NO_ROOTS;
		}
		else
		{
			tmp = sqrt(discriminant);
			//has same roots
			m_qs->r0 = -0.5 * m_qs->c0 + tmp;
			m_qs->r1 = -m_qs->c0 - m_qs->r0;

			m_qs->root_type = QUADRATIC_TWO_DIFF_ROOTS;
		}
	}
}

//-----------------------------------------------------------
//cubic equation solver
//
//a x^3 + b x^2 + c x + d = 0 (1)  a!=0
//Reduction to a depressed cubic
//c0 = b/a 
//c1 = c/a
//c2 = d/a 
//Dividing by a and substituting t − b/3a = x
//we get the equation t^3 + pt+ q = 0; (2)
//where p = (3ac-b^2)/3a^2 = c1-1/3* c0^2
//where q = (2b^3-9abc+27a^2d)/27a^3 = 2/27*c0^3-1/3*c0*c1 +c2
//The left hand side of equation (2) is called a depressed cubic, because the quadratic term has coefficient 0.

//using Cardano's method,which used in the depressed cubic
//We introduce two variables u and v linked by the condition u + v = t and substitute this in the depressed cubic (2),
//so we get u^3 + v^3 + ( 3 uv + p ) (u+v) + q =0
//at this point Cardano imposed a second condition for the variables u and v: 3uv + p = 0
//so we can get  u^3 + v^3 = −q(because the 3uv+p = 0) and u^3*v^3 = − p^3/27(3uv=-p,in both side do cubic)
//The combination of these two equations leads to a quadratic equation (since they are the sum and the product of u^3 and v^3).
//Thus u^3 and v^3 are the two roots of the quadratic equation z^2 + qz − p^3/27 = 0,
//so we can get discriminant : q^2/4+ p^3/27
//If discriminant > 0, then the cubic equation has one real and two complex conjugate roots;
//if discriminant = 0, then the equation has three real roots, whereby at least two roots are equal;
//if discriminant < 0 then the equation has three distinct real roots.
//-----------------------------------------------------------
int16 cubic_solver(CUBIC_SOLVER* m_cs)
{
	double tmp, tmp1;
	double p, q;
	double u3, v3;
	double u, v, t1, t2, t3;
	double discriminant;

	//-------------------------------------------------
	tmp = 0.3333333333333333 * m_cs->c0;
	//-------------------------------------------------
	//calculate p and q so we can get t^3 + pt+ q = 0;
	p = m_cs->c1 - tmp*m_cs->c0;
	q = 0.0740740740740741*m_cs->c0 *m_cs->c0*m_cs->c0 - tmp*m_cs->c1 + m_cs->c2;
	//-------------------------------------------------
	//special case 1: t^3 = 0 t = 0,so we get three same roots
	if ((fabs(p) < 1.0E-10) && (fabs(q) < 1.0E-10))
	{
		//t1 = t2 = t3 = 0;
		m_cs->r0 = -tmp;
		m_cs->r1 = -tmp;
		m_cs->r2 = -tmp;
		m_cs->root_type = CUBIC_THREE_SAME_REAL_SPECIAL_EQU0_ROOTS; //but m = 0 is illegal for next calculate
		return 0;
	}
	else if (fabs(p) < 1.0E-10)
	{
		//t^3 = -q ;
		if (q < 0.0)
		{
			u = pow(-q, 0.33333333333333333);//q<0 u>0 
		}
		else
		{
			u = -pow(q, 0.33333333333333333);//q>0 u<0
		}
		m_cs->r0 = u - tmp;
		m_cs->r1 = m_cs->r0; //r0 and r1 and r2 are real and same
		m_cs->r2 = m_cs->r0;

		if (m_cs->r0 > 0)
		{
			m_cs->root_type = CUBIC_THREE_SAME_REAL_SPECIAL_BIG0_ROOTS;
		}
		else
		{
			//maybe zero ,but we do not care ,because zero or negtive is same for me 
			m_cs->root_type = CUBIC_THREE_SAME_REAL_SPECIAL_LESS0_ROOTS; //but m <0 is illegal for next calculate		
		}
		return 0;
	}
	else if (fabs(q) < 1.0E-10)
	{
		//t^3 + pt = 0;
		//t(t^2+p) = 0
		//we can get t = 0 and t = sqrt(-p)
		m_cs->r0 = -tmp;
		if (p > 0)
		{
			//no root
			m_cs->r1 = 0;
			m_cs->r2 = 0;

			if (m_cs->r0 > 0)
			{
				m_cs->root_type = CUBIC_ONLY_ONE_REAL_SPECIAL_R0_ROOTS;
				return 0;
			}
			else
			{
				//maybe zero ,but we do not care ,because zero or negtive is same for me 
				m_cs->root_type = CUBIC_NO_REAL_ROOTS; //but m <0 is illegal for next calculate
				return -1;
			}
		}
		else
		{
			m_cs->r1 = sqrt(-p) - tmp; // r1 and r2 are real and same
			m_cs->r2 = m_cs->r1;

			if (m_cs->r1 > 0)
			{
				m_cs->r0 = m_cs->r1;
				m_cs->root_type = CUBIC_TWO_SAME_ONE_DIFF_REAL_SPECIAL_R1_ROOTS;
				//return 0;
			}
			else if (m_cs->r0 > 0)
			{
				m_cs->root_type = CUBIC_TWO_SAME_ONE_DIFF_REAL_SPECIAL_R0_ROOTS;
				//return 0;
			}
			else
			{
				//maybe zero ,but we do not care ,because zero or negtive is same for me 
				m_cs->root_type = CUBIC_NO_REAL_ROOTS; //but m <0 is illegal for next calculate
				return -1;
			}

			//if they all positive, choose the bigger one 
			if ((m_cs->r1 > 0) && (m_cs->r0 > 0))
			{
				if (m_cs->r0 < m_cs->r1)
				{
					m_cs->r0 = m_cs->r1;
				}
				m_cs->root_type = CUBIC_TWO_SAME_ONE_DIFF_REAL_SPECIAL_BIGGER_ROOTS;
			}
			return 0;
		}
	}
	//--------------------------------------------------------
	//Cardano's method
	// u^3 + v^3 = −q(because the 3uv+p = 0) and u^3*v^3 = − p^3/27(3uv=-p,in both side do cubic)
	//The combination of these two equations leads to a quadratic equation (since they are the sum and the product of u^3 and v^3).
	//Thus u^3 and v^3 are the two roots of the quadratic equation z^2 + qz − p^3/27 = 0,
	//so we can get discriminant : q^2/4+ p^3/27

	discriminant = 0.037037037037037 * p * p * p + 0.25 * q * q;

	//accoring to the discriminant,we can get the cubic root
	if (fabs(discriminant) < 1.0E-10)
	{
		//discriminant is zero ,so the discriminant = -0.5*q
		//u^3 = -0.5*q;
		//v^3 = -0.5*q;

		//q^2/4+ p^3/27 = 0 is equivalent to asserting that
		//
		//(27*q^2)/(4*p^3)= −1. (1)
		//
		//If u = v = 3*q/2*p, then u^3 = v^3 = (27*q^3)/(8*p^3)= −q/2 (using (1))
		//
		//and 3uv = 27*q^2/4*p^2 = −p.
		//
		//ξ = -0.5+(sqrt(3)/2)*i
		//ξ'= -0.5+-(sqrt(3)/2)*i 
		//ξ' = ξ^2
		//ξ^3 = 1
		//
		//ξu * ξ'v  = ξ'u * ξv = ξ^3*u*v = u*v = -p/3
		//Thus, Cardano's formula says that the roots of
		//u + v = 2u = 3q/p
		//ξu + ξ'v = −3q/2p
		//and ξ'u + ξv =  −3q/2p. they are same
		//so they have three real roots, the second and third are same

		t1 = 3 * q / p;
		t2 = -3 * q / 2 * p;
		t3 = t2;
		m_cs->r0 = t1 - tmp;
		m_cs->r1 = t2 - tmp;					//t2 and t3 are same
		m_cs->r2 = m_cs->r1;

		//because r0 and r1 are different sign
		if (m_cs->r0 > 0)
		{
			m_cs->root_type = CUBIC_TWO_SAME_ONE_DIFF_REAL_R0_ROOTS;
		}
		else if (m_cs->r1 > 0)
		{
			m_cs->r0 = m_cs->r1;
			m_cs->root_type = CUBIC_TWO_SAME_ONE_DIFF_REAL_R1_ROOTS;
		}
		else
		{
			m_cs->root_type = CUBIC_NO_REAL_ROOTS; //but m <0 is illegal for next calculate
			return -1;
		}
	}
	else if (discriminant > 0)
	{
		//If p and q are real numbers, so u and v are real
		//the roots are as follows
		//u + v 
		//ξu + ξ'v 
		//ξ'u + ξv
		//so we can know that r1 and r2 are complex number because u and v are different
		//so we just think about r0 

		u3 = -0.5*q + sqrt(discriminant);
		v3 = -0.5*q - sqrt(discriminant);

		if ((u3) > 0.0)
		{
			u = pow(u3, 0.33333333333333333);
		}
		else
		{
			u = -pow(-u3, 0.33333333333333333);
		}
		if ((v3) > 0.0)
		{
			v = pow(v3, 0.33333333333333333);
		}
		else
		{
			v = -pow(-v3, 0.33333333333333333);
		}
		t1 = u + v;

		m_cs->r0 = t1 - tmp;
		m_cs->r1 = 0; 	//r1 and r2 are complex conjugate,we do not need them
		m_cs->r2 = 0;
		if (m_cs->r0 > 0)
		{
			m_cs->root_type = CUBIC_ONE_REAL_TWO_COMPLEX_ROOTS;
		}
		else
		{
			m_cs->root_type = CUBIC_NO_REAL_ROOTS; //but m <0 is illegal for next calculate
			return -1;
		}
	}
	else
	{
		//  Viete trigonometric expression of the roots in the three-real-roots case
		// because in this situation ,p must be negtive then the discriminant <0

		tmp1 = acos((1.5*q / p)*sqrt(-3 / p)) * 0.33333333333333333;

		t1 = 2 * sqrt(-p*0.33333333333333333)*cos(tmp1);
		t2 = 2 * sqrt(-p*0.33333333333333333)*cos(tmp1 - 2.094395066666667); //2pi/3
		t3 = 2 * sqrt(-p*0.33333333333333333)*cos(tmp1 - 4.188790133333333); //4pi/3
		m_cs->r0 = t1 - tmp;
		m_cs->r1 = t2 - tmp;
		m_cs->r2 = t3 - tmp;
		if (m_cs->r0 > 0)
		{
			m_cs->root_type = CUBIC_THREE_DIFF_REAL_R0_ROOTS;
		}
		else if (m_cs->r1 > 0)
		{
			m_cs->r0 = m_cs->r1;
			m_cs->root_type = CUBIC_THREE_DIFF_REAL_R1_ROOTS;
		}
		else if (m_cs->r2 > 0)
		{
			m_cs->r0 = m_cs->r2;
			m_cs->root_type = CUBIC_THREE_DIFF_REAL_R2_ROOTS;
		}
		//if have two all positive
		//tbd
	}
	return 0;
}
//-----------------------------------------------------------
//  quartic solver
//  a4x^4 + a3 x^3 + a2 x^2 + a1 x + a0 = 0
//  x^4 + b x^3 + c x^2 + d x + e = 0  b = a3/a4, c = a2/a4, d = a1/a4, and e = a0/a4
//  c0 = b
//  c1 = c
//  c2 = d
//  c3 = e
//  x = y − b/4
//  y^4 + py^2 + qy + r = 0,

// p = (8c-3b^2)/8 = (8a2*a4-3*a3^2)/8*a4^2 = c1-3/8*c0^2

// q = (b^3-4bc+8d)/8 = (a3^3 - 4a2*a3*a4+8*a1*a4^2)/(8*a4^3) = 1/8*c0^3 -0.5*c0*c1+c2

// r = (-3b^4+256e-64bd+16b^2c)/256 = (-3*a3^4+256a0*a4^3-64a1*a3*a4^2+16a2*a3^2*a4)/256a4^4 = -3/256*c0^4 +c3 -0.25c0*c2 +1/16* c0^2*c1

// As explained in the preceding section, we may start with the depressed quartic equation y^4 + py^2 + qy + r = 0,

// expanding the square and regrouping all terms in the left-hand side

// we get (y^2+0.5p)^2 = -qy-r+p^2/4

//  we introduce a variable m into the factor on the left-hand side so we can get (y^2+0.5p + m)^2 = -qy-r+p^2/4+(2y^2m+pm+m^2)

// we choose m to let  2my^2-qy+m^2+mp+p^2/4-r to be perfect square in the right-hand side  discriminant in y of this quadratic equation is zero

//so we get discriminant =  8m^3+8pm^2+(2p^2-8r)m-q^2=0 we can use cubic to calculate this equation

//if m is a root of the cubic equation such that m > 0, so we can get

//(y^2+p/2+m)^2=(y *sqrt(2m)-q/(2*sqrt(2m)))^2.

//This equation is of the form M^2 = N^2, which can be rearranged as M^2 − N^2 = 0 or (M + N)(M − N) = 0

int16 quartic_solver(QUARTIC_SOLVER* m_qs)
{

	double tmp, tmp1;
	double p, q, r, m;
	int16 flag;
	QUADRATIC_SOLVER qs, qs0, qs1;
	CUBIC_SOLVER cs;
	int rtn;

	tmp = 0.25 * m_qs->c0;
	tmp1 = m_qs->c0 * m_qs->c0;
	//---------------------------------
	//first use x = y − b/4 ,change to y,so we get p,q,r
	p = m_qs->c1 - 0.375 * tmp1;

	q = (0.125 * tmp1 - 0.5 * m_qs->c1)* m_qs->c0 + m_qs->c2;

	r = (-0.01171875* tmp1*tmp1) + m_qs->c3 - (0.25*m_qs->c0*m_qs->c2) + (0.0625*tmp1*m_qs->c1);
	//---------------------------------
	// special case 1:  m = 0 => q = 0  from this formula  8m^3+8pm^2+(2p^2-8r)m-q^2=0
	// Biquadratic equations
	// y^4 + py^2 + r = 0, we can let z = y^2
	// z^2 + pz + r = 0;
	//---------------------------------
	if (fabs(q) < 1.0E-10)
	{
		qs.c0 = p;
		qs.c1 = r;
		quadratic_solver(&qs);

		switch (qs.root_type)
		{
		case QUADRATIC_NO_ROOTS:
		{
			m_qs->root_type = QUARTIC_NO_REAL_ROOTS;
			return -1;
		}
		break;
		case QUADRATIC_TWO_SAME_ROOTS:
			if (qs.r0 >= 0)
			{
				m_qs->r0 = sqrt(qs.r0) - tmp;
				m_qs->r1 = -sqrt(qs.r0) - tmp;  //because c0 >0 so,-tmp<0 ,so r1 <0 .this is illegal for t_acc
				m_qs->r2 = m_qs->r0;
				m_qs->r3 = m_qs->r1;
				//check whether r0 >=0 
				if (m_qs->r0 >= 0)
				{
					//choose r0
					m_qs->root_type = QUARTIC_SPECIAL_REAL_ROOT_R0;
				}
				else
				{
					m_qs->root_type = QUARTIC_NO_REAL_ROOTS;
					return -1;
				}
			}
			else
			{
				//y^2 < 0 impossible
				m_qs->root_type = QUARTIC_NO_REAL_ROOTS;
				return -1;
			}
			break;
		case QUADRATIC_TWO_DIFF_ROOTS:
			flag = 0;
			if (qs.r0 >= 0)
			{
				m_qs->r0 = sqrt(qs.r0) - tmp;
				m_qs->r1 = -sqrt(qs.r0) - tmp;  //because c0 >0 so,-tmp<0 ,so r1 <0 .this is illegal for t_acc
			}
			else
			{
				//y^2 < 0 impossible
				flag |= 1;
			}
			if (qs.r1 >= 0)
			{
				m_qs->r2 = sqrt(qs.r1) - tmp;
				m_qs->r3 = -sqrt(qs.r1) - tmp;  //because c0 >0 so,-tmp<0 ,so r1 <0 .this is illegal for t_acc
			}
			else
			{
				//y^2 < 0 impossible
				flag |= 2;
			}
			switch (flag)
			{
			case 0:
				//now we have r0 and r2 are ok,so check ro and r2 continue
				if (m_qs->r0 >= 0)
				{
					m_qs->root_type = QUARTIC_SPECIAL_REAL_ROOT_R0;
				}
				else if (m_qs->r2 >= 0)
				{
					m_qs->r0 = m_qs->r2;
					m_qs->root_type = QUARTIC_SPECIAL_REAL_ROOT_R2;
				}
				else
				{
					// no real roots
					m_qs->root_type = QUARTIC_NO_REAL_ROOTS;
					return -1;
				}
				//if the two root are all positive,choose the big one
				if ((m_qs->r0 >= 0) && (m_qs->r2 >= 0))
				{
					//output is r0
					m_qs->root_type = QUARTIC_SPECIAL_REAL_ROOT_BIGGER;
					if (m_qs->r0 < m_qs->r2)
					{
						m_qs->r0 = m_qs->r2;
					}
				}
				break;
			case 1:
				if (m_qs->r2 >= 0)
				{
					m_qs->r0 = m_qs->r2;
					m_qs->root_type = QUARTIC_SPECIAL_REAL_ROOT_R2;
				}
				else
				{
					// no real roots
					m_qs->root_type = QUARTIC_NO_REAL_ROOTS;
					return -1;
				}
				break;
			case 2:
				if (m_qs->r0 >= 0)
				{
					m_qs->root_type = QUARTIC_SPECIAL_REAL_ROOT_R0;
				}
				else
				{
					// no real roots
					m_qs->root_type = QUARTIC_NO_REAL_ROOTS;
					return -1;
				}
				break;
			case 3:
				m_qs->root_type = QUARTIC_NO_REAL_ROOTS;
				return -1;
				break;
			default:
				break;
			}
			break;
		default:
			break;
		}
		return 0;
	}
	//--------------------------------------------------------
	//we get m value to produce the two quadratic equation
	//first get the m ,m satisfied the formula as 8m ^ 3 + 8pm ^ 2 + (2p ^ 2 - 8r)m - q ^ 2 = 0
	//c0 = p
	//c1 = 0.25*p^2-r
	//c2 = -1/8*q^2   c2 <0
	cs.c0 = p;
	cs.c1 = 0.25*p*p - r;
	cs.c2 = -0.125*q*q;

	//we need get the m >0 to continue the calculate 
	//m = 0 is special ,has been calculated individual
	rtn = cubic_solver(&cs);
	if (rtn != 0)
	{
		return -1;
	}

	switch (cs.root_type)
	{
	case CUBIC_NO_REAL_ROOTS:
		return -2;
		break;
	case CUBIC_THREE_SAME_REAL_SPECIAL_EQU0_ROOTS:  //m = 0
		return -3;
		break;
	case CUBIC_THREE_SAME_REAL_SPECIAL_BIG0_ROOTS:
	case CUBIC_ONLY_ONE_REAL_SPECIAL_R0_ROOTS:
	case CUBIC_TWO_SAME_ONE_DIFF_REAL_SPECIAL_R1_ROOTS:
	case CUBIC_TWO_SAME_ONE_DIFF_REAL_SPECIAL_R0_ROOTS:
	case CUBIC_TWO_SAME_ONE_DIFF_REAL_SPECIAL_BIGGER_ROOTS:
	case CUBIC_TWO_SAME_ONE_DIFF_REAL_R0_ROOTS:
	case CUBIC_TWO_SAME_ONE_DIFF_REAL_R1_ROOTS:
	case CUBIC_ONE_REAL_TWO_COMPLEX_ROOTS:
	case CUBIC_THREE_DIFF_REAL_R0_ROOTS:
	case CUBIC_THREE_DIFF_REAL_R1_ROOTS:
	case CUBIC_THREE_DIFF_REAL_R2_ROOTS:
		m = cs.r0;
		break;
	default:
		break;
	}
	//--------------------------------------------------------
	//calculate the two quadratic equation
	// now m>0
	qs0.c0 = sqrt(2 * m);
	qs0.c1 = 0.5*p + m - (0.5*q) / (qs0.c0);
	quadratic_solver(&qs0);

	qs1.c0 = -sqrt(2 * m);
	qs1.c1 = 0.5*p + m + (0.5*q) / (-qs1.c0);
	quadratic_solver(&qs1);


	switch ((qs0.root_type) | (qs1.root_type << 8))
	{
	case (QUADRATIC_NO_ROOTS | (QUADRATIC_NO_ROOTS << 8)):
		m_qs->root_type = QUARTIC_NO_REAL_ROOTS;
		return -1;
		break;
	case (QUADRATIC_TWO_SAME_ROOTS | (QUADRATIC_NO_ROOTS << 8)):

		m_qs->r0 = qs0.r0 - tmp;
		if (m_qs->r0 > 0)
		{
			m_qs->root_type = QUARTIC_TWO_SAME_01_REAL_ROOTS;
		}
		else
		{
			m_qs->root_type = QUARTIC_NO_REAL_ROOTS;
			return -1;
		}
		break;
	case (QUADRATIC_NO_ROOTS | (QUADRATIC_TWO_SAME_ROOTS << 8)):
		m_qs->r0 = qs1.r0 - tmp;
		if (m_qs->r0 > 0)
		{
			m_qs->root_type = QUARTIC_TWO_SAME_23_REAL_ROOTS;
		}
		else
		{
			m_qs->root_type = QUARTIC_NO_REAL_ROOTS;
			return -1;
		}
		break;
	case (QUADRATIC_TWO_DIFF_ROOTS | (QUADRATIC_NO_ROOTS << 8)):
		m_qs->r0 = qs0.r0 - tmp;
		m_qs->r1 = qs0.r1 - tmp;
		if (m_qs->r0 > 0)
		{
			m_qs->root_type = QUARTIC_TWO_DIFF_01_REAL_ROOTS;
		}
		else if (m_qs->r1 > 0)
		{
			m_qs->r0 = m_qs->r1;
			m_qs->root_type = QUARTIC_TWO_DIFF_01_REAL_ROOTS;
		}
		else
		{
			m_qs->root_type = QUARTIC_NO_REAL_ROOTS;
			return -1;
		}
		break;
	case (QUADRATIC_NO_ROOTS | (QUADRATIC_TWO_DIFF_ROOTS << 8)):
		m_qs->r0 = qs1.r0 - tmp;
		m_qs->r1 = qs1.r1 - tmp;
		if (m_qs->r0 > 0)
		{
			m_qs->root_type = QUARTIC_TWO_DIFF_23_REAL_ROOTS;
		}
		else if (m_qs->r1 > 0)
		{
			m_qs->r0 = m_qs->r1;
			m_qs->root_type = QUARTIC_TWO_DIFF_23_REAL_ROOTS;
		}
		else
		{
			m_qs->root_type = QUARTIC_NO_REAL_ROOTS;
			return -1;
		}
		break;
	case (QUADRATIC_TWO_SAME_ROOTS | (QUADRATIC_TWO_SAME_ROOTS << 8)):
		m_qs->r0 = qs0.r0 - tmp;
		m_qs->r1 = qs1.r0 - tmp;
		if (m_qs->r0 > 0)
		{
			m_qs->root_type = QUARTIC_TWO_SAME_TWO_SAME_REAL_ROOTS;
		}
		else if (m_qs->r1 > 0)
		{
			m_qs->r0 = m_qs->r1;
			m_qs->root_type = QUARTIC_TWO_SAME_TWO_SAME_REAL_ROOTS;
		}
		else
		{
			m_qs->root_type = QUARTIC_NO_REAL_ROOTS;
			return -1;
		}
		break;
	case (QUADRATIC_TWO_SAME_ROOTS | (QUADRATIC_TWO_DIFF_ROOTS << 8)):
		m_qs->r0 = qs0.r0 - tmp;
		m_qs->r1 = qs1.r0 - tmp;
		m_qs->r2 = qs1.r1 - tmp;
		if (m_qs->r0 > 0)
		{
			m_qs->root_type = QUARTIC_TWO_SAME_TWO_DIFF_REAL_ROOTS;
		}
		else if (m_qs->r1 > 0)
		{
			m_qs->r0 = m_qs->r1;
			m_qs->root_type = QUARTIC_TWO_SAME_TWO_DIFF_REAL_ROOTS;
		}
		else if (m_qs->r2 > 0)
		{
			m_qs->r0 = m_qs->r2;
			m_qs->root_type = QUARTIC_TWO_SAME_TWO_DIFF_REAL_ROOTS;
		}
		else
		{
			m_qs->root_type = QUARTIC_NO_REAL_ROOTS;
			return -1;
		}
		break;
	case (QUADRATIC_TWO_DIFF_ROOTS | (QUADRATIC_TWO_SAME_ROOTS << 8)):
		m_qs->r0 = qs0.r0 - tmp;
		m_qs->r1 = qs0.r1 - tmp;
		m_qs->r2 = qs1.r0 - tmp;
		if (m_qs->r0 > 0)
		{
			m_qs->root_type = QUARTIC_TWO_DIFF_TWO_SAME_REAL_ROOTS;
		}
		else if (m_qs->r1 > 0)
		{
			m_qs->r0 = m_qs->r1;
			m_qs->root_type = QUARTIC_TWO_DIFF_TWO_SAME_REAL_ROOTS;
		}
		else if (m_qs->r2 > 0)
		{
			m_qs->r0 = m_qs->r2;
			m_qs->root_type = QUARTIC_TWO_DIFF_TWO_SAME_REAL_ROOTS;
		}
		else
		{
			m_qs->root_type = QUARTIC_NO_REAL_ROOTS;
			return -1;
		}
		break;
	case (QUADRATIC_TWO_DIFF_ROOTS | (QUADRATIC_TWO_DIFF_ROOTS << 8)):
		m_qs->r0 = qs0.r0 - tmp;
		m_qs->r1 = qs0.r1 - tmp;
		m_qs->r2 = qs1.r0 - tmp;
		m_qs->r3 = qs1.r1 - tmp;
		if (m_qs->r0 > 0)
		{
			m_qs->root_type = QUARTIC_TWO_DIFF_TWO_DIFF_REAL_ROOTS;
		}
		else if (m_qs->r1 > 0)
		{
			m_qs->r0 = m_qs->r1;
			m_qs->root_type = QUARTIC_TWO_DIFF_TWO_DIFF_REAL_ROOTS;
		}
		else if (m_qs->r2 > 0)
		{
			m_qs->r0 = m_qs->r2;
			m_qs->root_type = QUARTIC_TWO_DIFF_TWO_DIFF_REAL_ROOTS;
		}
		else if (m_qs->r3 > 0)
		{
			m_qs->r0 = m_qs->r3;
			m_qs->root_type = QUARTIC_TWO_DIFF_TWO_DIFF_REAL_ROOTS;
		}
		else
		{
			m_qs->root_type = QUARTIC_NO_REAL_ROOTS;
			return -1;
		}
		break;
	default:
		break;
	}

	return 0;
}
