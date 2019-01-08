
// mat_test.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "matrix.h"

#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
static double dmaxarg1, dmaxarg2;
#define DMAX(a,b) (dmaxarg1=(a),dmaxarg2=(b),(dmaxarg1) > (dmaxarg2) ? (dmaxarg1) : (dmaxarg2))
static int iminarg1, iminarg2;
#define IMIN(a,b) (iminarg1=(a),iminarg2=(b),(iminarg1) < (iminarg2) ? (iminarg1) : (iminarg2))
void mprintf(double** a, int m, int n)
{
	int i, j;
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			printf("%lf  ", a[i][j]);
		}
		printf("\n");
	}
}
void vecprintf(double* v, int n)
{
	int j;
	for (j = 0; j < n; j++)
	{
		printf("%lf  ", v[j]);
	}
}

double pythag(double a, double b)
/* compute (a2 + b2)^1/2 without destructive underflow or overflow */
{
	double absa, absb;
	absa = fabs(a);
	absb = fabs(b);
	if (absa > absb) return absa*sqrt(1.0 + (absb / absa)*(absb / absa));
	else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0 + (absa / absb)*(absa / absb)));
}

/*******************************************************************************
 singular value decomposition  A = U.W.VT. A=m*n
 input : mat=AT ,vec=n*1 ,VT=n*n.
 output : mat=UT ,vec=n*1(sigular value) ,VT=VT.
*******************************************************************************/
void svdcmp(matxx* mat, matxx* vec, matxx* VT)

{
	int flag, i, its, j, jj, k, l, nm, m, n;
	double anorm, c, f, g, h, s, scale, x, y, z;// , *rv1;
	double rv1[10];
	double **a;
	double **v;
	double *w;

	a = mat->point;
	n = mat->row;
	m = mat->column;
	w = *(vec->point);
	v = VT->point;

	g = scale = anorm = 0.0; /* Householder reduction to bidiagonal form */
	for (i = 0; i < n; i++) {
		l = i + 1;
		rv1[i] = scale*g;
		g = s = scale = 0.0;
		if (i < m) {
			for (k = i; k < m; k++)
				scale += fabs(a[k][i]);
			if (scale) {
				for (k = i; k < m; k++) {
					a[k][i] /= scale;
					s += a[k][i] * a[k][i];
				}
				f = a[i][i];
				g = -SIGN(sqrt(s), f);
				h = f*g - s;
				a[i][i] = f - g;
				for (j = l; j < n; j++) {
					for (s = 0.0, k = i; k < m; k++) s += a[k][i] * a[k][j];
					f = s / h;
					for (k = i; k < m; k++) a[k][j] += f*a[k][i];
				}
				for (k = i; k < m; k++) a[k][i] *= scale;
			}
		}
		w[i] = scale *g;
		g = s = scale = 0.0;
		if (i < m && i != n - 1) {
			for (k = l; k < n; k++) scale += fabs(a[i][k]);
			if (scale) {
				for (k = l; k < n; k++) {
					a[i][k] /= scale;
					s += a[i][k] * a[i][k];
				}
				f = a[i][l];
				g = -SIGN(sqrt(s), f);
				h = f*g - s;
				a[i][l] = f - g;
				for (k = l; k < n; k++) rv1[k] = a[i][k] / h;
				for (j = l; j < m; j++) {
					for (s = 0.0, k = l; k < n; k++) s += a[j][k] * a[i][k];
					for (k = l; k < n; k++) a[j][k] += s*rv1[k];
				}
				for (k = l; k < n; k++) a[i][k] *= scale;
			}
		}
		anorm = DMAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
	}
	mprintf(a, 4, 3);
	//vecprintf(rv1, 3);
	l = l - 1;
	for (i = n - 1; i >= 0; i--) { /* Accumulation of right-hand transformations. */
		if (i < n - 1) {
			if (g) {
				for (j = l; j < n; j++) /* Double division to avoid possible underflow. */
					v[j][i] = (a[i][j] / a[i][l]) / g;
				for (j = l; j < n; j++) {
					for (s = 0.0, k = l; k < n; k++) s += a[i][k] * v[k][j];
					for (k = l; k < n; k++) v[k][j] += s*v[k][i];
				}
			}
			for (j = l; j < n; j++) v[i][j] = v[j][i] = 0.0;
		}
		v[i][i] = 1.0;
		g = rv1[i];
		l = i;
	}
	//mprintf(v, 3, 3);

	for (i = IMIN(m, n) - 1; i >= 0; i--) { /* Accumulation of left-hand transformations. */
		l = i + 1;
		g = w[i];
		for (j = l; j < n; j++) a[i][j] = 0.0;
		if (g) {
			g = 1.0 / g;
			for (j = l; j < n; j++) {
				for (s = 0.0, k = l; k < m; k++) s += a[k][i] * a[k][j];
				f = (s / a[i][i])*g;
				for (k = i; k < m; k++) a[k][j] += f*a[k][i];
			}
			for (j = i; j < m; j++) a[j][i] *= g;
		}
		else for (j = i; j < m; j++) a[j][i] = 0.0;
		++a[i][i];
	}

	for (k = n - 1; k >= 0; k--) { /* Diagonalization of the bidiagonal form. */
		for (its = 1; its <= 30; its++) {
			flag = 1;
			for (l = k; l >= 0; l--) { /* Test for splitting. */
				nm = l - 1; /* Note that rv1[1] is always zero. */
				if ((double)(fabs(rv1[l]) + anorm) == anorm) {
					flag = 0;
					break;
				}
				if ((double)(fabs(w[nm]) + anorm) == anorm) break;
			}
			if (flag) {
				c = 0.0; /* Cancellation of rv1[l], if l > 1. */
				s = 1.0;
				for (i = l; i <= k; i++) {
					f = s*rv1[i];
					rv1[i] = c*rv1[i];
					if ((double)(fabs(f) + anorm) == anorm) break;
					g = w[i];
					h = pythag(f, g);
					w[i] = h;
					h = 1.0 / h;
					c = g*h;
					s = -f*h;
					for (j = 0; j < m; j++) {
						y = a[j][nm];
						z = a[j][i];
						a[j][nm] = y*c + z*s;
						a[j][i] = z*c - y*s;
					}
				}
			}
			z = w[k];
			if (l == k) { /* Convergence. */
				if (z < 0.0) { /* Singular value is made nonnegative. */
					w[k] = -z;
					for (j = 0; j < n; j++) v[j][k] = -v[j][k];
				}
				break;
			}
			if (its == 30)
			{
				printf("no convergence in 30 svdcmp iterations\n");
				return;
			}
			x = w[l]; /* Shift from bottom 2-by-2 minor. */
			nm = k - 1;
			y = w[nm];
			g = rv1[nm];
			h = rv1[k];
			f = ((y - z)*(y + z) + (g - h)*(g + h)) / (2.0*h*y);
			g = pythag(f, 1.0);
			f = ((x - z)*(x + z) + h*((y / (f + SIGN(g, f))) - h)) / x;
			c = s = 1.0; /* Next QR transformation: */
			for (j = l; j <= nm; j++) {
				i = j + 1;
				g = rv1[i];
				y = w[i];
				h = s*g;
				g = c*g;
				z = pythag(f, h);
				rv1[j] = z;
				c = f / z;
				s = h / z;
				f = x*c + g*s;
				g = g*c - x*s;
				h = y*s;
				y *= c;
				for (jj = 0; jj < n; jj++) {
					x = v[jj][j];
					z = v[jj][i];
					v[jj][j] = x*c + z*s;
v[jj][i] = z*c - x*s;
				}
				z = pythag(f, h);
				w[j] = z; /* Rotation can be arbitrary if z = 0. */
				if (z) {
					z = 1.0 / z;
					c = f*z;
					s = h*z;
				}
				f = c*g + s*y;
				x = c*y - s*g;
				for (jj = 0; jj < m; jj++) {
					y = a[jj][j];
					z = a[jj][i];
					a[jj][j] = y*c + z*s;
					a[jj][i] = z*c - y*s;
				}
			}
			rv1[l] = 0.0;
			rv1[k] = f;
			w[k] = x;
		}
	}
}
//------------- Ax=b ----------------
/* based on LU decomposition
 * mode !=0, Ax=b  ,output x replace b;
 * mode =0 , b=A^-1,
 *
 */
int Ax_b1(matxx* mat, matxx* b,int mode)
{
	int i, imax, j, k, n, ip, ii = 0;
	double big, dum, sum, temp;
	double vv[10];
	double** a;
	int indx[10];

	if (mat->column != mat->row)
		return -1;
	matxx_transpose(mat, mat);//we defined a transposed matrix. abnormal !!!

	//inverse of mat
	if (mode == 0);
	{
		if (b->column != b->row || b->column != mat->column)
			return -1;
		for (i = 0; i < b->row; i++)
		{
			for (j = 0; j < b->row; j++)
			{
				if (i==j)
				(*(b->point + j))[i] = 1;
				else
				(*(b->point + j))[i] = 0;
			}
		}
	}
	a = mat->point;
	n = mat->column;

	for (i = 0; i < n; i++) {
		//Loop over rows to get the implicit scaling information.
		big = 1E-10;
		for (j = 0; j < n; j++)
			if ((temp = fabs(a[i][j])) > big) big = temp;
		if (big <= 1E-10)
		{
			return -1;
		}
		//No nonzero largest element.
		vv[i] = 1.0 / big; //Save the scaling.
	}
	for (j = 0; j < n; j++) {
		for (i = 0; i < j; i++) {
			sum = a[i][j];
			for (k = 0; k < i; k++) sum -= a[i][k] * a[k][j];
			a[i][j] = sum;
		}
		big = 0.0;
		for (i = j; i < n; i++) {
			sum = a[i][j];
			for (k = 0; k < j; k++)
				sum -= a[i][k] * a[k][j];
			a[i][j] = sum;
			if ((dum = vv[i] * fabs(sum)) >= big) {
				big = dum;
				imax = i;
			}
		}
		if (j != imax) {
			for (k = 0; k < n; k++) {
				dum = a[imax][k];
				a[imax][k] = a[j][k];
				a[j][k] = dum;
			}
			vv[imax] = vv[j];
		}
		indx[j] = imax;
		if (j != n - 1) {
			dum = (1.0 / a[j][j]);
			for (i = j + 1; i < n; i++) a[i][j] *= dum;
		}
	}
	//LUx=b
	for (k = 0; k < b->column; k++)
	{
		for (i = 0; i < n; i++) {
			ip = indx[i];
			sum = (b->point)[k][ip];
			(b->point)[k][ip] = (b->point)[k][i];
			if (i != 0)
			{
				for (j = ii; j <= i - 1; j++)
					sum -= a[i][j] * (b->point)[k][j];
				//printf("\nii=%d  i=%d\n", ii,i);
			}
			//else if (sum) ii = i;
			(b->point)[k][i] = sum;
		}
		for (i = n - 1; i >= 0; i--)
		{
			sum = (b->point)[k][i];
			if (i != n - 1)
			{
				for (j = i + 1; j < n; j++)
					sum -= a[i][j] * (b->point)[k][j];
			}
			(b->point)[k][i] = sum / a[i][i];
		}
	}
	return 0;
}

//
//for (k = 0; k < b->column; k++)
//{
//	double* bb;
//	bb = (double*)(*(b->point + k));
//	if (0.001)
//		printf("\nNULL\n");
//	//LUx=b
//	for (i = 0; i < n; i++) {
//		ip = indx[i];
//		sum = bb[ip];
//		bb[ip] = bb[i];
//		if (i != 0)
//		{
//			for (j = ii; j <= i - 1; j++)
//				sum -= a[i][j] * bb[j];
//			//printf("\nii=%d  i=%d\n", ii,i);
//		}
//		//else if (sum) ii = i;
//		bb[i] = sum;
//	}
//	for (i = n - 1; i >= 0; i--)
//	{
//		sum = bb[i];
//		if (i != n - 1)
//		{
//			for (j = i + 1; j < n; j++)
//				sum -= a[i][j] * bb[j];
//		}
//		bb[i] = sum / a[i][i];
//	}
//}

int mainsvd()
{
	//------------- SVD --------------------
	int i;// , j;
	//int m = 4; int n = 3;

	matxx X, W, V, U, Xt,b;
	double data[100] = { 0,3,4,5,0,4,7,5,6,0,51,15,8,5,6,0,51,15,7,8,5,6,0,51,15,7,1,2,4,3 ,7,6
		,8,5,6,0,51,15,7,1,2,4,3 ,7,1,2,4,3 ,7,1,2,8,5,6,0,51,15,7,1,2,4,3 ,7,4,3 ,5,68,5,6,0,51,15,7,1,2,4,3 ,7,6
		,0,51,15,7,6,5,6,0,51,15,7,1,2,4,3 ,1,2,4,3 ,1,5};
	double I[9] = { 1,0,0,0,1,0,0,0,1 };
	matxx_malloc(&X, 10, 10);
	matxx_malloc(&b, 10, 10);
	matxx_malloc(&Xt, 10, 10);
	//matxx_malloc(&U, 4, 3);
	//matxx_malloc(&V, 3, 3);
	//matxx_malloc(&W, 3, 1);
	matxx_assign(&X, data, 100);
	//matxx_assign(&b, I, 9);
	matxx_transpose(&X, &Xt);
	matxx_printf(&X);
	//mprintf(X.point, 3, 4);
	////matxx_printf(&Xt);
	//svdcmp(&Xt, &W, &V);
	//matxx_printf(&Xt);
	//matxx_printf(&W);
	//matxx_printf(&V);
	//mprintf(Xt.point, 4, 3);
	//------------- LU --------------
	double a[3][3] = { 2,3,4,5,3,4,5,6,4 };
//	int indx1[4];
//	double col[4];
	double y[3][3];
	//printf("\n\n");
	for (i=1;i<=10000;i++)
	Ax_b(&X, &b,0);

	matxx_printf(&b);
	

	return 0;

}

int mainsvd1()
{
	//------------- SVD --------------------
//	int i;// , j;
	int m = 4; int n = 3;

	matxx X, W, V, U, Xt;
	double data[12] = { 0,3,4,5,0,4,5,6,0,5,6,7 };
	matxx_malloc(&X, 4, 3);
	matxx_malloc(&Xt, 3, 4);
	matxx_malloc(&U, 4, 3);
	matxx_malloc(&V, 3, 3);
	matxx_malloc(&W, 3, 1);
	matxx_assign(&X, data, 12);
	matxx_transpose(&X, &Xt);
	//matxx_printf(&X);
	//mprintf(X.point, 3, 3);
	//matxx_printf(&Xt);
	svdcmp(&Xt, &W, &V);
	//matxx_printf(&Xt);
	//matxx_printf(&W);
	//matxx_printf(&V);
	//mprintf(Xt.point, 4, 3);


	return 0;

}











