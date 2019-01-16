/*
* matrix.h
*
*  Created on: Mar 15, 2018
*      Author: root
*/

#ifndef ROBOT_CONTROL_MATH_INCLUDE_MATRIX_H_
#define ROBOT_CONTROL_MATH_INCLUDE_MATRIX_H_

#include "type_def.h"
#include "robot_ctl_utility.h"

#define SPMATXX_MAX_NUM 	16
//#define min(x,y) (x<y?x:y)
//#define max(x,y) (x<y?y:x)
//-----------------------------------------------------------

typedef struct {
	double** 	point; 			//matrix value
	Uint8 		row;
	Uint8 		column;
}matxx;
//-----------------------------------------------------------
typedef struct TRI
{
	Uint8 row;
	Uint8 column;
	double value; 				//non zero value
}tri;
//-----------------------------------------------------------
typedef struct SPMATXX
{
	Uint8 row;
	Uint8 column;
	Uint16 non_zero; 			//non zero number
	tri data[SPMATXX_MAX_NUM];
}SPMatxx;

//-----------------------------------------------------------

void matxx_malloc(matxx* m, Uint8 row, Uint8 column);

void matxx_assign(matxx* m, double* init_m, Uint8 init_size);

double* matxx_get_column(matxx* m, Uint8 column);

void matxx_create(matxx* m, Uint8 row, Uint8 column, double* init_m, Uint8 init_size);

void matxx_delete(matxx* m);

void matxx_get_element(matxx* m, Uint8 row, Uint8 column, double* value);

void matxx_set_element(matxx* m, Uint8 row, Uint8 column, double* value);

void matxx_printf(matxx* m);

// extended definition
#define VECTOR3(m) matxx_malloc((matxx*) m, 3,1);

#define VECTOR6(m) matxx_malloc((matxx*) m, 6,1);

#define VECTOR3I(m, data) matxx_create((matxx*) m, 3, 1, (double*) data, 3);

#define SQUARE3(m) matxx_malloc((matxx*) m,3,3);

#define SQUARE6(m) matxx_malloc((matxx*) m, 6, 6);

#define SQUARE3I(m, data) matxx_create((matxx*) m, 3, 3, (double*) data, 9);

//a vector by a constant
//x := k*x
void matxx_k_mult(double k, matxx* x);

//copy matrix x to matrix y
void matxx_copy(matxx* x, matxx* y);

//constant times a matrix plus a matrix
// y := k*x + y
void matxx_k_mac(double k, matxx* x, matxx* y);

// vector dot product
double matxx_dot(matxx* x, matxx* y);

//Euclidean norm
double matxx_EuclideanNorm2(matxx* x);

//cross product: only for 3d vector
// z := x X y
void matxx_cross(matxx* x, matxx* y, matxx* z);

// Y = X^T transpose
void matxx_transpose(matxx* x, matxx* y);

// z = x*y
// matrix multiplication ,just for 3*3
void matxx_multiply(matxx* x, matxx* y, matxx*z);
//--------------------------------------------------------------------------
//performs one of the matrix-vector operations
//y' := alpha*x'*A + beta*y'
//where alpha and beta are scalars, x and y are vectors and A
void matxx_mat_vec_opt(matxx* x, matxx* A, double alpha, double beta, matxx* y);

//-----------------------------------------------------------------
// Transformations

// Convert axis-angle rotation to rotation matrix
void matxx_axisAngl2Rotm(double angle, matxx* axis_angle, matxx* rot_m);

// Convert axis-angle rotation to rotation matrix's transposition
void matxx_axisAngl2Rotm_t(double angle, matxx* axis, matxx* rot_m);

//  Convert axis-angle to rotation matrix and theta partial differential
void matxx_axisAngle2protm(double angle, double w, matxx* axis, matxx* protm);

void matxx_axisAngle2protm_t(double angle, double w, matxx* axis, matxx* protm);

//  inv of diag matrix.
void diag_inv(matxx* x, matxx* y);

//Uint16 matxx_inv(matxx* x, matxx* y);

void matxx_reshape(matxx* A, matxx* A1, matxx* A2, matxx* A3);

double determinant(double matrix[10][10], int k);

int16 Ax_b(matxx* mat, matxx* b, Uint8 mode);

void xyz2r(double* xyz, matxx* R, Uint8 mode);

void r2xyz(matxx* R, double* xyz, Uint8 mode);

int16 zyz2xyz(double* zyz, double* xyz);

int16 r2AxisAngle(matxx* R, matxx* axis, double* angle);

void axisvel2dxyz(matxx* axisvel, double* xyz, double* dxyz, Uint8 mode);

void dh2t(double a, double alpha, double d, double theta, matxx* T, Uint8 mode);

void t2rd(matxx* A, matxx* R, matxx* d);

void rd2t(matxx* R, matxx* d, matxx* A);

void pb2pa(matxx* T_A_B, matxx* pb, matxx* pa);

void rotz(double var_deg, matxx* T);

void transz(double d_mm, matxx* T);

int16 zyz2r(double* zyz, matxx* R);

int16 r2zyz(matxx* R, double* zyz);

int16 zyz2xyz(double* zyz, double* xyz);

int16 xyz2zyz(double* xyz, double* zyz);

#endif /* ROBOT_CONTROL_MATH_INCLUDE_MATRIX_H_ */
