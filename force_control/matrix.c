/*
* matrix.c
*
*  Created on: Mar 15, 2018
*      Author: root
*/

#include "type_def.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "matrix.h"

//--------------------------------------------------------------------------
void matxx_get_element(matxx* m, Uint8 row, Uint8 column, double* value)
{
	*value = *(*(m->point + column) + row);
	return;
}
//--------------------------------------------------------------------------
void matxx_set_element(matxx* m, Uint8 row, Uint8 column, double* value)
{
	*(*(m->point + column) + row) = *value;
	return;
}
//--------------------------------------------------------------------------
void matxx_malloc(matxx* m, Uint8 row, Uint8 column)
{
	Uint8 col_id = 0;

	//default 1*1
	if (column < 1)
	{
		column = 1;
	}
	if (row < 1)
	{
		row = 1;
	}

	m->column = column;
	m->row = row;

	/* memory allocation */
	m->point = (double**)malloc(sizeof(double*)*m->column);

	for (col_id = 0; col_id < m->column; col_id++)
	{
		*(m->point + col_id) = (double*)calloc(1, sizeof(double)*m->row);
	}
	return;
}
//--------------------------------------------------------------------------
void matxx_assign(matxx* m, double* init_m, Uint8 init_size)
{
	Uint8 col_id = 0;
	Uint8 row_id = 0;
	Uint8 total = 0;

	// default matrix[1,1]
	if (init_size < 1)
	{
		init_size = 1;
	}
	if (init_size >(m->column * m->row))
	{
		init_size = m->column * m->row;
	}
	// assignment
	if (init_m == NULL)
	{
		// do not need initialization
		return;
	}

	while (total < init_size)
	{
		row_id = total - m->row * col_id;
		if (row_id >= m->row)
		{
			// go to next column
			col_id++;
			row_id = total - m->row * col_id;
		}
		matxx_set_element(m, row_id, col_id, &init_m[total]);
		total++;
	}
	return;
}
//--------------------------------------------------------------------------
//get one column data
double* matxx_get_column(matxx* m, Uint8 column)
{
	double* data = NULL;
	data = (double*)(*(m->point + column));
	return data;
}
//--------------------------------------------------------------------------
//malloc and assign
void matxx_create(matxx* m, Uint8 row, Uint8 column, double* init_m, Uint8 init_size)
{
	matxx_malloc(m, row, column);
	matxx_assign(m, init_m, init_size);
	return;
}
//--------------------------------------------------------------------------
void matxx_delete(matxx* m)
{
	Uint8 col_id = 0;

	// memory free
	for (col_id = 0; col_id < m->column; col_id++) {
		free(*(m->point + col_id));
	}
	free(m->point);
	return;
}
//--------------------------------------------------------------------------
void matxx_printf(matxx* m)
{
	Uint8 row_id;
	Uint8 col_id;
	double value = 0.0;

	if ((m->point == NULL) || (m->column <1) || (m->row < 1))
	{
		printf(" m is empty; \n");
		return;
	}

	printf("MATRIX := \n");
	for (row_id = 0; row_id < m->row; row_id++)
	{
		for (col_id = 0; col_id < m->column; col_id++)
		{
			matxx_get_element(m, row_id, col_id, &value);
			printf("%f  ", value);
		}
		printf("\n");
	}
	return;
}
//--------------------------------------------------------------------------
// a matrix multiply by a constant
// x := k*x

void matxx_k_mult(double k, matxx* x)
{
	Uint8 row_id, col_id;

	for (row_id = 0; row_id < x->row; row_id++)
	{
		for (col_id = 0; col_id < x->column; col_id++)
		{
			(*(x->point + col_id))[row_id] *= k;
		}
	}
	return;
}
/*
void matxx_k_mult(double k,matxx* x)
{
Uint8 row_id;
double* data_x = NULL;

//get 0 column
data_x = matxx_get_column(x,0);
for(row_id = 0; row_id < x->row; row_id++)
{
*(data_x + row_id) *= k;
}
return;
}
*/
//--------------------------------------------------------------------------
// copy matrix x into matrix y
void matxx_copy(matxx* x, matxx* y)
{
	Uint8 row_id, col_id;
	Uint8 row_min, col_min;

	// x and y have the same address.
	if (x == y) {
		return;
	}

	col_min = min(x->column, y->column);
	row_min = min(x->row, y->row);

	for (row_id = 0; row_id < row_min; row_id++)
	{
		for (col_id = 0; col_id < col_min; col_id++)
		{
			(*(y->point + col_id))[row_id] = (*(x->point + col_id))[row_id];
		}
	}
	return;
}
/*
void matxx_copy(matxx* x,matxx* y)
{
Uint8 row_id;

double* data_x = NULL;
double* data_y = NULL;

// x and y have the same size.
if(x->row != y->row){
return;
}
//get the first column
data_x = matxx_get_column(x,0);
data_y = matxx_get_column(y,0);

for(row_id = 0; row_id < x->row; row_id++)
{
*(data_y + row_id) = *(data_x + row_id);
}
return;
}
*/
//--------------------------------------------------------------------------
// constant times a matrix plus a matrix
// y := k*x + y
void matxx_k_mac(double k, matxx* x, matxx* y)  // changed by hqi
{
	Uint8 row_id, col_id;
	Uint8 row_min, col_min;

	row_min = min(x->row, y->row);
	col_min = min(x->column, y->column);

	for (row_id = 0; row_id < row_min; row_id++)
	{
		for (col_id = 0; col_id < col_min; col_id++)
		{
			(*(y->point + col_id))[row_id] += k*(*(x->point + col_id))[row_id];
		}
	}
	return;
}
//--------------------------------------------------------------------------
// vector dot product

double matxx_dot(matxx* x, matxx* y)
{
	Uint8 row_id;
	double* data_x = NULL;
	double* data_y = NULL;
	double result = 0.0;

	// x and y have the same size.
	if (x->row != y->row) {
		return -1;
	}
	// point to  the first column
	data_x = matxx_get_column(x, 0);
	data_y = matxx_get_column(y, 0);

	for (row_id = 0; row_id < x->row; row_id++)
	{
		result += ((*(data_x + row_id)) * (*(data_y + row_id)));
	}

	return result;
}

//--------------------------------------------------------------------------
//vector Euclidean norm2
//sqrt(x1^2+x2^2 +...)
double matxx_EuclideanNorm2(matxx* x)
{
	Uint8 row_id;

	double* data_x = NULL;
	double result = 0.0;

	// point to  the first column
	data_x = matxx_get_column(x, 0);

	for (row_id = 0; row_id < x->row; row_id++)
	{
		result += ((*(data_x + row_id)) * (*(data_x + row_id)));
	}
	result = sqrt(result);
	return result;
}

//--------------------------------------------------------------------------
// vector cross product 3*3
// z := x X y
void matxx_cross(matxx* x, matxx* y, matxx* z)
{
	double result[3] = { 0.0,0.0,0.0 };
	double* data_x = NULL;
	double* data_y = NULL;

	// x and y have the same size.
	if (x->row != y->row)
	{
		return;
	}
	if (x->row != 3)
	{
		return;
	}
	// point to  the first column
	data_x = matxx_get_column(x, 0);
	data_y = matxx_get_column(y, 0);
	/*
	* x = [x0, x1, x2]
	* y = [y0, y1, y2]
	* x X y = [x1*y2 - y1*x2, -(x0*y2 - y0*x2), x0*y1 - y0*x1]
	* */
	result[0] = data_x[1] * data_y[2];
	result[0] -= (data_x[2] * data_y[1]);

	result[1] = data_x[2] * data_y[0];
	result[1] -= (data_x[0] * data_y[2]);

	result[2] = data_x[0] * data_y[1];
	result[2] -= (data_x[1] * data_y[0]);

	matxx_assign(z, result, 3);

	return;
}
//--------------------------------------------------------------------------
// Y = X^T transpose also for x=y
//--------------------------------------------------------------------------
void matxx_transpose(matxx* x, matxx* y)
{
	Uint8 row_id, col_id;
	double data[8][8];

	if (x->column != y->row || x->row != y->column)
	{
		return;
	}
	if (x == y)
	{
		for (row_id = 0; row_id < y->row; row_id++)
		{
			for (col_id = 0; col_id < y->column; col_id++)
			{
				data[row_id][col_id] = (*(x->point + row_id))[col_id];
			}
		}
		for (row_id = 0; row_id < y->row; row_id++)
		{
			for (col_id = 0; col_id < y->column; col_id++)
			{
				(*(x->point + row_id))[col_id] = data[col_id][row_id];
			}
		}
		return;
	}
	for (row_id = 0; row_id < y->row; row_id++)
	{
		for (col_id = 0; col_id < y->column; col_id++)
		{
			(*(y->point + col_id))[row_id] = (*(x->point + row_id))[col_id];
		}
	}
	return;

}
/*
void matxx_transpose(matxx* x,matxx* y)
{
Uint8 j;
double* data_x 		= NULL;
double* data_y_1 	= NULL;
double* data_y_2 	= NULL;
double* data_y_3 	= NULL;

// x and y have the same size.
if(x->row != y->row)
{
return;
}
if(x->row != 3)
{
return;
}
// point to  the first column,second column,third column
data_y_1 = matxx_get_column(y,0);
data_y_2 = matxx_get_column(y,1);
data_y_3 = matxx_get_column(y,2);


for(j = 0;j<3;++j)
{
// point to x the j column
data_x = matxx_get_column(x,j);

data_y_1[j] = data_x[0];
data_y_2[j] = data_x[1];
data_y_3[j] = data_x[2];
}
return;

}
*/
//--------------------------------------------------------------------------
// z = x*y,z==x||z==y is ok,8*8 limit
// matrix multiplication
//--------------------------------------------------------------------------
void matxx_multiply(matxx* x, matxx* y, matxx*z)
{
	Uint8 row_id, col_id, i;
	double data[8][8];

	if (x->column != y->row || z->row != x->row || z->column != y->column)
	{
		return;
	}
	//if x==z or y==z
	if (z == x || z == y)
	{
		for (row_id = 0; row_id < z->row; row_id++)
		{
			for (col_id = 0; col_id < z->column; col_id++)
			{
				data[row_id][col_id] = 0;
				for (i = 0; i < x->column; i++)
				{
					data[row_id][col_id] += (*(x->point + i))[row_id] * (*(y->point + col_id))[i];
				}
			}
		}
		for (row_id = 0; row_id < z->row; row_id++)
		{
			for (col_id = 0; col_id < z->column; col_id++)
			{

				(*(z->point + col_id))[row_id] = data[row_id][col_id];

			}
		}
		return;

	}

	for (row_id = 0; row_id < z->row; row_id++)
	{
		for (col_id = 0; col_id < z->column; col_id++)
		{
			(*(z->point + col_id))[row_id] = 0;
			for (i = 0; i < x->column; i++)
			{
				(*(z->point + col_id))[row_id] += (*(x->point + i))[row_id] * (*(y->point + col_id))[i];
			}
		}
	}

	return;
}
/*
void matxx_multiply(matxx* x,matxx* y,matxx*z)
{
Uint8 row_id,col_id;

double* data_x[3] 	= {NULL};
double* data_y[3] 	= {NULL};

// x and y have the same size.
if(x->column != y->row)
{
return;
}
data_x[0] = matxx_get_column(x,0);
data_x[1] = matxx_get_column(x,1);
data_x[2] = matxx_get_column(x,2);

data_y[0] = matxx_get_column(y,0);
data_y[1] = matxx_get_column(y,1);
data_y[2] = matxx_get_column(y,2);

for(col_id = 0; col_id < z->column; col_id++)
{
for(row_id = 0; row_id < z->row; row_id++)
{
(*(z->point+col_id))[row_id] = data_x[0][row_id]*data_y[col_id][0] + \
data_x[1][row_id]*data_y[col_id][1] + \
data_x[2][row_id]*data_y[col_id][2];
}
}

return;
}
*/
//--------------------------------------------------------------------------
//  performs one of the matrix-vector operations
//  y' := alpha*x'*A + beta*y'
//  where alpha and beta are scalars, x and y are vectors and A is matrix

void matxx_mat_vec_opt(matxx* x, matxx* A, double alpha, double beta, matxx* y)
{
	double* vector_x = NULL;
	double* vector_y = NULL;
	double* column_A = NULL;

	double result[10];
	Uint8 row_id = 0;
	Uint8 col_id = 0;

	// check the conditions of the matrix-vector operations
	if (A->row != x->row) {
		return;
	}
	if (A->column != y->row) {
		return;
	}
	if (x->column != y->column) {
		return;
	}
	// get the data of MATXX
	vector_x = matxx_get_column(x, 0);
	vector_y = matxx_get_column(y, 0);

	for (row_id = 0; row_id < y->row; row_id++)
	{
		result[row_id] = 0;
		column_A = (double*)(*(A->point + row_id));

		//x is row like [a b c], x' = column
		for (col_id = 0; col_id < x->row; col_id++)
		{
			result[row_id] += ((*(vector_x + col_id)) * (*(column_A + col_id)));
		}

		*(vector_y + row_id) *= beta;
		*(vector_y + row_id) += (result[row_id] * alpha);
	}
	return;
}

//--------------------------------------------------------------------------
// Convert axis-angle rotation to rotation matrix

void matxx_axisAngl2Rotm(double angle, matxx* axis, matxx* rot_m)
{
	/*
	% For a single axis-angle vector [ax ay az theta] the output rotation
	% matrix R can be computed as follows:
	% R =  [t*x*x + c	  t*x*y - z*s	   t*x*z + y*s
	%       t*x*y + z*s	  t*y*y + c	       t*y*z - x*s
	%       t*x*z - y*s	  t*y*z + x*s	   t*z*z + c]
	% where,
	% c = cos(theta)
	% s = sin(theta)
	% t = 1 - c
	% x = normalized axis ax coordinate
	% y = normalized axis ay coordinate
	% z = normalized axis az coordinate
	*/
	double c, s, t;
	double x, y, z;
	double data = 0.0;
	c = cos(angle);
	s = sin(angle);
	t = 1.0 - c;
	matxx_get_element(axis, 0, 0, &x);
	matxx_get_element(axis, 1, 0, &y);
	matxx_get_element(axis, 2, 0, &z);


	/* 1st Column */
	data = t*x*x + c;
	matxx_set_element(rot_m, 0, 0, &data);
	data = t*x*y + z*s;
	matxx_set_element(rot_m, 1, 0, &data);
	data = t*x*z - y*s;
	matxx_set_element(rot_m, 2, 0, &data);

	/* 2nd Column */
	data = t*x*y - z*s;
	matxx_set_element(rot_m, 0, 1, &data);
	data = t*y*y + c;
	matxx_set_element(rot_m, 1, 1, &data);
	data = t*y*z + x*s;
	matxx_set_element(rot_m, 2, 1, &data);

	/* 3rd Column */
	data = t*x*z + y*s;
	matxx_set_element(rot_m, 0, 2, &data);
	data = t*y*z - x*s;
	matxx_set_element(rot_m, 1, 2, &data);
	data = t*z*z + c;
	matxx_set_element(rot_m, 2, 2, &data);

	return;
}
//--------------------------------------------------------------------------
// Convert axis-angle rotation to rotation matrix's transposition
void matxx_axisAngl2Rotm_t(double angle, matxx* axis, matxx* rot_m)
{
	/*
	% For a single axis-angle vector [ax ay az theta] the output rotation
	% matrix R can be computed as follows:
	% R =  [t*x*x + c	  t*x*y - z*s	   t*x*z + y*s
	%       t*x*y + z*s	  t*y*y + c	       t*y*z - x*s
	%       t*x*z - y*s	  t*y*z + x*s	   t*z*z + c]
	% where,
	% c = cos(theta)
	% s = sin(theta)
	% t = 1 - c
	% x = normalized axis ax coordinate
	% y = normalized axis ay coordinate
	% z = normalized axis az coordinate
	*/
	double c, s, t;
	double x, y, z;
	double data = 0.0;
	c = cos(angle);
	s = sin(angle);
	t = 1.0 - c;
	matxx_get_element(axis, 0, 0, &x);
	matxx_get_element(axis, 1, 0, &y);
	matxx_get_element(axis, 2, 0, &z);


	/* 1st Column */
	data = t*x*x + c;
	matxx_set_element(rot_m, 0, 0, &data);
	data = t*x*y + z*s;
	matxx_set_element(rot_m, 0, 1, &data);
	data = t*x*z - y*s;
	matxx_set_element(rot_m, 0, 2, &data);

	/* 2nd Column */
	data = t*x*y - z*s;
	matxx_set_element(rot_m, 1, 0, &data);
	data = t*y*y + c;
	matxx_set_element(rot_m, 1, 1, &data);
	data = t*y*z + x*s;
	matxx_set_element(rot_m, 1, 2, &data);

	/* 3rd Column */
	data = t*x*z + y*s;
	matxx_set_element(rot_m, 2, 0, &data);
	data = t*y*z - x*s;
	matxx_set_element(rot_m, 2, 1, &data);
	data = t*z*z + c;
	matxx_set_element(rot_m, 2, 2, &data);

	return;
}


//--------------------------------------------------------------------------
//  Convert axis-angle to rotation matrix and theta partial differential

void matxx_axisAngle2protm(double angle, double w, matxx* axis, matxx* protm)
{
	/*
	% For a single axis-angle vector [ax ay az theta] the output rotation
	% matrix R can be computed as follows:
	% R =  [t*x*x + c	  t*x*y - z*s	   t*x*z + y*s
	%       t*x*y + z*s	  t*y*y + c	       t*y*z - x*s
	%       t*x*z - y*s	  t*y*z + x*s	   t*z*z + c]

	% dR =w* [s*x*x - s     s*x*y - z*c	   s*x*z + y*c
	%       s*x*y + z*c	  s*y*y - s	       s*y*z - x*c
	%       s*x*z - y*c	  s*y*z + x*c	   s*z*z - s]
	% where,
	% c = cos(theta)
	% s = sin(theta)
	% t = 1 - c
	% x = normalized axis ax coordinate
	% y = normalized axis ay coordinate
	% z = normalized axis az coordinate
	*/
	double c, s;
	double x, y, z;
	double data = 0.0;
	c = cos(angle);
	s = sin(angle);
	matxx_get_element(axis, 0, 0, &x);
	matxx_get_element(axis, 1, 0, &y);
	matxx_get_element(axis, 2, 0, &z);


	/* 1st Column */
	data = w*(s*x*x - s);
	matxx_set_element(protm, 0, 0, &data);
	data = w*(s*x*y + z*c);
	matxx_set_element(protm, 1, 0, &data);
	data = w*(s*x*z - y*c);
	matxx_set_element(protm, 2, 0, &data);

	/* 2nd Column */
	data = w*(s*x*y - z*c);
	matxx_set_element(protm, 0, 1, &data);
	data = w*(s*y*y - s);
	matxx_set_element(protm, 1, 1, &data);
	data = w*(s*y*z + x*c);
	matxx_set_element(protm, 2, 1, &data);

	/* 3rd Column */
	data = w*(s*x*z + y*c);
	matxx_set_element(protm, 0, 2, &data);
	data = w*(s*y*z - x*c);
	matxx_set_element(protm, 1, 2, &data);
	data = w*(s*z*z - s);
	matxx_set_element(protm, 2, 2, &data);

	return;
}
void matxx_axisAngle2protm_t(double angle, double w, matxx* axis, matxx* protm)
{
	/*
	% For a single axis-angle vector [ax ay az theta] the output rotation
	% matrix R can be computed as follows:
	% R =  [t*x*x + c	  t*x*y - z*s	   t*x*z + y*s
	%       t*x*y + z*s	  t*y*y + c	       t*y*z - x*s
	%       t*x*z - y*s	  t*y*z + x*s	   t*z*z + c]

	% dR = [s*x*x - s	  s*x*y - z*c	   s*x*z + y*c
	%       s*x*y + z*c	  s*y*y - s	       s*y*z - x*c
	%       s*x*z - y*c	  s*y*z + x*c	   s*z*z - s]
	% where,
	% c = cos(theta)
	% s = sin(theta)
	% t = 1 - c
	% x = normalized axis ax coordinate
	% y = normalized axis ay coordinate
	% z = normalized axis az coordinate
	*/
	double c, s;
	double x, y, z;
	double data = 0.0;
	c = cos(angle);
	s = sin(angle);
	matxx_get_element(axis, 0, 0, &x);
	matxx_get_element(axis, 1, 0, &y);
	matxx_get_element(axis, 2, 0, &z);
	/*
	* dR is stored as dR',
	* for dR*v is replaces with v'*dR' */

	/* 1st Column */
	data = w*(s*x*x - s);
	matxx_set_element(protm, 0, 0, &data);
	data = w*(s*x*y - z*c);
	matxx_set_element(protm, 1, 0, &data);
	data = w*(s*x*z + y*c);
	matxx_set_element(protm, 2, 0, &data);

	/* 2nd Column */
	data = w*(s*x*y + z*c);
	matxx_set_element(protm, 0, 1, &data);
	data = w*(s*y*y - s);
	matxx_set_element(protm, 1, 1, &data);
	data = w*(s*y*z - x*c);
	matxx_set_element(protm, 2, 1, &data);

	/* 3rd Column */
	data = w*(s*x*z - y*c);
	matxx_set_element(protm, 0, 2, &data);
	data = w*(s*y*z + x*c);
	matxx_set_element(protm, 1, 2, &data);
	data = w*(s*z*z - s);
	matxx_set_element(protm, 2, 2, &data);

	return;
}

//inv of matrix diag.
void diag_inv(matxx* x, matxx* y)
{
	Uint8    i = 0;
	(*(y->point))[1] = 0.0;
	(*(y->point + 1))[0] = 0.0;
	for (i = 0; i<x->column; i++)
	{
		(*(y->point + i))[i] = 1.0 / (*(x->point + i))[i];
	}
	return;
};

//----------- determinant -------------
//
//input:  the matrix.
//output:
//return: the determinant of the matrix.
//fuction: get the determinant of a matrix.
//
//-------------------------------------
double determinant(double matrix[10][10], int k)
{
	int m, n, i, j, c;
	double b[10][10];
	double det = 0, s = 1;
	if (k == 1)
	{
		return (matrix[0][0]);
	}
	else
	{
		for (c = 0; c<k; c++)
		{
			m = 0;
			n = 0;
			for (i = 0; i < k; i++)
			{
				for (j = 0; j < k; j++)
				{
					b[i][j] = 0;
					if (i != 0 && j != c)
					{
						b[m][n] = matrix[i][j];
						if (n < (k - 2))
							n++;
						else
						{
							n = 0;
							m++;
						}
					}
				}
			}
			det = det + s*(matrix[0][c] * determinant(b, k - 1));
			s = -1 * s;
		}
	}
	return det;
}

//----------- matxx_inv ---------
//
//input:  the matrix.
//output:  the inverse of the matrix
//function: to find cofactor of a matrix.
// A*A^(-1)=1
// A^(-1)=A*/|A|
// A*= {{A11 ,A12, A13},{A21 ,A22, A23},{A31 ,A32, A33}}
// |A|=DET(A)
// A11=det({{a22,a23},{a32,a33}})
//-------------------------------------
int16 matxx_inv(matxx* x, matxx* y)
{
	int p, q, m, n, i, j, k;
	double b[10][10];
	double num[10][10];
	double  fac[10][10];
	double   d;

	k = x->column;

	if (x->column != x->row || y->column != y->row || x->column != y->column)
	{
		return -1;
	}

	for (i = 0; i < x->row; i++)
	{
		for (j = 0; j < x->column; j++)
		{
			num[i][j] = (*(x->point + j))[i];
		}
	}

	d = determinant(num, k);

	if (fabs(d) < 1.0E-10)
	{
		return -1;
	}

	for (q = 0; q < k; q++)
	{
		for (p = 0; p < k; p++)
		{
			m = 0;
			n = 0;
			for (i = 0; i < k; i++)
			{
				for (j = 0; j < k; j++)
				{
					if (i != q&&j != p)
					{
						b[m][n] = num[i][j];
						if (n<(k - 2))
							n++;
						else
						{
							n = 0;
							m++;
						}
					}

				}
			}
			fac[q][p] = pow(-1, p + q)*determinant(b, k - 1);
		}
	}

	for (i = 0; i < k; i++)
	{
		for (j = 0; j < k; j++)
		{
			(*(y->point + i))[j] = fac[j][i] / d;
		}
	}
	return 0;
}
//--------------------------------------------------------------------
// A=[A1;A2;A3]
void matxx_reshape(matxx* A, matxx* A1, matxx* A2, matxx* A3)
{
	int16 i, j, k, g;

	if (A1 == NULL || A == NULL)
	{
		return;
	}
	k = min(A1->row, A->row);
	for (i = 0; i< k; i++)
	{
		for (j = 0; j<min(A1->column, A->column); j++)
		{
			A->point[j][i] = A1->point[j][i];
		}
	}
	g = min(A->row - A1->row, A2->row);
	if (A2 == NULL || g <= 0)
	{
		return;
	}

	for (i = k; i<k + g; i++)
	{
		for (j = 0; j<min(A2->column, A->column); j++)
		{
			A->point[j][i] = A2->point[j][i - k];
		}
	}
	k = k + g;
	g = min(A->row - A1->row - A2->row, A3->row);
	if (A3 == NULL || g <= 0)
	{
		return;
	}

	for (i = k; i<k + g; i++)
	{
		for (j = 0; j<min(A3->column, A->column); j++)
		{
			A->point[j][i] = A3->point[j][i - k];
		}
	}

	return;
}

//----------------------------- Ax=b -------------------------------
/* based on LU decomposition
* mode !=1, Ax=b  ,output x replace b;
* mode =1 , b=A^-1,
*
*/
int16 Ax_b(matxx* mat, matxx* b, Uint8 mode)
{
	int i, imax, j, k, n, ip, ii = 0;
	double big, dum, sum, temp;
	double vv[10];
	double** a;
	int indx[10];

	if (mat->column != mat->row)
		return -1;

	if (b->column != b->row || b->column != mat->column)
		return -1;
	for (i = 0; i < mat->row; i++)
	{
		for (j = 0; j < mat->column; j++)
		{
			if (mode == 1)//inverse of mat
			{
				if (i == j)
					(*(b->point + j))[i] = 1;
				else
					(*(b->point + j))[i] = 0;
			}
			if (i < j)//we defined a transposed matrix. abnormal !!!
			{
				sum = (*(mat->point + i))[j];
				(*(mat->point + i))[j] = (*(mat->point + j))[i];
				(*(mat->point + j))[i] = sum;
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
//----------------------------------------------------------------
/* function                :    xyz2r
* r(rot x),p(rot y),y(rot z) in deg/rad to rotation matrix
* input
* 						   :mode = 0 ,in deg
* 						   :mode = 1 ,in rad
*                          : yaw(rotx),pitch(roty),roll(rotz)=r,b,a
*
* output                   :rotation matrix
*
*/
void xyz2r(double* xyz, matxx* R, Uint8 mode)
{
	double sr, cr, sa, ca, sb, cb;
	double tmp[3];
	if (mode == 0)
	{
		tmp[0] = xyz[0] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
		tmp[1] = xyz[1] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
		tmp[2] = xyz[2] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	}
	else
	{
		tmp[0] = xyz[0];
		tmp[1] = xyz[1];
		tmp[2] = xyz[2];
	}

	sr = sin(tmp[0]);
	sb = sin(tmp[1]);
	sa = sin(tmp[2]);
	cr = cos(tmp[0]);
	cb = cos(tmp[1]);
	ca = cos(tmp[2]);

	(*(R->point + 0))[0] = ca*cb;
	(*(R->point + 0))[1] = sa*cb;
	(*(R->point + 0))[2] = -sb;

	(*(R->point + 1))[0] = ca*sb*sr - sa*cr;
	(*(R->point + 1))[1] = sa*sb*sr + ca*cr;
	(*(R->point + 1))[2] = cb*sr;

	(*(R->point + 2))[0] = ca*sb*cr + sa*sr;
	(*(R->point + 2))[1] = sa*sb*cr - ca*sr;
	(*(R->point + 2))[2] = cb*cr;
	return;
}
//--------------------------------------------------------------------------
// mode==0 deg;mode==1 rad
void r2xyz(matxx* R, double* xyz, Uint8 mode)
{
	xyz[1] = atan2(-*(*(R->point + 0) + 2), pow(pow(*(*(R->point + 0) + 0), 2) + pow(*(*(R->point + 0) + 1), 2), 0.5));

	if (fabs(xyz[1] - MOTION_MODULE_CONSTANT_PI / 2)<1E-10)
	{
		xyz[2] = 0;
		xyz[0] = atan2(*(*(R->point + 1) + 0), *(*(R->point + 1) + 1));
	}
	else if (fabs(xyz[1] + MOTION_MODULE_CONSTANT_PI / 2)<1E-10)
	{
		xyz[2] = 0;
		xyz[0] = -atan2(*(*(R->point + 1) + 0), *(*(R->point + 1) + 1));
	}
	else
	{
		xyz[2] = atan2(*(*(R->point + 0) + 1) / cos(xyz[1]), *(*(R->point + 0) + 0) / cos(xyz[1]));
		xyz[0] = atan2(*(*(R->point + 1) + 2) / cos(xyz[1]), *(*(R->point + 2) + 2) / cos(xyz[1]));
	}

	if (mode == 0)
	{
		xyz[0] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
		xyz[1] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
		xyz[2] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
	}
	return;
}
int16 zyz2r(double* zyz, matxx* R)
{
	double ca, cb, cr, sa, sb, sr;
	double a, b, r;
	a = zyz[0]* MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	b = zyz[1]* MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	r = zyz[2]* MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	ca = cos(a);
	sa = sin(a);
	cb = cos(b);
	sb = sin(b);
	cr = cos(r);
	sr = sin(r);
	if (R->column < 3 || R->row < 3)
	{
		return -1;
	}
	R->point[0][0] = ca*cb*cr - sa*sr;
	R->point[0][1] = sa*cb*cr + ca*sr;
	R->point[0][2] = -sb*cr;
	R->point[1][0] = -ca*cb*sr - sa*cr;
	R->point[1][1] = -sa*cb*sr + ca*cr;
	R->point[1][2] = sb*sr;
	R->point[2][0] = ca*sb;
	R->point[2][1] = sa*sb;
	R->point[2][2] = cb;

	return;
}
int16 zyz2xyz(double* zyz,double* xyz)
{
	double R[3][3];  // same as the matrix we define
	double ca, cb, cr, sa, sb, sr;
	double a, b, r;
	// zyz angle to rotation mateix
	a = zyz[0] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	b = zyz[1] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	r = zyz[2] * MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	ca = cos(a);
	sa = sin(a);
	cb = cos(b);
	sb = sin(b);
	cr = cos(r);
	sr = sin(r);

	R[0][0] = ca*cb*cr - sa*sr;
	R[0][1] = sa*cb*cr + ca*sr;
	R[0][2] = -sb*cr;
	R[1][0] = -ca*cb*sr - sa*cr;
	R[1][1] = -sa*cb*sr + ca*cr;
	R[1][2] = sb*sr;
	R[2][0] = ca*sb;
	R[2][1] = sa*sb;
	R[2][2] = cb;

	// rotation matrix to zyz
	//xyz[1] = atan2(-*(*(R->point + 0) + 2), pow(pow(*(*(R->point + 0) + 0), 2) + pow(*(*(R->point + 0) + 1), 2), 0.5));
	xyz[1] = atan2(-R[0][2], pow(pow(R[0][0], 2) + pow(R[0][1], 2), 0.5));
	if (fabs(xyz[1] - MOTION_MODULE_CONSTANT_PI / 2)<1E-10)
	{
		xyz[2] = 0;
		xyz[0] = atan2(R[1][0], R[1][1]);
	}
	else if (fabs(xyz[1] + MOTION_MODULE_CONSTANT_PI / 2)<1E-10)
	{
		xyz[2] = 0;
		xyz[0] = -atan2(R[1][0], R[1][1]);
	}
	else
	{
		xyz[2] = atan2(R[0][1] / cos(xyz[1]), R[0][0] / cos(xyz[1]));
		xyz[0] = atan2(R[1][2] / cos(xyz[1]), R[2][2] / cos(xyz[1]));
	}

	xyz[0] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
	xyz[1] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
	xyz[2] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;

	return 0;
}

int16 r2AxisAngle(matxx* R, matxx* axis, double* angle)
{
	double theta_tmp = 0;
	//calculate the axis angle
	*angle = acos(0.5*((*(R->point + 0))[0] + (*(R->point + 1))[1] + (*(R->point + 2))[2] - 1));

	//check if the angle is too small.which can not calculate the axis
	if (sin(*angle)< MOTION_MODULE_CONSTANT_MIN_POSITIVE)
	{
		//printf("the line move pose angle is too small\n");
		//can not calculate axis
		//return;
		return -1;
	}
	else
	{

		//common part 1/2/sin(angle)
		theta_tmp = 1.0 / 2.0 / sin(*angle);

		//r32-r23
		(*(axis->point + 0))[0] = theta_tmp*((*(R->point + 1))[2] - (*(R->point + 2))[1]);
		//r13-r31
		(*(axis->point + 0))[1] = theta_tmp*((*(R->point + 2))[0] - (*(R->point + 0))[2]);
		//r21 - r12
		(*(axis->point + 0))[2] = theta_tmp*((*(R->point + 0))[1] - (*(R->point + 1))[0]);
		return 0;
	}
}

void axisvel2dxyz(matxx* axisvel, double* xyz, double* dxyz, Uint8 mode)
{
	double w[3];
	// Wx  [ axis_x ]
	// Wy =[ axis_y ] * angular_vel = axisvel
	// Wz  [ axis_z ]
	w[0] = *(*(axisvel->point + 0) + 0);
	w[1] = *(*(axisvel->point + 0) + 1);
	w[2] = *(*(axisvel->point + 0) + 2);
	// d a /d t          [ 0    sr    cr    ]   [ Wx ]
	// d b /d t = 1/cb * [ 0    crcb  -srcb ] * [ Wy ]
	// d r /d t          [ cb   srsb  crsb  ]   [ Wz ]
	if (fabs(xyz[1] - MOTION_MODULE_CONSTANT_PI / 2) < MOTION_MODULE_CONSTANT_MIN_POSITIVE)
	{
		//this function has error,we need fixed it
		dxyz[1] = w[1] / cos(xyz[0]);
		dxyz[0] = -w[2] / sin(xyz[0]);
		dxyz[2] = dxyz[0] - w[0];

	}
	else
	{
		dxyz[2] = 1 / cos(xyz[1])*(sin(xyz[0])*w[1] + cos(xyz[0])* w[2]);
		dxyz[1] = 1 / cos(xyz[1])*(cos(xyz[0])*cos(xyz[1])*w[1] - sin(xyz[0])*cos(xyz[1])*w[2]);
		dxyz[0] = w[0] + sin(xyz[0])*tan(xyz[1])*w[1] + cos(xyz[0])*tan(xyz[1])*w[2];
	}

	if (mode == 0)
	{
		xyz[0] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
		xyz[1] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
		xyz[2] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
		dxyz[0] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
		dxyz[1] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
		dxyz[2] *= MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
	}
	return;
}



void dh2t(double a, double alpha, double d, double theta, matxx* T, Uint8 mode)
{
	double et, ap;
	if (mode == 0)
	{
		et = theta*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
		ap = alpha*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	}
	else
	{
		et = theta;
		ap = alpha;
	}
	T->point[0][0] = cos(et);
	T->point[0][1] = sin(et);
	T->point[0][2] = 0;
	T->point[0][3] = 0;
	T->point[1][0] = -sin(et)*cos(ap);
	T->point[1][1] = cos(et)*cos(ap);
	T->point[1][2] = sin(ap);
	T->point[1][3] = 0;
	T->point[2][0] = sin(et)*sin(ap);
	T->point[2][1] = -cos(et)*sin(ap);
	T->point[2][2] = cos(ap);
	T->point[2][3] = 0;
	T->point[3][0] = a*cos(et);
	T->point[3][1] = a*sin(et);
	T->point[3][2] = d;
	T->point[3][3] = 1;

	return;
}

void t2rd(matxx* A, matxx* R, matxx* d)
{
	Uint8 i, j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			R->point[i][j] = A->point[i][j];
		}
	}
	for (i = 0; i < 3; i++)
	{
		d->point[0][i] = A->point[3][i];
	}
	return;
}
/* P_{A}=R{A,B} * p_{B} + d_{A};
*
* T :4*4= [R d;0 1]
*/
void rd2t(matxx* R, matxx* d, matxx* A)
{
	Uint8 i, j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			A->point[i][j] = R->point[i][j];
		}
		A->point[3][i] = d->point[0][i];
	}
	A->point[0][3] = 0;
	A->point[1][3] = 0;
	A->point[2][3] = 0;
	A->point[3][3] = 1;
	return;
}
/* function    : Pb2Pa
*               A'P=A'R_B * B'p + A'd
* input
*       A'T_B := [R d;0 1]
*/
void pb2pa(matxx* T_A_B, matxx* pb, matxx* pa)
{
	int i, j;
	for (i = 0; i < T_A_B->row - 1; i++)  //row-1=3
	{
		pa->point[0][i] = T_A_B->point[3][i];
		for (j = 0; j < T_A_B->column - 1; j++)  //column=4
		{
			pa->point[0][i] += T_A_B->point[j][i] * pb->point[0][j];
		}
	}
	return;
}



void rotz(double var_deg, matxx* T)
{
	double var_rad;
	var_rad = var_deg*MOTION_MODULE_UNIT_TRANSFORM_DEGREE_2_RAD;
	T->point[0][0] = cos(var_rad);
	T->point[0][1] = sin(var_rad);
	T->point[0][2] = 0;
	T->point[0][3] = 0;

	T->point[1][0] = -sin(var_rad);
	T->point[1][1] = cos(var_rad);
	T->point[1][2] = 0;
	T->point[1][3] = 0;

	T->point[2][0] = 0;
	T->point[2][1] = 0;
	T->point[2][2] = 1;
	T->point[2][3] = 0;

	T->point[3][0] = 0;
	T->point[3][1] = 0;
	T->point[3][2] = 0;
	T->point[3][3] = 1;
	return;
}

void transz(double d_mm, matxx* T)
{
	T->point[0][0] = 1;
	T->point[0][1] = 0;
	T->point[0][2] = 0;
	T->point[0][3] = 0;

	T->point[1][0] = 0;
	T->point[1][1] = 1;
	T->point[1][2] = 0;
	T->point[1][3] = 0;

	T->point[2][0] = 0;
	T->point[2][1] = 0;
	T->point[2][2] = 1;
	T->point[2][3] = 0;

	T->point[3][0] = 0;
	T->point[3][1] = 0;
	T->point[3][2] = d_mm;
	T->point[3][3] = 1;

	return;
}