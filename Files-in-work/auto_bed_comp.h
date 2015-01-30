#ifndef AUTO_BED_COMP_H
#define AUTO_BED_COMP_H

/*****************************************************************************************************************************
  This file is a mash up of vector_3.h & qr_solve.h from Marlin for use in Dolphin
  
  Created by Meek66e 1/30/15
*****************************************************************************************************************************/

#include Configuration.h

//==========================================================================================================================//
//=====================================================3 Point Bed Comp=====================================================//
//==========================================================================================================================//

#ifdef ENABLE_AUTO_BED_LEVELING
class matrix_3x3;

struct vector_3
{
	float x, y, z;

        vector_3();
	vector_3(float x, float y, float z);

	static vector_3 cross(vector_3 a, vector_3 b);

	vector_3 operator+(vector_3 v);
	vector_3 operator-(vector_3 v);
	void normalize();
	float get_length();
	vector_3 get_normal();

	void debug(char* title);
	
	void apply_rotation(matrix_3x3 matrix);
};

struct matrix_3x3
{
	float matrix[9];

	static matrix_3x3 create_from_rows(vector_3 row_0, vector_3 row_1, vector_3 row_2);
	static matrix_3x3 create_look_at(vector_3 target);
	static matrix_3x3 transpose(matrix_3x3 original);

	void set_to_identity();

	void debug(char* title);
};


void apply_rotation_xyz(matrix_3x3 rotationMatrix, float &x, float& y, float& z);
#endif // ENABLE_AUTO_BED_LEVELING

//==========================================================================================================================//
//====================================================Advanced Bed Comp=====================================================//
//==========================================================================================================================//

#ifdef ADVANCED_BED_LEVELING

void daxpy ( int n, double da, double dx[], int incx, double dy[], int incy );
double ddot ( int n, double dx[], int incx, double dy[], int incy );
double dnrm2 ( int n, double x[], int incx );
void dqrank ( double a[], int lda, int m, int n, double tol, int *kr, 
  int jpvt[], double qraux[] );
void dqrdc ( double a[], int lda, int n, int p, double qraux[], int jpvt[], 
  double work[], int job );
int dqrls ( double a[], int lda, int m, int n, double tol, int *kr, double b[], 
  double x[], double rsd[], int jpvt[], double qraux[], int itask );
void dqrlss ( double a[], int lda, int m, int n, int kr, double b[], double x[], 
  double rsd[], int jpvt[], double qraux[] );
int dqrsl ( double a[], int lda, int n, int k, double qraux[], double y[], 
  double qy[], double qty[], double b[], double rsd[], double ab[], int job );
void dscal ( int n, double sa, double x[], int incx );
void dswap ( int n, double x[], int incx, double y[], int incy );
double *qr_solve ( int m, int n, double a[], double b[] );

#endif // ADVANCED_BED_LEVELING


#endif // EOF
