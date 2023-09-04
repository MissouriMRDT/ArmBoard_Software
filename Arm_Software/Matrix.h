/*
 *  MatrixMath.h Library for Matrix Math
 *
 *  Created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, algorithm from
 *  NUMERICAL RECIPES: The Art of Scientific Computing.
 *  Modified to work with Arduino 1.0/1.5 by randomvibe & robtillaart
 *  Made into a real library on GitHub by Vasilis Georgitzikis (tzikis)
 *  so that it's easy to use and install (March 2015)
 *
 *
 *  Converted by drue satterfield to C friendly interface
 */

#ifndef CMatrixMath_h
#define CMatrixMath_h

#ifdef __cplusplus
extern "C" {
#endif

//copies matrix a (n x m) to matrix b (n x m)
void matrixMathCopy(float* A, int n, int m, float* B);

// A = input matrix (m x p)
// B = input matrix (p x n)
// m = number of rows in A
// p = number of columns in A = number of rows in B
// n = number of columns in B
// C = output matrix = A*B (m x n)
void matrixMathMultiply(float* A, float* B, int m, int p, int n, float* C);

// A = input matrix (m x n)
// B = input matrix (m x n)
// m = number of rows in A = number of rows in B
// n = number of columns in A = number of columns in B
// C = output matrix = A+B (m x n)
void matrixMathAdd(float* A, float* B, int m, int n, float* C);

// A = input matrix (m x n)
// B = input matrix (m x n)
// m = number of rows in A = number of rows in B
// n = number of columns in A = number of columns in B
// C = output matrix = A-B (m x n)
void matrixMathSubtract(float* A, float* B, int m, int n, float* C);

// A = input matrix (m x n)
// m = number of rows in A
// n = number of columns in A
// C = output matrix = the transpose of A (n x m)
void matrixMathTranspose(float* A, int m, int n, float* C);

//scales the values in input matrix A (m x n) by k
void matrixMathScale(float* A, int m, int n, float k);

//Matrix Inversion Routine
// * This function inverts a matrix based on the Gauss Jordan method.
// * Specifically, it uses partial pivoting to improve numeric stability.
// * The algorithm is drawn from those presented in
//   NUMERICAL RECIPES: The Art of Scientific Computing.
// * The function returns 1 on success, 0 on failure.
// * NOTE: The argument is ALSO the result matrix, meaning the input matrix is REPLACED
//
// A = input matrix AND result matrix
// n = number of rows = number of columns in A (n x n)
//int matrixMathInvert(float* A, int n);

#ifdef __cplusplus
}
#endif

#endif