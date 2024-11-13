#pragma once

#include <ode/common.h>

//------------------------------------------------------------------------------
// Missing from <ode/common.h>

#if defined(dSINGLE)
#define dAsin(x) ((float)asinf(float(x)))
#elif defined(dDOUBLE)
#define dAsin(x) asin(x)
#else
#error You must #define dSINGLE or dDOUBLE
#endif

typedef dReal Vector6[6];
typedef dReal Matrix33[9];

//------------------------------------------------------------------------------
// Lin. Algebra

/* set a vector/matrix of size n to all zeros, or to a specific value. */
void utils_SetZero(dReal *a, int n);
void utils_SetValue(dReal *a, int n, dReal value);
void utils_Assign(dReal *a, const dReal *b, int n);
void utils_Add(dReal *a, const dReal *b, int n);

/* matrix multiplication. all matrices are stored in standard row format.
 * the digit refers to the argument that is transposed:
 *		A = B  * C   (sizes: A:p*r B:p*q C:q*r)
 */
void utils_Multiply(dReal *A, const dReal *B, const dReal *C, int p, int q, int r);

/* Matrix inversion */
void utils_InvertMatrix33(dReal *m);
