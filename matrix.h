#ifndef MATRIX_H
#define MATRIX_H

void matrix_transpose(double *, double *, int, int);
void matrix_multiply(double *, double *, double *, int, int, int);
double* matrix_invert(int numRowsCols, double* dstM, double* srcM, double* pivotFlag, int* swappedRows, int* swappedCols);

#endif
