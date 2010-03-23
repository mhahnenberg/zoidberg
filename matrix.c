#include "matrix.h"
#include <math.h>
#include <stdlib.h>

void matrix_transpose(double *result, double *matrix, int rows, int cols) {
	int i, j;
	for (i = 0; i < rows; i++)
		for (j = 0; j < cols; j++)
			result[rows*j+i] = matrix[cols*i+j];	
}

void matrix_multiply(double *result, double *matrix1, double *matrix2, int rows, int cols1, int cols2) {
	int i, j, k;
	for (i = 0; i < rows; i++) {
		for (k = 0; k < cols2; k++) {
			result[cols2*i+k] = 0.0;
			for (j = 0; j < cols1; j++)
				result[cols2*i+k] += matrix1[cols1*i+j]*matrix2[cols2*j+k];
		}
	}	
}

//Based on Microchip library
double* matrix_invert (
   int numRowsCols,			/* number rows and columns in matrix */
   double* dstM,				/* ptr to destination matrix */
   double* srcM,				/* ptr to source matrix */
   double* pivotFlag,			/* internal use; size numRowsCols */
   int* swappedRows,			/* internal use; size numRowsCols */
   int* swappedCols			/* internal use; size numRowsCols */
   					/* last three vectors required from */
					/* user, so that function is not */
					/* responsible for memory management */
					/* dstM returned (or NULL on error */
					/* if source matrix is singular) */
) {

   /* Local declarations. */
   double* retM = dstM;
   double absVal = 0;
   double maxVal = 0;
   int cntr = 0;
   int r = 0;
   int c = 0;
   int ir = 0;
   int ic = 0;

   /* Initialized local arrays to zero. */
   for (r = 0; r < numRowsCols; r++) {
      pivotFlag[r] = 0.0;
      swappedRows[r] = 0;
      swappedCols[r] = 0;
   }

   /* Since the Gauss-Jordan algorithm operates in place... */
   if (srcM != dstM) {
      for (r = 0; r < numRowsCols; r++) {
         for (c = 0; c < numRowsCols; c++) {
            *(dstM++) = *(srcM++);
         }
      }
      dstM = retM;						/* rewind */
   }

   /* Now, apply algorithm to dstM. */
   for (cntr = 0; cntr < numRowsCols; cntr++) {		/* pivoting iterates */

      /* Find pivot element. */
      maxVal = 0;
      for (r = 0; r < numRowsCols; r++) {
	 if (!pivotFlag[r]) {				/* unused pivot */
	    for (c = 0; c < numRowsCols; c++) {
	       if (!pivotFlag[c]) {			/* unused pivot */
	          absVal = fabs (dstM[r*numRowsCols+c]);
		  if (absVal >= maxVal) {
		     /* Update. */
		     maxVal = absVal;
		     ir = r;
		     ic = c;
		  }
	       }
	    }
	 }
      }
      pivotFlag[ic]++;					/* mark pivot used */

      /* Swap rows to make this diagonal the largest absolute pivot. */
      if (ir != ic) {
         for (c = 0; c < numRowsCols; c++) {
	    absVal = dstM[ir*numRowsCols+c];		/* reusing absVal */
	    dstM[ir*numRowsCols+c] = dstM[ic*numRowsCols+c];
	    dstM[ic*numRowsCols+c] = absVal;
	 }
      }

      /* Update swapping status. */
      swappedRows[cntr] = ir;
      swappedCols[cntr] = ic;

      /* Bail out if matrix is singular. */
      if (dstM[ic*numRowsCols+ic] == 0.0 ) {
         return ((double*) NULL);
      }

      /* Divide the row by the pivot. */
      absVal = 1.0/dstM[ic*numRowsCols+ic];		/* reusing absVal */
      dstM[ic*numRowsCols+ic] = 1.0;			/* avoid round off */
      for (c = 0; c < numRowsCols; c++) {
         dstM[ic*numRowsCols+c] *= absVal;
      }

      /* Fix other rows by subtraction. */
      for (r = 0; r < numRowsCols; r++) {
         if (r != ic) {
	    absVal = dstM[r*numRowsCols+ic];		/* reusing absVal */
	    dstM[r*numRowsCols+ic] = 0.0;
	    for (c = 0; c < numRowsCols; c++) {
	       dstM[r*numRowsCols+c] -= (dstM[ic*numRowsCols+c]*absVal);
	    }
	 }
      }
   }

   /* Reorganized swaps prior to returning. */
   for (c = numRowsCols-1; c >= 0; c--) {
      if (swappedRows[c] != swappedCols[c]) {
         for (r = 0; r < numRowsCols; r++) {
	    absVal = dstM[r*numRowsCols+swappedRows[c]];/* reusing absVal */
	    dstM[r*numRowsCols+swappedRows[c]] = dstM[r*numRowsCols+swappedCols[c]];
	    dstM[r*numRowsCols+swappedCols[c]] = absVal;
	 }
      }
   }

   /* Return destination vector pointer. */
   return (retM);

} 
