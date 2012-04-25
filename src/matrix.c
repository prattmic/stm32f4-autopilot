/* Michael Pratt <michael@pratt.im>
 * Do matrix operations */

#include "matrix.h"

/* Multiplies matrices of indeterminate size.
 * Obviously, A->cols must equal B->rows, and
 * out->rows = A->rows and out->cols = B->cols */
void matrix_multiply(struct matrix *A, struct matrix *B, struct matrix *out) {
    int i, j, k = 0;

    for (i = 0; i < A->rows; i++) {
        for (j = 0; j < B->cols; j++) {
            out->data[out->cols*i + j] = 0;
            for (k = 0; k < A->cols; k++) {
                out->data[out->cols*i + j] += A->data[A->cols*i + k] * B->data[B->cols*k + j];
            }
        }
    }
}

/* Inverts a 2x2 matrix.  Make sure the output size is 2x2
 * See http://en.wikipedia.org/wiki/Invertible_matrix#Inversion_of_2.C3.972_matrices */
void matrix_invert_2(struct matrix *A, struct matrix *out) {
    float idet = 1/(A->data[0]*A->data[3] - A->data[1]*A->data[2]);     /* 1/det */
    /* By saving the first index, this function can invert a matrix in place by simply passing the same struct for each argument. */
    float i0 = out->data[0];        

    out->data[0] = idet * A->data[3];
    out->data[1] = idet * -A->data[1];
    out->data[2] = idet * -A->data[2];
    out->data[3] = idet * i0;
}

void dumb_matrix_multiply(volatile float *one, volatile float *two, volatile float *out) {
    /* Dumb matrix multiplication expects 3x3 matrix */

    out[0] = one[0]*two[0] + one[1]*two[3] + one[2]*two[6];
    out[1] = one[0]*two[1] + one[1]*two[4] + one[2]*two[7];
    out[2] = one[0]*two[2] + one[1]*two[5] + one[2]*two[8];
    out[3] = one[3]*two[0] + one[4]*two[3] + one[5]*two[6];
    out[4] = one[3]*two[1] + one[4]*two[4] + one[5]*two[7];
    out[5] = one[3]*two[2] + one[3]*two[5] + one[5]*two[8];
    out[6] = one[6]*two[0] + one[7]*two[3] + one[8]*two[6];
    out[7] = one[6]*two[1] + one[7]*two[4] + one[8]*two[7];
    out[8] = one[6]*two[2] + one[7]*two[5] + one[8]*two[8];
}
