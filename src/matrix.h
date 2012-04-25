/* Michael Pratt <michael@pratt.im>
 * Matrix operations */

#ifndef MATRIX_H_INCLUDED
#define MATRIX_H_INCLUDED

struct matrix {
    volatile float *data;
    int rows;
    int cols;
};

void matrix_multiply(struct matrix *A, struct matrix *B, struct matrix *out);
void matrix_invert_2(struct matrix *A, struct matrix *out);
void dumb_matrix_multiply(volatile float *one, volatile float *two, volatile float *out);

#endif /* MATRIX_H_INCLUDED */
