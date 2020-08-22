//
// Created by qixiaofeng on 2020/8/22.
//

#ifndef QXF_CANVAS_MATRIX_H
#define QXF_CANVAS_MATRIX_H

#define p_M_matrix_get_item(m, n)\
/** This method use index that start from number 1. */\
double M##m##x##n##_get_item(M##m##x##n const *matrix, unsigned i, unsigned j)

#define M_declare_matrix(m, n)\
typedef struct M##m##x##n {\
    double ns[m][n];\
} M##m##x##n;\
p_M_matrix_get_item(m, n)

#define M_define_matrix(m, n) p_M_matrix_get_item(m, n) {\
    return matrix->ns[i - 1][j - 1];\
}

// LaTeX:
// c_{i,j} = \sum_{k=0}^{s}{ a_{i,k} * b_{k,j} }
#define M_declare_matrix_multiply(m, s, n)\
void M##m##x##s##_multiply_M##s##x##n(M##m##x##n *output, M##m##x##s const *a, M##s##x##n const *b)

#define M_define_matrix_multiply(m, s, n) M_declare_matrix_multiply(m, s, n) {\
    for (unsigned i = 0; i < m; ++i) {\
        for (unsigned j = 0; j < n; ++j) {\
            double sum = 0.0;\
            for (unsigned k = 0; k < s; ++k) {\
                sum += a->ns[i][k] * b->ns[k][j];\
            }\
            output->ns[i][j] = sum;\
        }\
    }\
}

M_declare_matrix(1, 2);

M_declare_matrix(2, 2);

M_declare_matrix_multiply(1, 2, 2);

M_declare_matrix_multiply(2, 2, 2);

M_declare_matrix(3, 3);

M_declare_matrix_multiply(3, 3, 3);

/**
 * tau is the permutation inversions number.
 */
unsigned calc_indices_permutation_tau(unsigned const *indices, unsigned count);

#endif //QXF_CANVAS_MATRIX_H
