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

#define M_declare_matrix_multiply_scalar(m, n)\
void M##m##x##n##_multiply_scalar(M##m##x##n *output, M##m##x##n const *a, double s)

#define M_define_matrix_multiply_scalar(m, n) M_declare_matrix_multiply_scalar(m, n) {\
    for(unsigned i = 0; i < m; ++i) {\
        for (unsigned j = 0; j < n; ++j) {\
            output->ns[i][j] = a->ns[i][j] * s;\
        }\
    }\
}

#define M_declare_vector_dot(n)\
double M1x##n##_dot_M1x##n(M1x##n const *a, M1x##n const *b)

#define M_define_vector_dot(n) M_declare_vector_dot(n) {\
    double dot = 0;\
    for (unsigned i = 0; i < n; ++i) {\
        dot += (a->ns[0][i] * b->ns[0][i]);\
    }\
    return dot;\
}

M_declare_matrix(1, 2);

M_declare_matrix(2, 2);

M_declare_matrix_multiply(1, 2, 2);

M_declare_matrix_multiply(2, 2, 2);

M_declare_matrix(3, 3);

M_declare_matrix_multiply(3, 3, 3);

M_declare_vector_dot(2);

M_declare_matrix_multiply_scalar(1, 2);

/**
 * tau is the permutation inversions number.
 */
unsigned calc_indices_permutation_tau(unsigned const *indices, unsigned count);

#endif //QXF_CANVAS_MATRIX_H
