//
// Created by qixiaofeng on 2020/8/22.
//

#ifndef QXF_CANVAS_MATRIX_H
#define QXF_CANVAS_MATRIX_H

#define M_define_matrix(m, n)\
typedef struct M##m##x##n {\
    double numbers[m][n];\
} M##m##x##n;\
double M##m##x##n##_get_item(M##m##x##n const *matrix, unsigned i, unsigned j) {\
    return matrix->numbers[i - 1][j - 1];\
}

#define M_define_matrix_multiply(m, s, n)\
void M##m##x##s##_multiply_M##s##x##n(M##m##x##n *output, M##m##x##s const *a, M##s##x##n const *b) {\
    (void)output, (void)a, (void)b;\
    for (unsigned i = 0; i < m; ++i) {\
        for (unsigned j = 0; j < n; ++j) {\
            double sum = 0.0;\
            for (unsigned k = 0; k < s; ++k) {\
                sum += a->numbers[i][k] * b->numbers[k][j];\
            }\
            output->numbers[i][j] = sum;\
        }\
    }\
}

M_define_matrix(2, 2)

M_define_matrix(1, 2)

M_define_matrix_multiply(1, 2, 2)

#endif //QXF_CANVAS_MATRIX_H
