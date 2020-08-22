//
// Created by qixiaofeng on 2020/8/22.
//

#ifndef QXF_CANVAS_MATRIX_H
#define QXF_CANVAS_MATRIX_H

#define M_declare_matrix(m, n) \
typedef struct M##m##x##n { \
    double numbers[m][n]; \
} M##m##x##n;\
double M##m##x##n##_get_item(M##m##x##n const *matrix, unsigned i, unsigned j) {\
    return matrix->numbers[i - 1][j - 1];\
}

#define M_define_matrix_operators(aM, aN, bM, bN) \
void M##aM##x##aN##_multiply_M##bM##x##bN(M##aM##x##aN *output, M##aM##x##aN const *a, M##bM##x##bN const *b) {\
    (void)output, (void)a, (void)b;\
}

M_declare_matrix(2,2)
M_declare_matrix(1, 2)
M_define_matrix_operators(1, 2, 2, 2)

#endif //QXF_CANVAS_MATRIX_H
