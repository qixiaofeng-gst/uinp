//
// Created by qixiaofeng on 2020/8/22.
//

#include "matrix.h"

M_define_matrix(2, 2)

M_define_matrix(1, 2)

M_define_matrix_multiply(1, 2, 2)

M_define_matrix_multiply(2, 2, 2)

M_define_matrix(3, 3)

M_define_matrix_multiply(3, 3, 3)

unsigned calc_indices_permutation_tau(unsigned const *indices, unsigned count) {
    unsigned tau = 0;
    for (unsigned i = 0; i < count; ++i) {
        for (unsigned j = i + 1; j < count; ++j) {
            if (indices[i] > indices[j]) {
                ++tau;
            }
        }
    }
    return tau;
}
