#include "testool.h"
#include "test-physics.h"

#include "matrix.h"

/*TODO
 * # Two ball bounce.
 *   - Both moving.
 *   - One moving.
 * # Matrix math.
 * # Engine: File based objects loader.
 * # Engine: Objects manager.
 * # Engine: Physics manager.
 * # Double-buffer.
 */

#define M_print_aabb(aabb) p_print_aabb(aabb, #aabb, __FUNCTION__)

void p_print_aabb(AABB const *aabb, char const *name, char const *function) {
    printf("%s: AABB [%s], left: %6.2f, right: %6.2f; "
           "top: %6.2f, bottom: %6.2f\n",
           function, name,
           aabb->left, aabb->right,
           aabb->top, aabb->bottom);
}

void p_test_matrix() {
    M1x2 m1X2 = {{{2, 3}}}, out1x2;
    M2x2
            i2x2 = {{{1, 0}, {0, 1}}},
            a2x2 = {{{4, 5}, {6, 7}}},
            out2x2;

    M1x2_multiply_M2x2(&out1x2, &m1X2, &i2x2);
    M_test_int(M1x2_get_item(&m1X2, 1, 1) == M1x2_get_item(&out1x2, 1, 1), true);
    M_test_int(M1x2_get_item(&m1X2, 1, 2) == M1x2_get_item(&out1x2, 1, 2), true);
    M2x2_multiply_M2x2(&out2x2, &i2x2, &a2x2);
    M_test_int(M2x2_get_item(&a2x2, 1, 1) == M2x2_get_item(&out2x2, 1, 1), true);
    M_test_int(M2x2_get_item(&a2x2, 1, 2) == M2x2_get_item(&out2x2, 1, 2), true);
    M_test_int(M2x2_get_item(&a2x2, 2, 1) == M2x2_get_item(&out2x2, 2, 1), true);
    M_test_int(M2x2_get_item(&a2x2, 2, 2) == M2x2_get_item(&out2x2, 2, 2), true);

    M3x3
            a3x3 = {{{8, 9, 10,}, {11, 12, 13,}, {14, 15, 16,}}},
            i3x3 = {{{1, 0, 0,}, {0, 1, 0,}, {0, 0, 1,}}},
            out3x3;
    M3x3_multiply_M3x3(&out3x3, &a3x3, &i3x3);
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j < 3; ++j) {
            M_test_int(a3x3.ns[i][j] == out3x3.ns[i][j], true);
        }
    }
}

void p_test_misc() {
    unsigned p[] = {1, 3, 9, 2, 4, 6, 8, 5, 7,};
    M_test_int(calc_indices_permutation_tau(p, sizeof(p) / sizeof(unsigned)), 1 + 6 + 1 + 2);
}

void p_test_is_overlap() {
    AABB aabbOne = {0, 0, 10, 10,};
    AABB aabbTwo = {10.01, 10.01, 20, 20};
    M_print_aabb(&aabbOne);
    M_test_int(p_is_aabb_overlap(&aabbOne, &aabbTwo), false);
    aabbTwo.left = 10;
    M_test_int(p_is_aabb_overlap(&aabbOne, &aabbTwo), false);
    aabbTwo.top = 10;
    M_test_int(p_is_aabb_overlap(&aabbOne, &aabbTwo), true);
    aabbTwo.top = 9.99;
    M_test_int(p_is_aabb_overlap(&aabbOne, &aabbTwo), true);
    aabbTwo.left = -0.01;
    aabbTwo.right = 10.01;
    M_test_int(p_is_aabb_overlap(&aabbOne, &aabbTwo), true);
}

int main() {
    M_run_test_suite(p_test_is_overlap);
    M_run_test_suite(p_test_matrix);
    M_run_test_suite(p_test_misc);
    return 0;
}
