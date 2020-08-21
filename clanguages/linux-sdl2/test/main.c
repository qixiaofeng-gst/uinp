#include "testool.h"
#include "test-physics.h"

/*TODO
 * 1. Four direction bounce.
 * 2. Double-buffer.
 */

void p_test_is_overlap() {
    AABB aabbOne = {0, 0, 10, 10,};
    AABB aabbTwo = {10.01, 10.01, 20, 20};
    p_print_aabb(&aabbOne, "test aabb");
    M_test_int(p_is_overlap(&aabbOne, &aabbTwo), false);
    aabbTwo.left = 10;
    M_test_int(p_is_overlap(&aabbOne, &aabbTwo), false);
    aabbTwo.top = 10;
    M_test_int(p_is_overlap(&aabbOne, &aabbTwo), true);
    aabbTwo.top = 9.99;
    M_test_int(p_is_overlap(&aabbOne, &aabbTwo), true);
    aabbTwo.left = -0.01;
    aabbTwo.right = 10.01;
    M_test_int(p_is_overlap(&aabbOne, &aabbTwo), true);
}

int main() {
    M_run_test_suite(p_test_is_overlap);
    return 0;
}
