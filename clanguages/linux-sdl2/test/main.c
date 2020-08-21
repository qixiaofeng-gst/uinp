#include "testool.h"
#include "test-physics.h"

void p_test_placeholder() {
    AABB aabbOne = {{0,  0,},
                    {10, 10,},};
    AABB aabbTwo = {{10, 10,},
                    {20, 20},};
    M_test_int(p_is_overlap(&aabbOne, &aabbTwo), false);
    aabbTwo.leftTop.x = 9;
    M_test_int(p_is_overlap(&aabbOne, &aabbTwo), false);
    aabbTwo.leftTop.y = 9;
    M_test_int(p_is_overlap(&aabbOne, &aabbTwo), true);
}

int main() {
    M_run_test_suite(p_test_placeholder);
    return 0;
}
