//
// Created by qixiaofeng on 2020/8/21.
//

#ifndef QXF_CANVAS_TEST_PHYCIS_H
#define QXF_CANVAS_TEST_PHYCIS_H

#include "physics.h"

bool p_is_overlap(AABB const *a, AABB const *b);
void p_print_aabb(AABB const *aabb, char const *name);

#endif //QXF_CANVAS_TEST_PHYCIS_H
