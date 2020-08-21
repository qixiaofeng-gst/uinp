//
// Created by qixiaofeng on 2020/8/20.
//

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "physics.h"

#define m_g 9.8
#define m_pixels_per_meter 50
#define M_print_aabb(aabb) p_print_aabb(aabb, #aabb, __FUNCTION__)

void p_print_aabb(AABB const *aabb, char const *name, char const *function) {
    printf("%s: AABB [%s], left: %6.2f, right: %6.2f; "
           "top: %6.2f, bottom: %6.2f\n",
           function, name,
           aabb->left, aabb->right,
           aabb->top, aabb->bottom);
}

bool p_is_segment_overlap(double aMin, double aMax, double bMin, double bMax) {
    if (aMin > bMax) {
        return false;
    }
    if (bMin > aMax) {
        return false;
    }
    return true;
}

bool p_is_overlap(AABB const *a, AABB const *b) {
    if (false == p_is_segment_overlap(a->left, a->right, b->left, b->right)) {
        return false;
    }
    if (false == p_is_segment_overlap(a->top, a->bottom, b->top, b->bottom)) {
        return false;
    }
    // M_print_aabb(a);
    // M_print_aabb(b);
    return true;
}

void get_circle_aabb(Circle const *circle, AABB *aabb) {
    aabb->left = circle->origin.x - circle->radius;
    aabb->top = circle->origin.y - circle->radius;
    aabb->right = circle->origin.x + circle->radius;
    aabb->bottom = circle->origin.y + circle->radius;
}

void update_motion(RigidCircle *rigidCircle, double deltaSeconds) {
    #define M_update(t) \
    double const\
        t##VelocityIncrement = deltaSeconds * rigidCircle->motion.acceleration.t,\
        t##VelocityIncrementHalf = t##VelocityIncrement * 0.5,\
        t##VelocityMiddle = rigidCircle->motion.velocity.t + t##VelocityIncrementHalf,\
        t##Movement = deltaSeconds * t##VelocityMiddle;\
    /*printf(#t": %+5.4f, d"#t": %+5.4f, ds: %+5.4f, mv: %+5.4f; ",\
        rigidCircle->motion.velocity.t, rigidCircle->motion.acceleration.t,\
        deltaSeconds, t##Movement);*/\
    rigidCircle->motion.velocity.t += t##VelocityIncrement;\
    rigidCircle->circle.origin.t += t##Movement;

    M_update(y)
    M_update(x)
    // printf("\n");

    #undef M_update
}

void add_gravity_to(RigidCircle *rigidCircle) {
    static double const G_actual_g = m_g * m_pixels_per_meter;
    rigidCircle->motion.acceleration.y += G_actual_g;
}

void collide_circle_with_aabb(RigidCircle *rigidCircle, AABB const *aabb) {
    static AABB circleAABB;
    get_circle_aabb(&rigidCircle->circle, &circleAABB);

    if (false == p_is_overlap(&circleAABB, aabb)) {
        return;
    }

    if (
            (rigidCircle->circle.origin.y < aabb->top && rigidCircle->motion.velocity.y > 0) ||
            (rigidCircle->circle.origin.y > aabb->bottom && rigidCircle->motion.velocity.y < 0)
            ) {
        // Vertical
        rigidCircle->motion.velocity.y = -rigidCircle->motion.velocity.y;
    } else {
        // Horizontal
        rigidCircle->motion.velocity.x = -rigidCircle->motion.velocity.x;
    }
}

#undef m_g
#undef m_pixels_per_meter
