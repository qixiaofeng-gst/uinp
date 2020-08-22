//
// Created by qixiaofeng on 2020/8/20.
//

#include <math.h>
#include <stdbool.h>
#include "physics.h"

#define m_g 9.8
#define m_pixels_per_meter 50

typedef struct p_CircleCollision {
    Point contactPoint;
    Vector normalVector;
    bool isCollided;
} CircleCollision;

double p_calc_vector_len(Vector const *v) {
    return sqrt(v->x * v->x + v->y * v->y);
}

void p_normalize_vector(Vector *output, Vector const *v, double length) {
    output->x = v->x / length;
    output->y = v->y / length;
}

void p_vector_multiply_scalar(Vector *output, Vector const *v, double s) {
    output->x = v->x * s;
    output->y = v->y * s;
}

void p_vector_add_vector(Vector *output, Vector const *a, Vector const *b) {
    output->x = a->x + b->x;
    output->y = a->y + b->y;
}

void p_vector_rotate(Vector *output, Vector const *v, double theta) {
    // TODO Implement.
    (void) output, (void) v, (void) theta;
}

bool p_is_circle_overlap(Circle const *circleA, Circle const *circleB, CircleCollision *output) {
    static Vector ab;
    ab.x = circleB->origin.x - circleA->origin.x;
    ab.y = circleB->origin.y - circleA->origin.y;

    output->isCollided = true;
    if (0 == ab.x && 0 == ab.y) {
        output->contactPoint.x = circleA->origin.x;
        output->contactPoint.y = circleA->origin.y;
        output->normalVector.x = 0;
        output->normalVector.y = 0;
        return output->isCollided;
    }
    double const
            radiusSum = circleA->radius + circleB->radius,
            length = p_calc_vector_len(&ab);
    if (length <= radiusSum) {
        double const acLength = length * circleA->radius / radiusSum;
        p_normalize_vector(&ab, &ab, length);
        p_vector_rotate(&output->normalVector, &ab, 3.1415926 * 0.5);
        p_vector_multiply_scalar(&ab, &ab, acLength);
        p_vector_add_vector(&output->contactPoint, &ab, &circleA->origin);
        return output->isCollided;
    }

    output->isCollided = false;
    return output->isCollided;
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

bool p_is_aabb_overlap(AABB const *a, AABB const *b) {
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

    if (false == p_is_aabb_overlap(&circleAABB, aabb)) {
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

void collide_circles(RigidCircle *rigidCircleA, RigidCircle *rigidCircleB) {
    static AABB aabbA, aabbB;
    static CircleCollision collision;
    get_circle_aabb(&rigidCircleA->circle, &aabbA);
    get_circle_aabb(&rigidCircleB->circle, &aabbB);

    if (false == p_is_aabb_overlap(&aabbA, &aabbB)) {
        return;
    }

    if (false == p_is_circle_overlap(&rigidCircleA->circle, &rigidCircleB->circle, &collision)) {
        return;
    }
}

#undef m_g
#undef m_pixels_per_meter
