//
// Created by qixiaofeng on 2020/8/20.
//

#include <math.h>
#include <stdbool.h>
#include "matrix.h"
#include "physics.h"

#define m_g 9.8
#define m_pixels_per_meter 50

typedef struct p_CircleCollision {
    Point contactPoint;
    Vector normalVector;
    bool isCollided;
} CircleCollision;

void p_get_rotation_matrix(M2x2 *output, double theta) {
    double const sinTheta = sin(theta), cosTheta = cos(theta);
    output->ns[0][0] = cosTheta;
    output->ns[0][1] = -sinTheta;
    output->ns[1][0] = sinTheta;
    output->ns[1][1] = cosTheta;
}

double p_calc_vector_len(Vector const *v) {
    return sqrt(M1x2_dot_M1x2(v, v));
}

void p_normalize_vector(Vector *output, Vector const *v, double length) {
    output->ns[0][0] = v->ns[0][0] / length;
    output->ns[0][1] = v->ns[0][1] / length;
}

void vector_rotate(Vector *output, Vector const *v, double theta) {
    static M2x2 rotation;
    p_get_rotation_matrix(&rotation, theta);

    M1x2 const toMultiply = {{{v->ns[0][0], v->ns[0][1]}}};
    M1x2 rotated;
    M1x2_multiply_M2x2(&rotated, &toMultiply, &rotation);
    output->ns[0][0] = rotated.ns[0][0];
    output->ns[0][1] = rotated.ns[0][1];
}

bool p_is_circle_overlap(Circle const *circleA, Circle const *circleB, CircleCollision *output) {
    static Vector ab;
    ab.ns[0][0] = circleB->origin.ns[0][0] - circleA->origin.ns[0][0];
    ab.ns[0][1] = circleB->origin.ns[0][1] - circleA->origin.ns[0][1];

    output->isCollided = true;
    if (0 == ab.ns[0][0] && 0 == ab.ns[0][1]) {
        output->contactPoint.ns[0][0] = circleA->origin.ns[0][0];
        output->contactPoint.ns[0][1] = circleA->origin.ns[0][1];
        output->normalVector.ns[0][0] = 0;
        output->normalVector.ns[0][1] = 0;
        return output->isCollided;
    }
    double const
            radiusSum = circleA->radius + circleB->radius,
            length = p_calc_vector_len(&ab);
    if (length <= radiusSum) {
        double const acLength = length * circleA->radius / radiusSum;
        p_normalize_vector(&ab, &ab, length);
        vector_rotate(&output->normalVector, &ab, 3.1415926 * 0.5);
        M1x2_multiply_scalar(&ab, &ab, acLength);
        M1x2_add_M1x2(&output->contactPoint, &ab, &circleA->origin);
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

void get_circle_aabb(AABB *output, Circle const *circle) {
    output->left = circle->origin.ns[0][0] - circle->radius;
    output->top = circle->origin.ns[0][1] - circle->radius;
    output->right = circle->origin.ns[0][0] + circle->radius;
    output->bottom = circle->origin.ns[0][1] + circle->radius;
}

void update_motion(RigidCircle *rigidCircle, double deltaSeconds) {
    #define M_update(t) \
    double const\
        velocityIncrement##t = deltaSeconds * rigidCircle->motion.acceleration.ns[0][t],\
        velocityIncrementHalf##t = velocityIncrement##t * 0.5,\
        velocityMiddle##t = rigidCircle->motion.velocity.ns[0][t] + velocityIncrementHalf##t,\
        movement##t = deltaSeconds * velocityMiddle##t;\
    /*printf(#t": %+5.4f, d"#t": %+5.4f, ds: %+5.4f, mv: %+5.4f; ",\
        rigidCircle->motion.velocity.t, rigidCircle->motion.acceleration.t,\
        deltaSeconds, t##Movement);*/\
    rigidCircle->motion.velocity.ns[0][t] += velocityIncrement##t;\
    rigidCircle->circle.origin.ns[0][t] += movement##t;

    M_update(0)
    M_update(1)
    // printf("\n");

    #undef M_update
}

void add_gravity_to(RigidCircle *rigidCircle) {
    static double const G_actual_g = m_g * m_pixels_per_meter;
    rigidCircle->motion.acceleration.ns[0][1] += G_actual_g;
}

void collide_circle_with_aabb(RigidCircle *rigidCircle, AABB const *aabb) {
    static AABB circleAABB;
    get_circle_aabb(&circleAABB, &rigidCircle->circle);

    if (false == p_is_aabb_overlap(&circleAABB, aabb)) {
        return;
    }

    if (
            (rigidCircle->circle.origin.ns[0][1] < aabb->top && rigidCircle->motion.velocity.ns[0][1] > 0) ||
            (rigidCircle->circle.origin.ns[0][1] > aabb->bottom && rigidCircle->motion.velocity.ns[0][1] < 0)
            ) {
        // Vertical
        rigidCircle->motion.velocity.ns[0][1] = -rigidCircle->motion.velocity.ns[0][1];
    } else {
        // Horizontal
        rigidCircle->motion.velocity.ns[0][0] = -rigidCircle->motion.velocity.ns[0][0];
    }
}

void collide_circle_with_circle(RigidCircle *rigidCircle, Circle const *staticCircle) {
    static AABB a, b;
    static CircleCollision collision;
    get_circle_aabb(&a, &rigidCircle->circle);
    get_circle_aabb(&b, staticCircle);
    if (false == p_is_aabb_overlap(&a, &b)) {
        return;
    }
    if (false == p_is_circle_overlap(&rigidCircle->circle, staticCircle, &collision)) {
        return;
    }
    /**
     * a and b are vectors.
     * a_1 is projection of a on b.
     * a_2 is rejection of a on b.
     * a_1 = b * \dot(a, b)/\dot(b, b);
     * a_2 = a - a_1;
     */
    // TODO Implement.
}

void collide_circles(RigidCircle *rigidCircleA, RigidCircle *rigidCircleB) {
    static AABB aabbA, aabbB;
    static CircleCollision collision;
    get_circle_aabb(&aabbA, &rigidCircleA->circle);
    get_circle_aabb(&aabbB, &rigidCircleB->circle);

    if (false == p_is_aabb_overlap(&aabbA, &aabbB)) {
        return;
    }

    if (false == p_is_circle_overlap(&rigidCircleA->circle, &rigidCircleB->circle, &collision)) {
        return;
    }
}

#undef m_g
#undef m_pixels_per_meter
