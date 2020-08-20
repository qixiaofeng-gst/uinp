//
// Created by qixiaofeng on 2020/8/20.
//

#include <stdio.h>
#include "physics.h"

#define m_g 9.8
#define m_pixels_per_meter 50

void update_motion(RigidCircle *rigidCircle, double deltaSeconds) {
    #define M_update(t) \
    double const\
        t##VelocityIncrement = deltaSeconds * rigidCircle->motion.acceleration.t,\
        t##VelocityIncrementHalf = t##VelocityIncrement * 0.5,\
        t##VelocityMiddle = rigidCircle->motion.velocity.t + t##VelocityIncrementHalf,\
        t##Movement = deltaSeconds * t##VelocityMiddle;\
    printf(#t": %.4f, ", rigidCircle->motion.velocity.t);\
    rigidCircle->motion.velocity.t += t##VelocityIncrement;\
    rigidCircle->circle.origin.t += t##Movement;

    M_update(y)
    M_update(x)
    printf("\n");

    #undef M_update
}

void add_gravity_to(RigidCircle *rigidCircle) {
    static double const G_actual_g = m_g * m_pixels_per_meter;
    rigidCircle->motion.acceleration.y += G_actual_g;
}

void collide_circle_with_aabb(RigidCircle *rigidCircle, RigidAABB *rigidAABB) {
    if ((rigidCircle->circle.origin.y + rigidCircle->circle.radius) >
        (rigidAABB->aabb.origin.y - rigidAABB->aabb.halfSize.y)) {
        if (rigidCircle->motion.velocity.y < 0) {
            // Up
            double const
                    distance = rigidCircle->circle.origin.y - rigidAABB->aabb.origin.y,
                    distanceLimit = rigidAABB->aabb.halfSize.y + rigidCircle->circle.radius;
            if (distance < distanceLimit) {
                rigidCircle->motion.velocity.y = -rigidCircle->motion.velocity.y;
            }
        }
    }
    if ((rigidCircle->circle.origin.y - rigidCircle->circle.radius) <
        (rigidAABB->aabb.origin.y + rigidAABB->aabb.halfSize.y)) {
        if (rigidCircle->motion.velocity.y > 0) {
            // Down
            double const
                    distance = rigidAABB->aabb.origin.y - rigidCircle->circle.origin.y,
                    distanceLimit = rigidAABB->aabb.halfSize.y + rigidCircle->circle.radius;
            if (distance < distanceLimit) {
                rigidCircle->motion.velocity.y = -rigidCircle->motion.velocity.y;
            }
        }
    }

    if ((rigidCircle->circle.origin.x - rigidCircle->circle.radius) >
        (rigidAABB->aabb.origin.x + rigidAABB->aabb.halfSize.x)) {
        if (rigidCircle->motion.velocity.x < 0) {
            // Right
            double const
                    distance = rigidCircle->circle.origin.x - rigidAABB->aabb.origin.x,
                    distanceLimit = rigidAABB->aabb.halfSize.x + rigidCircle->circle.radius;
            if (distance < distanceLimit) {
                printf("=======\n");
                rigidCircle->motion.velocity.x = -rigidCircle->motion.velocity.x;
            }
        }
    }
    if ((rigidCircle->circle.origin.x + rigidCircle->circle.radius) <
        (rigidAABB->aabb.origin.x - rigidAABB->aabb.halfSize.x)) {
        if (rigidCircle->motion.velocity.x > 0) {
            // Left
            double const
                    distance = rigidAABB->aabb.origin.x - rigidCircle->circle.origin.x,
                    distanceLimit = rigidAABB->aabb.halfSize.x + rigidCircle->circle.radius;
            if (distance < distanceLimit) {
                printf("------- %f, %f\n", distance, distanceLimit);
                rigidCircle->motion.velocity.x = -rigidCircle->motion.velocity.x;
            }
        }
    }
}

#undef m_g
#undef m_pixels_per_meter
