//
// Created by qixiaofeng on 2020/8/20.
//

#include "physics.h"

#define m_g 9.8
#define m_pixels_per_meter 50

double const G_actual_g = m_g * m_pixels_per_meter;

void update_motion(RigidCircle *rigidCircle, double deltaSeconds) {
    double const yVelocityIncrement = deltaSeconds * rigidCircle->motion.acceleration.y;
    double const yVelocityIncrementHalf = yVelocityIncrement * 0.5;
    double const yVelocityMiddle = rigidCircle->motion.velocity.y + yVelocityIncrementHalf;
    double const yMovement = deltaSeconds * yVelocityMiddle;
    rigidCircle->motion.velocity.y += yVelocityIncrement;
    rigidCircle->circle.origin.y += yMovement;
}

void add_gravity_to(RigidCircle *rigidCircle) {
    rigidCircle->motion.acceleration.y += G_actual_g;
}

#undef m_g
#undef m_pixels_per_meter
