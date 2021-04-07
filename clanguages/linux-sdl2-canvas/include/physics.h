//
// Created by qixiaofeng on 2020/8/20.
//

#ifndef QXF_CANVAS_PHYSICS_H
#define QXF_CANVAS_PHYSICS_H

#include "matrix.h"

typedef M1x2 Point, Vector;

typedef struct p_Motion {
    Vector velocity;
    Vector acceleration;
} Motion;

typedef struct p_Circle {
    Point origin;
    double radius;
} Circle;

typedef struct p_RigidCircle {
    Circle circle;
    Motion motion;
} RigidCircle;

typedef struct p_AABB {
    double left;
    double top;
    double right;
    double bottom;
} AABB;

typedef struct p_RigidAABB {
    AABB aabb;
    Motion motion;
} RigidAABB;

void vector_rotate(Vector *output, Vector const *v, double theta);
void add_gravity_to(RigidCircle *);
void update_motion(RigidCircle *, double);
void get_circle_aabb(AABB *, Circle const *);

void collide_circle_with_aabb(RigidCircle *, AABB const *);
void collide_circle_with_circle(RigidCircle *, Circle const *);
void collide_circles(RigidCircle *, RigidCircle *);

#endif //QXF_CANVAS_PHYSICS_H
