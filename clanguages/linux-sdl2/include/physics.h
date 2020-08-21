//
// Created by qixiaofeng on 2020/8/20.
//

#ifndef QXF_CANVAS_PHYSICS_H
#define QXF_CANVAS_PHYSICS_H

typedef struct p_2D {
    double x;
    double y;
} Point, Vector, Size;

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

void add_gravity_to(RigidCircle *rigidCircle);
void update_motion(RigidCircle *rigidCircle, double deltaSeconds);
void collide_circle_with_aabb(RigidCircle *rigidCircle, AABB const *aabb);
void get_circle_aabb(Circle const *circle, AABB *aabb);

#endif //QXF_CANVAS_PHYSICS_H
