//
// Created by qixiaofeng on 2020/8/20.
//

#ifndef QXF_CANVAS_PHYSICS_H
#define QXF_CANVAS_PHYSICS_H

typedef struct p_2D {
    double x;
    double y;
} Point, Vector;

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

void add_gravity_to(RigidCircle *rigidCircle);
void update_motion(RigidCircle *rigidCircle, double deltaSeconds);

#endif //QXF_CANVAS_PHYSICS_H
