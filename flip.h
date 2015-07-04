#ifndef FLIP_H_INCLUDED
#define FLIP_H_INCLUDED

#include <ode/ode.h>
#include "track_kinematic_model.h"
#include "track.h"

typedef struct {
    TrackKinematicModel *m;
    dReal density;
    dBodyID flipBody;
    dBodyID wheel1Body;
    dBodyID wheel2Body;
    dMass flipMass;
    dMass wheel1Mass;
    dMass wheel2Mass;
    dGeomID wheel1Geom;
    dGeomID wheel2Geom;
    dGeomID planeGeom1; // plane geometry
    dGeomID planeGeom2;
    dJointID wheel1Joint;
    dJointID wheel2Joint;
    dBodyID *grouserBody;
    dGeomID *grouserGeom;
    dJointID *grouserJoint;
    dMass *grouserMass;
    dReal xOffset;
    dReal yOffset;
    dReal zOffset;
} Flip;

Flip * flip_init(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserHeight_, dReal flipDepth_, dReal xOffset, dReal yOffset, dReal zOffset);

void flip_create(Flip * f, Track * t, dWorldID world, dSpaceID space);

void flip_destroy(Flip *t);

void flip_deinit(Flip *t);

void flip_draw(Flip *t, int draw_planes);

#endif // FLIP_H_INCLUDED
