//
//  Track.h
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef TRACK_H_INCLUDED
#define TRACK_H_INCLUDED

#include <ode/ode.h>
#include "TrackKinematicModel.h"

class Environment;

class Track {
public:
    TrackKinematicModel *m;
    dReal density;
    dBodyID trackBody;
    dBodyID wheel1Body;
    dBodyID wheel2Body;
    dMass trackMass;
    dMass wheel1Mass;
    dMass wheel2Mass;
    dGeomID wheel1Geom;
    dGeomID wheel2Geom;
    dJointID wheel1Joint;
    dJointID wheel2Joint;
    //dJointID guideJoint;
    dBodyID *grouserBody;
    dGeomID *grouserGeom;
    dJointID *grouserJoint;
    dMass *grouserMass;
    dReal xOffset;
    dReal yOffset;
    dReal zOffset;
    size_t numGrousers;

    Track(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserHeight_, dReal trackDepth_, dReal xOffset, dReal yOffset, dReal zOffset);
    virtual ~Track();
    void create(Environment *environment);
    void destroy();
    void draw();
};

#endif // TRACK_H_INCLUDED
