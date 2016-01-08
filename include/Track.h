//
//  Track.h
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef TRACK_H_INCLUDED
#define TRACK_H_INCLUDED

#include <string>
#include <ode/ode.h>
#include "utils.h"
#include "TrackKinematicModel.h"
#include "ODEUtils.h"

class Environment;

#define NUM_GUIDE_GEOMS 2

class Track {
public:
    std::string name;
    TrackKinematicModel *m;
    dReal density;
    dBodyID trackBody;
    dMass trackMass;
    dBodyID wheelBody[2];
    dMass wheelMass[2];
    dGeomID wheelGeom[2];
    dJointID wheelJoint[2];
    dGeomID guideGeom[NUM_GUIDE_GEOMS];
    dBodyID *linkBody;
    dGeomID *linkGeom;
    dGeomID *grouserGeom;
    dJointID *linkJoint;
    dMass *linkMass;
    dReal xOffset;
    dReal yOffset;
    dReal zOffset;
    size_t numGrousers;
    dReal linkThickness;
    dReal grouserHeight;
    LinVelProfInt velocity;
    dRigidBodyArrayID bodyArray;

    Track(const std::string& name_, dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal linkThickness_, dReal grouserHeight_, dReal trackDepth_, dReal xOffset, dReal yOffset, dReal zOffset);
    virtual ~Track();
    void create(Environment *environment);
    void destroy();
    void step(dReal stepSize);
    void draw();
    void setVelocity(dReal velocity);
};

#endif // TRACK_H_INCLUDED
