//
//  TrackedVehicle.h
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef TRACKED_VEHICLE_H_INCLUDED
#define TRACKED_VEHICLE_H_INCLUDED

#include <string>
#include <ode/ode.h>
#include "Track.h"

class Environment;

class TrackedVehicle {
public:
    std::string name;
    Track *leftTrack;
    Track *rightTrack;
    dReal density;
    dBodyID vehicleBody;
    dMass vehicleMass;
    dGeomID vehicleGeom;
    dJointID leftTrackJoint;
    dJointID rightTrackJoint;
    dReal width;
    dReal xOffset;
    dReal yOffset;
    dReal zOffset;
    
    dBodyID *bodyArray;
    size_t bodyArraySize;
    
    TrackedVehicle(const std::string& name_, dReal wheelRadius_, dReal wheelBase_, dReal trackWidth_, dReal vehicleWidth_, dReal xOffset, dReal yOffset, dReal zOffset);
    virtual ~TrackedVehicle();
    void create(Environment *environment);
    void destroy();
    void draw();
    void setTrackVelocities(dReal left, dReal right);
    const dReal * getPosition();
    const dReal * getLinearVel();
    const dReal * getAngularVel();
    const dReal * getQuaternion();
    void setPosition(const dReal *p);
    void setVel(const dReal *linear, const dReal *angular);
    void setQuaternion(const dReal *q);
};

#endif // TRACKED_VEHICLE_H_INCLUDED
