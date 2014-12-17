//
//  TrackedVehicle.h
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef TRACKED_VEHICLE_H_INCLUDED
#define TRACKED_VEHICLE_H_INCLUDED

#include <ode/ode.h>
#include "Track.h"

class World;

class TrackedVehicle {
public:
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
    
    TrackedVehicle(dReal wheelRadius_, dReal wheelBase_, dReal trackWidth_, dReal vehicleWidth_, dReal xOffset, dReal yOffset, dReal zOffset);
    virtual ~TrackedVehicle();
    void create(World *world);
    void destroy();
    void draw();
};

#endif // TRACKED_VEHICLE_H_INCLUDED
