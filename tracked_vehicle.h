//
//  tracked_vehicle.h
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef TRACKED_VEHICLE_H_INCLUDED
#define TRACKED_VEHICLE_H_INCLUDED

#include <ode/ode.h>
#include "track.h"

typedef struct {
    Track *leftTrack;
    Track *rightTrack;
    dReal density;
    dBodyID vehicleBody;
    dMass vehicleMass;
    dGeomID vehicleGeom;
    dJointID leftTrackJoint;
    dJointID rightTrackJoint;
    dReal width;
} TrackedVehicle;

TrackedVehicle * tracked_vehicle_create(dWorldID world, dSpaceID space, dReal wheelRadius_, dReal wheelBase_, dReal trackWidth_, dReal vehicleWidth_, dReal xOffset, dReal yOffset, dReal zOffset);

void tracked_vehicle_destroy(TrackedVehicle *v);

void tracked_vehicle_draw(TrackedVehicle *v);

#endif // TRACKED_VEHICLE_H_INCLUDED
