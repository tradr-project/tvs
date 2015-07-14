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
#include "flip.h"
#include "backflip.h"

typedef struct {
    Track *leftTrack;
    Track *rightTrack;
    Flip *leftFlip; // aggiunto
    Flip *rightFlip; // aggiunto
    BackFlip *leftBackFlip;
    BackFlip *rightBackFlip;
    dReal density;
    dBodyID vehicleBody;
    dMass vehicleMass;
    dGeomID vehicleGeom;

    dJointID leftTrackJoint;
    dJointID rightTrackJoint;
    dJointID leftFlipJoint;
    dJointID rightFlipJoint;
    dJointID leftBackFlipJoint;
    dJointID rightBackFlipJoint;

    dReal width;
    dReal xOffset;
    dReal yOffset;
    dReal zOffset;
} TrackedVehicle;

TrackedVehicle * tracked_vehicle_init(dReal wheelRadius_, dReal wheelBase_, dReal flipWheelRadius, dReal flipWheelBase, dReal trackWidth_, dReal flipWidth_, dReal vehicleWidth_, dReal xOffset, dReal yOffset, dReal zOffset);

void tracked_vehicle_create(TrackedVehicle *v, dWorldID world, dSpaceID space);

void tracked_vehicle_destroy(TrackedVehicle *v);

void tracked_vehicle_deinit(TrackedVehicle *v);

void tracked_vehicle_draw(TrackedVehicle *v);

void setFlipAngle(dJointID flipjoint,dReal DesiredPosition);

#endif // TRACKED_VEHICLE_H_INCLUDED
