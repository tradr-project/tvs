//
//  tracked_vehicle.c
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "tracked_vehicle.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <drawstuff/drawstuff.h>

TrackedVehicle * tracked_vehicle_create(dWorldID world, dSpaceID space, dReal wheelRadius_, dReal wheelBase_, dReal trackWidth_, dReal vehicleWidth_, dReal xOffset, dReal yOffset, dReal zOffset) {
    TrackedVehicle *v = (TrackedVehicle *)malloc(sizeof(TrackedVehicle));
    v->density = 1.0;
    v->width = vehicleWidth_;
    const size_t numGrousers = 30;
    const dReal grouserHeight = 0.01;
    dReal w = v->width + 2 * trackWidth_;
    v->leftTrack = track_create(world, space, wheelRadius_, wheelRadius_, wheelBase_, numGrousers, grouserHeight, trackWidth_, xOffset, yOffset - 0.5 * w, zOffset);
    v->rightTrack = track_create(world, space, wheelRadius_, wheelRadius_, wheelBase_, numGrousers, grouserHeight, trackWidth_, xOffset, yOffset + 0.5 * w, zOffset);
    v->vehicleBody = dBodyCreate(world);
    v->vehicleGeom = dCreateBox(space, v->leftTrack->m->distance, v->width, v->leftTrack->m->radius1);
    dMassSetBox(&v->vehicleMass, v->density, v->leftTrack->m->distance, v->width, v->leftTrack->m->radius1);
    dGeomSetCategoryBits(v->vehicleGeom, 0x0);
    dGeomSetCollideBits(v->vehicleGeom, 0x0);
    dBodySetMass(v->vehicleBody, &v->vehicleMass);
    dBodySetPosition(v->vehicleBody, xOffset, yOffset, zOffset);
    dGeomSetBody(v->vehicleGeom, v->vehicleBody);
    v->leftTrackJoint = dJointCreateFixed(world, 0);
    v->rightTrackJoint = dJointCreateFixed(world, 0);
    dJointAttach(v->leftTrackJoint, v->vehicleBody, v->leftTrack->trackBody);
    dJointAttach(v->rightTrackJoint, v->vehicleBody, v->rightTrack->trackBody);
    dJointSetFixed(v->leftTrackJoint);
    dJointSetFixed(v->rightTrackJoint);
    return v;
}

void tracked_vehicle_destroy(TrackedVehicle *v) {
    track_destroy(v->leftTrack);
    track_destroy(v->rightTrack);
    free(v);
}

void tracked_vehicle_draw(TrackedVehicle *v) {
    {
        const dReal *pos = dGeomGetPosition(v->vehicleGeom);
        const dReal *R = dGeomGetRotation(v->vehicleGeom);
        dReal sides[3];
        dGeomBoxGetLengths(v->vehicleGeom, sides);
        dsDrawBoxD(pos, R, sides);
    }

    track_draw(v->leftTrack);
    track_draw(v->rightTrack);
}

