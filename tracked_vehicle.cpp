//
//  tracked_vehicle.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "world.h"
#include "tracked_vehicle.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <drawstuff/drawstuff.h>

TrackedVehicle::TrackedVehicle(dReal wheelRadius_, dReal wheelBase_, dReal trackWidth_, dReal vehicleWidth_, dReal xOffset, dReal yOffset, dReal zOffset) {
    this->density = 1.0;
    this->width = vehicleWidth_;
    const size_t numGrousers = 30;
    const dReal grouserHeight = 0.01;
    dReal w = this->width + 2 * trackWidth_;
    this->leftTrack = new Track(wheelRadius_, wheelRadius_, wheelBase_, numGrousers, grouserHeight, trackWidth_, xOffset, yOffset - 0.5 * w, zOffset);
    this->rightTrack = new Track(wheelRadius_, wheelRadius_, wheelBase_, numGrousers, grouserHeight, trackWidth_, xOffset, yOffset + 0.5 * w, zOffset);
    this->xOffset = xOffset;
    this->yOffset = yOffset;
    this->zOffset = zOffset;
}

TrackedVehicle::~TrackedVehicle() {
    delete this->leftTrack;
    delete this->rightTrack;
}

void TrackedVehicle::create(World *world) {
    this->leftTrack->create(world);
    this->rightTrack->create(world);
    this->vehicleBody = dBodyCreate(world->world);
    this->vehicleGeom = dCreateBox(world->space, this->leftTrack->m->distance, this->width, this->leftTrack->m->radius1);
    dMassSetBox(&this->vehicleMass, this->density, this->leftTrack->m->distance, this->width, this->leftTrack->m->radius1);
    dGeomSetCategoryBits(this->vehicleGeom, 0x0);
    dGeomSetCollideBits(this->vehicleGeom, 0x0);
    dBodySetMass(this->vehicleBody, &this->vehicleMass);
    dBodySetPosition(this->vehicleBody, this->xOffset, this->yOffset, this->zOffset);
    dGeomSetBody(this->vehicleGeom, this->vehicleBody);
    this->leftTrackJoint = dJointCreateFixed(world->world, 0);
    this->rightTrackJoint = dJointCreateFixed(world->world, 0);
    dJointAttach(this->leftTrackJoint, this->vehicleBody, this->leftTrack->trackBody);
    dJointAttach(this->rightTrackJoint, this->vehicleBody, this->rightTrack->trackBody);
    dJointSetFixed(this->leftTrackJoint);
    dJointSetFixed(this->rightTrackJoint);
}

void TrackedVehicle::destroy() {
    this->leftTrack->destroy();
    this->rightTrack->destroy();
}

void TrackedVehicle::draw() {
    {
        const dReal *pos = dGeomGetPosition(this->vehicleGeom);
        const dReal *R = dGeomGetRotation(this->vehicleGeom);
        dReal sides[3];
        dGeomBoxGetLengths(this->vehicleGeom, sides);
        dsDrawBoxD(pos, R, sides);
    }

    this->leftTrack->draw();
    this->rightTrack->draw();
}

