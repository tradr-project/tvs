//
//  TrackedVehicle.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Environment.h"
#include "TrackedVehicle.h"
#include <cstdio>
#include <cstdlib>
#include <cassert>
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

void TrackedVehicle::create(Environment *environment) {
    this->leftTrack->create(environment);
    this->rightTrack->create(environment);
    this->vehicleBody = dBodyCreate(environment->world);
    this->vehicleGeom = dCreateBox(environment->space, this->leftTrack->m->distance, this->width, this->leftTrack->m->radius1);
    dMassSetBox(&this->vehicleMass, this->density, this->leftTrack->m->distance, this->width, this->leftTrack->m->radius1);
    dGeomSetCategoryBits(this->vehicleGeom, 0x0);
    dGeomSetCollideBits(this->vehicleGeom, 0x0);
    dBodySetMass(this->vehicleBody, &this->vehicleMass);
    dBodySetPosition(this->vehicleBody, this->xOffset, this->yOffset, this->zOffset);
    dGeomSetBody(this->vehicleGeom, this->vehicleBody);
    this->leftTrackJoint = dJointCreateFixed(environment->world, 0);
    this->rightTrackJoint = dJointCreateFixed(environment->world, 0);
    dJointAttach(this->leftTrackJoint, this->vehicleBody, this->leftTrack->trackBody);
    dJointAttach(this->rightTrackJoint, this->vehicleBody, this->rightTrack->trackBody);
    dJointSetFixed(this->leftTrackJoint);
    dJointSetFixed(this->rightTrackJoint);
    
    this->bodyArraySize = (this->leftTrack->numGrousers + 1 + 2) * 2 + 1;
    this->bodyArray = new dBodyID[this->bodyArraySize];
    size_t j = 0;
    this->bodyArray[j++] = this->vehicleBody;
    this->bodyArray[j++] = this->leftTrack->trackBody;
    this->bodyArray[j++] = this->rightTrack->trackBody;
    this->bodyArray[j++] = this->leftTrack->wheel1Body;
    this->bodyArray[j++] = this->leftTrack->wheel2Body;
    this->bodyArray[j++] = this->rightTrack->wheel1Body;
    this->bodyArray[j++] = this->rightTrack->wheel2Body;
    for(size_t i = 0; i < this->leftTrack->numGrousers; i++) {
        this->bodyArray[j++] = this->leftTrack->grouserBody[i];
        this->bodyArray[j++] = this->rightTrack->grouserBody[i];
    }
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

