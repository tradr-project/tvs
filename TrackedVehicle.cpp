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
#include "ODEUtils.h"

TrackedVehicle::TrackedVehicle(dReal wheelRadius_, dReal wheelBase_, dReal trackWidth_, dReal vehicleWidth_, dReal xOffset, dReal yOffset, dReal zOffset) {
    this->density = 1.0;
    this->width = vehicleWidth_;
    const size_t numGrousers = 30;
    const dReal grouserHeight = 0.01;
    dReal w = this->width + 2 * trackWidth_;
    this->leftTrack = new Track(wheelRadius_, wheelRadius_, wheelBase_, numGrousers, grouserHeight, trackWidth_, xOffset - wheelBase_ * 0.5, yOffset - 0.5 * w, zOffset);
    this->rightTrack = new Track(wheelRadius_, wheelRadius_, wheelBase_, numGrousers, grouserHeight, trackWidth_, xOffset - wheelBase_ * 0.5, yOffset + 0.5 * w, zOffset);
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
    this->vehicleGeom = dCreateBox(environment->space, this->leftTrack->m->distance, this->width, this->leftTrack->m->radius[0]);
    dMassSetBox(&this->vehicleMass, this->density, this->leftTrack->m->distance, this->width, this->leftTrack->m->radius[0]);
    dGeomSetCategoryBits(this->vehicleGeom, Category::OBSTACLE);
    dGeomSetCollideBits(this->vehicleGeom, Category::OBSTACLE | Category::TERRAIN);
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
    for(int w = 0; w < 2; w++) {
        this->bodyArray[j++] = this->leftTrack->wheelBody[w];
        this->bodyArray[j++] = this->rightTrack->wheelBody[w];
    }
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

void TrackedVehicle::setTrackVelocities(dReal left, dReal right) {
    this->leftTrack->setVelocity(left);
    this->rightTrack->setVelocity(right);
}

const dReal * TrackedVehicle::getPosition() {
    return dBodyGetPosition(this->vehicleBody);
}

const dReal * TrackedVehicle::getLinearVel() {
    return dBodyGetLinearVel(this->vehicleBody);
}

const dReal * TrackedVehicle::getAngularVel() {
    return dBodyGetAngularVel(this->vehicleBody);
}

const dReal * TrackedVehicle::getQuaternion() {
    return dBodyGetQuaternion(this->vehicleBody);
}

void TrackedVehicle::setPosition(const dReal *p) {
    dRigidBodyArraySetPosition(this->bodyArray, this->bodyArraySize, this->vehicleBody, p[0], p[1], p[2]);
}

void TrackedVehicle::setVel(const dReal *linear, const dReal *angular) {
    dRigidBodyArraySetVel(this->bodyArray, this->bodyArraySize, this->vehicleBody, linear[0], linear[1], linear[2], angular[0], angular[1], angular[2]);
}

void TrackedVehicle::setQuaternion(const dReal *q) {
    dRigidBodyArraySetQuaternion(this->bodyArray, this->bodyArraySize, this->vehicleBody, q);
}
