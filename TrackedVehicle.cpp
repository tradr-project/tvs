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

TrackedVehicle::TrackedVehicle(const std::string& name_, dReal xOffset, dReal yOffset, dReal zOffset) : name(name_) {
    const dReal wheelRadius = 0.078;
    const dReal wheelBase = 0.4997;
    const dReal trackWidth = 0.097;
    const dReal vehicleBodyWidth = 0.254;
    const size_t numGrousers = 30;
    const dReal linkThickness = 0.01;
    const dReal grouserHeight = 0.015;
    const dReal trackVehicleSpace = 0.024;
    this->density = 1.4;
    this->width = vehicleBodyWidth;
    dReal w = this->width + trackWidth + 2 * trackVehicleSpace;
    this->leftTrack = new Track(name + ".leftTrack", wheelRadius, wheelRadius, wheelBase, numGrousers, linkThickness, grouserHeight, trackWidth, xOffset - wheelBase * 0.5, yOffset - 0.5 * w, zOffset);
    this->rightTrack = new Track(name + ".rightTrack", wheelRadius, wheelRadius, wheelBase, numGrousers, linkThickness, grouserHeight, trackWidth, xOffset - wheelBase * 0.5, yOffset + 0.5 * w, zOffset);
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
    environment->setGeomName(this->vehicleGeom, name + ".vehicleGeom");
    dMassSetBox(&this->vehicleMass, this->density, this->leftTrack->m->distance, this->width, this->leftTrack->m->radius[0]);
    dGeomSetCategoryBits(this->vehicleGeom, Category::OBSTACLE);
    dGeomSetCollideBits(this->vehicleGeom, Category::OBSTACLE | Category::TERRAIN);
    dBodySetMass(this->vehicleBody, &this->vehicleMass);
    dBodySetPosition(this->vehicleBody, this->xOffset, this->yOffset, this->zOffset);
    dGeomSetBody(this->vehicleGeom, this->vehicleBody);
    dGeomSetOffsetPosition(this->vehicleGeom, 0, 0, this->leftTrack->m->radius[0]);
    this->leftTrackJoint = dJointCreateFixed(environment->world, 0);
    this->rightTrackJoint = dJointCreateFixed(environment->world, 0);
    dJointAttach(this->leftTrackJoint, this->vehicleBody, this->leftTrack->trackBody);
    dJointAttach(this->rightTrackJoint, this->vehicleBody, this->rightTrack->trackBody);
    dJointSetFixed(this->leftTrackJoint);
    dJointSetFixed(this->rightTrackJoint);
    
    this->bodyArray = dRigidBodyArrayCreate(this->vehicleBody);
    dRigidBodyArrayAdd(this->bodyArray, this->leftTrack->trackBody);
    dRigidBodyArrayAdd(this->bodyArray, this->rightTrack->trackBody);
    for(int w = 0; w < 2; w++) {
        dRigidBodyArrayAdd(this->bodyArray, this->leftTrack->wheelBody[w]);
        dRigidBodyArrayAdd(this->bodyArray, this->rightTrack->wheelBody[w]);
    }
    for(size_t i = 0; i < this->leftTrack->numGrousers; i++) {
        dRigidBodyArrayAdd(this->bodyArray, this->leftTrack->linkBody[i]);
        dRigidBodyArrayAdd(this->bodyArray, this->rightTrack->linkBody[i]);
    }
}

void TrackedVehicle::destroy() {
    this->leftTrack->destroy();
    this->rightTrack->destroy();

    dBodyDestroy(this->vehicleBody);
    dGeomDestroy(this->vehicleGeom);
    
    dRigidBodyArrayDestroy(this->bodyArray);
}

void TrackedVehicle::step(dReal stepSize) {
    this->leftTrack->step(stepSize);
    this->rightTrack->step(stepSize);
}

void TrackedVehicle::draw() {
    {
        dsSetColor(0, 0, 1);
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

const dReal * TrackedVehicle::getRotation() {
    return dBodyGetRotation(this->vehicleBody);
}

void TrackedVehicle::setPosition(const dReal *p) {
    dRigidBodyArraySetPosition(this->bodyArray, p[0], p[1], p[2]);
}

void TrackedVehicle::setVel(const dReal *linear, const dReal *angular) {
    dRigidBodyArraySetVel(this->bodyArray, linear[0], linear[1], linear[2], angular[0], angular[1], angular[2]);
}

void TrackedVehicle::setQuaternion(const dReal *q) {
    dRigidBodyArraySetQuaternion(this->bodyArray, q);
}

void TrackedVehicle::setRotation(const dReal *R) {
    dRigidBodyArraySetRotation(this->bodyArray, R);
}
