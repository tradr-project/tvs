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

TrackedVehicle::TrackedVehicle(const std::string& name_, dReal xOffset, dReal yOffset, dReal zOffset) : Vehicle(name_) {
    wheelRadius = 0.078;
    wheelBase = 0.4997;
    trackWidth = 0.097;
    vehicleBodyWidth = 0.254;
    const size_t numGrousers = 30;
    const dReal linkThickness = 0.01;
    const dReal grouserHeight = 0.015;
    trackVehicleSpace = 0.024;
    flipperRadius = 0.016;
    flipperBase = 0.32;
    flipperWidth = 0.05;

    this->density = 1.4;
    this->width = vehicleBodyWidth;
    dReal w = this->width + 2*trackWidth + 2 * trackVehicleSpace;
    this->leftTrack = new Track(name + ".leftTrack", wheelRadius, wheelRadius, wheelBase, numGrousers, linkThickness, grouserHeight, trackWidth, xOffset - wheelBase * 0.5, yOffset - 0.5 * w + trackWidth/2.0, zOffset);
    this->rightTrack = new Track(name + ".rightTrack", wheelRadius, wheelRadius, wheelBase, numGrousers, linkThickness, grouserHeight, trackWidth, xOffset - wheelBase * 0.5, yOffset + 0.5 * w - trackWidth/2.0, zOffset);
    this->frontLeftFlipper = new Track(name + ".frontLeftFlipper", wheelRadius, flipperRadius, flipperBase, numGrousers, linkThickness, grouserHeight, flipperWidth, xOffset - wheelBase/2.0, yOffset - 0.5 * w - flipperWidth/2.0, zOffset);
    this->frontRightFlipper = new Track(name + ".frontRightFlipper", wheelRadius, flipperRadius, flipperBase, numGrousers, linkThickness, grouserHeight, flipperWidth, xOffset - wheelBase/2.0, yOffset + 0.5 * w + flipperWidth/2.0, zOffset);
    this->rearLeftFlipper = new Track(name + ".rearLeftFlipper", wheelRadius, flipperRadius, flipperBase, numGrousers, linkThickness, grouserHeight, flipperWidth, xOffset + wheelBase/2.0, yOffset - 0.5 * w - flipperWidth/2.0, zOffset);
    this->rearRightFlipper = new Track(name + ".rearRightFlipper", wheelRadius, flipperRadius, flipperBase, numGrousers, linkThickness, grouserHeight, flipperWidth, xOffset + wheelBase/2.0, yOffset + 0.5 * w + flipperWidth/2.0, zOffset);
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

    this->frontLeftFlipper->create(environment);
    this->frontRightFlipper->create(environment);
    this->rearLeftFlipper->create(environment);
    this->rearRightFlipper->create(environment);

    // rotate the front flippers by 180 degrees
    dMatrix3 rot;
    dRFromAxisAndAngle(rot, 0, 1, 0, M_PI);
    dRigidBodyArraySetRotationRelative(this->frontLeftFlipper->bodyArray, rot);
    dRigidBodyArraySetRotationRelative(this->frontRightFlipper->bodyArray, rot);

    this->vehicleBody = dBodyCreate(environment->world);
    this->vehicleGeom = dCreateBox(environment->space, this->leftTrack->m->distance, this->width, this->leftTrack->m->radius[0]);
    environment->setGeomName(this->vehicleGeom, name + ".vehicleGeom");
    dMassSetBox(&this->vehicleMass, this->density, this->leftTrack->m->distance, this->width, this->leftTrack->m->radius[0]);
    //dMassAdjust(&this->vehicleMass, 2.40);
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

    this->frontLeftFlipperJoint = dJointCreateHinge(environment->world, 0);
    this->frontRightFlipperJoint = dJointCreateHinge(environment->world, 0);
    this->rearLeftFlipperJoint = dJointCreateHinge(environment->world, 0);
    this->rearRightFlipperJoint = dJointCreateHinge(environment->world, 0);
    dJointAttach(this->frontLeftFlipperJoint, this->leftTrack->trackBody, this->frontLeftFlipper->trackBody);
    dJointAttach(this->frontRightFlipperJoint, this->rightTrack->trackBody, this->frontRightFlipper->trackBody);
    dJointAttach(this->rearLeftFlipperJoint, this->leftTrack->trackBody, this->rearLeftFlipper->trackBody);
    dJointAttach(this->rearRightFlipperJoint, this->rightTrack->trackBody, this->rearRightFlipper->trackBody);
    dJointSetHingeAnchor(this->frontLeftFlipperJoint, xOffset + wheelBase/2.0, yOffset - (vehicleBodyWidth/2 + trackWidth + trackVehicleSpace), zOffset);
    dJointSetHingeAnchor(this->frontRightFlipperJoint, xOffset + wheelBase/2.0, yOffset + vehicleBodyWidth/2 + trackWidth + trackVehicleSpace, zOffset);
    dJointSetHingeAnchor(this->rearLeftFlipperJoint, xOffset - wheelBase/2.0, yOffset - (vehicleBodyWidth/2 + trackWidth + trackVehicleSpace), zOffset);
    dJointSetHingeAnchor(this->rearRightFlipperJoint, xOffset - wheelBase/2.0, yOffset + vehicleBodyWidth/2 + trackWidth + trackVehicleSpace, zOffset);
    dJointSetHingeAxis(this->frontLeftFlipperJoint, 0, 1, 0);
    dJointSetHingeAxis(this->frontRightFlipperJoint, 0, 1, 0);
    dJointSetHingeAxis(this->rearLeftFlipperJoint, 0, 1, 0);
    dJointSetHingeAxis(this->rearRightFlipperJoint, 0, 1, 0);

    this->bodyArray = dRigidBodyArrayCreate(this->vehicleBody);
    dRigidBodyArrayAdd(this->bodyArray, this->leftTrack->trackBody);
    dRigidBodyArrayAdd(this->bodyArray, this->rightTrack->trackBody);
    dRigidBodyArrayAdd(this->bodyArray, this->frontLeftFlipper->trackBody);
    dRigidBodyArrayAdd(this->bodyArray, this->frontRightFlipper->trackBody);
    dRigidBodyArrayAdd(this->bodyArray, this->rearLeftFlipper->trackBody);
    dRigidBodyArrayAdd(this->bodyArray, this->rearRightFlipper->trackBody);
    for(int w = 0; w < 2; w++) {
        dRigidBodyArrayAdd(this->bodyArray, this->leftTrack->wheelBody[w]);
        dRigidBodyArrayAdd(this->bodyArray, this->rightTrack->wheelBody[w]);
        dRigidBodyArrayAdd(this->bodyArray, this->frontLeftFlipper->wheelBody[w]);
        dRigidBodyArrayAdd(this->bodyArray, this->frontRightFlipper->wheelBody[w]);
        dRigidBodyArrayAdd(this->bodyArray, this->rearLeftFlipper->wheelBody[w]);
        dRigidBodyArrayAdd(this->bodyArray, this->rearRightFlipper->wheelBody[w]);
    }
    for(size_t i = 0; i < this->leftTrack->numGrousers; i++) {
        dRigidBodyArrayAdd(this->bodyArray, this->leftTrack->linkBody[i]);
        dRigidBodyArrayAdd(this->bodyArray, this->rightTrack->linkBody[i]);
        dRigidBodyArrayAdd(this->bodyArray, this->frontLeftFlipper->linkBody[i]);
        dRigidBodyArrayAdd(this->bodyArray, this->frontRightFlipper->linkBody[i]);
        dRigidBodyArrayAdd(this->bodyArray, this->rearLeftFlipper->linkBody[i]);
        dRigidBodyArrayAdd(this->bodyArray, this->rearRightFlipper->linkBody[i]);
    }

}

void TrackedVehicle::destroy() {
    this->leftTrack->destroy();
    dJointDestroy(this->leftTrackJoint);
    this->rightTrack->destroy();
    dJointDestroy(this->rightTrackJoint);

    this->frontLeftFlipper->destroy();
    dJointDestroy(this->frontLeftFlipperJoint);
    this->frontRightFlipper->destroy();
    dJointDestroy(this->frontRightFlipperJoint);
    this->rearLeftFlipper->destroy();
    dJointDestroy(this->rearLeftFlipperJoint);
    this->rearRightFlipper->destroy();
    dJointDestroy(this->rearRightFlipperJoint);

    dBodyDestroy(this->vehicleBody);
    dGeomDestroy(this->vehicleGeom);
    
    dRigidBodyArrayDestroy(this->bodyArray);
}

void TrackedVehicle::step(dReal stepSize) {
    this->leftTrack->step(stepSize);
    this->rightTrack->step(stepSize);

    this->frontLeftFlipper->step(stepSize);
    this->frontRightFlipper->step(stepSize);
    this->rearLeftFlipper->step(stepSize);
    this->rearRightFlipper->step(stepSize);
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

    this->frontLeftFlipper->draw();
    this->frontRightFlipper->draw();
    this->rearLeftFlipper->draw();
    this->rearRightFlipper->draw();
}

void TrackedVehicle::setVelocities(dReal a, dReal b) {
    setTrackVelocities(a, b);
}

void TrackedVehicle::setTrackVelocities(dReal left, dReal right) {
    this->leftTrack->setVelocity(left);
    this->rightTrack->setVelocity(right);

    this->frontLeftFlipper->setVelocity(left);
    this->frontRightFlipper->setVelocity(right);
    this->rearLeftFlipper->setVelocity(left);
    this->rearRightFlipper->setVelocity(right);
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
