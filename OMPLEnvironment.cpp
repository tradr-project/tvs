//
//  OMPLEnvironment.cpp
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "OMPLEnvironment.h"

OMPLEnvironment::OMPLEnvironment() : oc::OpenDEEnvironment() {
    createWorld();
}

OMPLEnvironment::~OMPLEnvironment() {
    destroyWorld();
}

unsigned int OMPLEnvironment::getControlDimension() const {
    return 3;
}

void OMPLEnvironment::getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const {
    static double maxForce = 0.2;
    
    lower.resize(3);
    lower[0] = -maxForce;
    lower[1] = -maxForce;
    lower[2] = -maxForce;
    
    upper.resize(3);
    upper[0] = maxForce;
    upper[1] = maxForce;
    upper[2] = maxForce;
}

void OMPLEnvironment::applyControl(const double *control) const {
    dBodyAddForce(boxBody, control[0], control[1], control[2]);
}

bool OMPLEnvironment::isValidCollision(dGeomID g1, dGeomID g2, const dContact& contact) const {
    return false;
}

void OMPLEnvironment::setupContact(dGeomID g1, dGeomID g2, dContact &contact) const {
    contact.surface.mode = dContactSoftCFM | dContactApprox1;
    contact.surface.mu = 0.9;
    contact.surface.soft_cfm = 0.2;
}

void OMPLEnvironment::createWorld() {
    // BEGIN SETTING UP AN OPENDE ENVIRONMENT
    // ***********************************
    
    bodyWorld = dWorldCreate();
    space = dHashSpaceCreate(0);
    
    dWorldSetGravity(bodyWorld, 0, 0, -0.981);
    
    double lx = 0.2;
    double ly = 0.2;
    double lz = 0.1;
    
    dMassSetBox(&m, 1, lx, ly, lz);
    
    boxGeom = dCreateBox(space, lx, ly, lz);
    boxBody = dBodyCreate(bodyWorld);
    dBodySetMass(boxBody, &m);
    dGeomSetBody(boxGeom, boxBody);
    
    // *********************************
    // END SETTING UP AN OPENDE ENVIRONMENT
    
    setPlanningParameters();
}

void OMPLEnvironment::destroyWorld() {
    dSpaceDestroy(space);
    dWorldDestroy(bodyWorld);
}

void OMPLEnvironment::setPlanningParameters() {
    // Fill in parameters for OMPL:
    world_ = bodyWorld;
    collisionSpaces_.push_back(space);
    stateBodies_.push_back(boxBody);
    stepSize_ = 0.05;
    maxContacts_ = 3;
    minControlSteps_ = 10;
    maxControlSteps_ = 500;
}
