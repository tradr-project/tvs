//
//  OMPLEnvironment.cpp
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "OMPLEnvironment.h"

OMPLEnvironment::OMPLEnvironment() {
    env = new Environment();
    env->create();
}

OMPLEnvironment::~OMPLEnvironment() {
    env->destroy();
    delete env;
}

unsigned int OMPLEnvironment::getControlDimension() const {
    return 2;
}

void OMPLEnvironment::getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const {
    static double maxVelocity = 1.1;
    
    lower.resize(2);
    lower[0] = -maxVelocity;
    lower[1] = -maxVelocity;
    
    upper.resize(2);
    upper[0] = maxVelocity;
    upper[1] = maxVelocity;
}

void OMPLEnvironment::applyControl(const double *control) const {
    env->v->setTrackVelocities(control[0], control[1]);
}

bool OMPLEnvironment::isValidCollision(dGeomID g1, dGeomID g2, const dContact& contact) const {
    unsigned long c1 = dGeomGetCategoryBits(g1);
    unsigned long c2 = dGeomGetCategoryBits(g2);
    if((c1 & Category::GROUSER) && (c2 & Category::TERRAIN)) return true;
    if((c2 & Category::GROUSER) && (c1 & Category::TERRAIN)) return true;
    if((c1 & Category::WHEEL) && (c2 & Category::GROUSER)) return true;
    if((c2 & Category::WHEEL) && (c1 & Category::GROUSER)) return true;
    return false;
}
