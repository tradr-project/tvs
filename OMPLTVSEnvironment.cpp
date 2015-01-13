//
//  OMPLTVSEnvironment.cpp
//  tvs
//
//  Created by Federico Ferri on 03/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "OMPLTVSEnvironment.h"
#include "OMPLTVSStateSpace.h"
#include <boost/lexical_cast.hpp>

OMPLTVSEnvironment::OMPLTVSEnvironment(Environment *env)
: stepSize_(0.5), maxControlSteps_(5), minControlSteps_(1), env_(env) {
}

OMPLTVSEnvironment::~OMPLTVSEnvironment() {
}

unsigned int OMPLTVSEnvironment::getControlDimension(void) const {
    return 2;
}

void OMPLTVSEnvironment::getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const {
    static double maxVel = 5.0;
    lower.resize(2);
    lower[0] = -maxVel;
    lower[1] = -maxVel;
    
    upper.resize(2);
    upper[0] = maxVel;
    upper[1] = maxVel;
}

void OMPLTVSEnvironment::applyControl(const double *control) const {
    env_->v->setTrackVelocities(control[0], control[1]);
}

void OMPLTVSEnvironment::addToSearchTree(const ompl::base::State *s1, const ompl::base::State *s2) {
    searchTree.resize(searchTree.size() + 1);
    memcpy(searchTree.back().a, s1->as<OMPLTVSStateSpace::StateType>()->getBodyPosition(0), sizeof(dReal) * 3);
    memcpy(searchTree.back().b, s2->as<OMPLTVSStateSpace::StateType>()->getBodyPosition(0), sizeof(dReal) * 3);
}
