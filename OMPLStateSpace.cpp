//
//  OMPLStateSpace.cpp
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "OMPLStateSpace.h"
#include "OMPLStateProjectionEvaluator.h"

OMPLStateSpace::OMPLStateSpace(const OMPLEnvironmentPtr &env) {
    this->env = env;

    // pose:
    ob::SE3StateSpace *poseSpace = new ob::SE3StateSpace();
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-5.0);
    bounds.setHigh(5.0);
    poseSpace->setBounds(bounds);
    
    // tracks velocity:
    ob::RealVectorStateSpace *tracksVelSpace = new ob::RealVectorStateSpace(2);
    tracksVelSpace->setBounds(-3, 3);
    
    addSubspace(ob::StateSpacePtr(poseSpace), 1.0);
    addSubspace(ob::StateSpacePtr(tracksVelSpace), 1.0);
    lock();
    
    //sanityChecks();
    
    registerProjections();
}

OMPLStateSpace::~OMPLStateSpace() {
}

void OMPLStateSpace::registerProjections() {
    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new OMPLStateProjectionEvaluator(this)));
}

double OMPLStateSpace::distance(const ob::State *s1, const ob::State *s2) const {
    double dx = fabs(s1->as<ob::RealVectorStateSpace::StateType>()->values[0]
                     - s2->as<ob::RealVectorStateSpace::StateType>()->values[0]);
    double dy = fabs(s1->as<ob::RealVectorStateSpace::StateType>()->values[1]
                     - s2->as<ob::RealVectorStateSpace::StateType>()->values[1]);
    double dz = fabs(s1->as<ob::RealVectorStateSpace::StateType>()->values[2]
                     - s2->as<ob::RealVectorStateSpace::StateType>()->values[2]);
    return sqrt(dx * dx + dy * dy + dz * dz);
}

/*
bool OMPLStateSpace::evaluateCollision(const ob::State *state) const {
    if (state->as<StateType>()->collision & (1 << STATE_COLLISION_KNOWN_BIT))
        return state->as<StateType>()->collision & (1 << STATE_COLLISION_VALUE_BIT);
    env_->mutex_.lock();
    writeState(state);
    CallbackParam cp = { env_.get(), false };
    for (unsigned int i = 0 ; cp.collision == false && i < env_->collisionSpaces_.size() ; ++i)
        dSpaceCollide(env_->collisionSpaces_[i], &cp, &nearCallback);
    env_->mutex_.unlock();
    if (cp.collision)
        state->as<StateType>()->collision &= (1 << STATE_COLLISION_VALUE_BIT);
    state->as<StateType>()->collision &= (1 << STATE_COLLISION_KNOWN_BIT);
    return cp.collision;
}
 */
