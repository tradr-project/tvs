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
    
    addSubspace(ob::StateSpacePtr(new ob::SO3StateSpace()), 1.0);
    addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 1.0);
    lock();
}

OMPLStateSpace::~OMPLStateSpace() {
}

/*
double OMPLStateSpace::distance(const ob::State *s1, const ob::State *s2) const {
    const double *p1 = s1->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
    const double *p2 = s2->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
    double dx = fabs(p1[0] - p2[0]);
    double dy = fabs(p1[1] - p2[1]);
    double dz = fabs(p1[2] - p2[2]);
    return sqrt(dx * dx + dy * dy + dz * dz);
}

void OMPLStateSpace::registerProjections() {
    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new OMPLStateProjectionEvaluator(this)));
}
*/
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
