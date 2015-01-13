//
//  OMPLTVSStatePropagator.cpp
//  tvs
//
//  Created by Federico Ferri on 03/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "OMPLTVSStatePropagator.h"
#include "OMPLTVSStateSpace.h"
#include "OMPLTVSControlSpace.h"
#include <ompl/util/Exception.h>
#include <ompl/util/Console.h>

OMPLTVSStatePropagator::OMPLTVSStatePropagator(const ompl::control::SpaceInformationPtr &si) : ompl::control::StatePropagator(si) {
    if(OMPLTVSStateSpace *oss = dynamic_cast<OMPLTVSStateSpace*>(si->getStateSpace().get()))
        env_ = oss->getEnvironment();
    else
        throw ompl::Exception("OMPLTVS State Space needed for OMPLTVSStatePropagator");
}

void OMPLTVSStatePropagator::propagate(const ompl::base::State *state, const ompl::control::Control *control, const double duration, ompl::base::State *result) const {
    env_->mutex_.lock();

    // place the OMPLTVS world at the start state
    si_->getStateSpace()->as<OMPLTVSStateSpace>()->writeState(state);

    // apply the controls
    env_->applyControl(control->as<ompl::control::RealVectorControlSpace::ControlType>()->values);

    bool collision = false;
    for(double t = 0; t < duration; t += env_->env_->stepSize) {
        collision = env_->env_->step() || collision;
        if(collision) break;
    }

    // read the final state from the OMPLTVS world
    si_->getStateSpace()->as<OMPLTVSStateSpace>()->readState(result);

#if 1
    const dReal *a = state->as<OMPLTVSStateSpace::StateType>()->getBodyPosition(0);
    const dReal *b = result->as<OMPLTVSStateSpace::StateType>()->getBodyPosition(0);
    std::cout
    << control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0] << " "
    << control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1] << " "
    << "   "
    << duration
    << "   "
    << a[0] << " " << a[1] << " " << a[2]
    << "   "
    << b[0] << " " << b[1] << " " << b[2]
    << std::endl;
#endif
    env_->addToSearchTree(state, result); // for visualization purposes
    
    env_->mutex_.unlock();

    // update the collision flag for the start state, if needed
    if(!(state->as<OMPLTVSStateSpace::StateType>()->collision & (1 << OMPLTVSStateSpace::STATE_COLLISION_KNOWN_BIT))) {
        if(collision)
            state->as<OMPLTVSStateSpace::StateType>()->collision &= (1 << OMPLTVSStateSpace::STATE_COLLISION_VALUE_BIT);
        state->as<OMPLTVSStateSpace::StateType>()->collision &= (1 << OMPLTVSStateSpace::STATE_COLLISION_KNOWN_BIT);
    }
}

bool OMPLTVSStatePropagator::canPropagateBackward() const {
    return false;
}
