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
    for(double t = 0; t < duration; t += env_->env_->config.step.step_size) {
        collision = env_->env_->step() || collision;
        if(collision) {
            std::cout << "propagate: collision" << std::endl;
            break;
        }
    }

    // read the final state from the OMPLTVS world
    si_->getStateSpace()->as<OMPLTVSStateSpace>()->readState(result);

    env_->addToSearchTree(state, control, duration, result); // for visualization purposes
    
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
