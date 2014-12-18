//
//  OMPLStatePropagator.cpp
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "OMPLStatePropagator.h"
#include "OMPLStateSpace.h"
#include <ompl/util/Exception.h>

OMPLStatePropagator::OMPLStatePropagator(const oc::SpaceInformationPtr &si) : oc::OpenDEStatePropagator(si) {
    OMPLStateSpace *oss = dynamic_cast<OMPLStateSpace *>(si->getStateSpace().get());
    if(!oss)
        throw ompl::Exception("OMPLStateSpace needed for OMPLStatePropagator");
}

OMPLStatePropagator::~OMPLStatePropagator() {
}

void OMPLStatePropagator::propagate(const ob::State *state, const oc::Control *control, const double duration, ob::State *result) const {
#if 0
    env_->mutex_.lock();
    
    // place the OpenDE world at the start state
    si_->getStateSpace()->as<OpenDEStateSpace>()->writeState(state);
    
    // apply the controls
    env_->applyControl(control->as<RealVectorControlSpace::ControlType>()->values);
    
    // created contacts as needed
    CallbackParam cp = { env_.get(), false };
    for (unsigned int i = 0 ; i < env_->collisionSpaces_.size() ; ++i)
        dSpaceCollide(env_->collisionSpaces_[i],  &cp, &nearCallback);
    
    // propagate one step forward
    dWorldQuickStep(env_->world_, (const dReal)duration);
    
    // remove created contacts
    dJointGroupEmpty(env_->contactGroup_);
    
    // read the final state from the OpenDE world
    si_->getStateSpace()->as<OpenDEStateSpace>()->readState(result);
    
    env_->mutex_.unlock();
    
    // update the collision flag for the start state, if needed
    if (!(state->as<OpenDEStateSpace::StateType>()->collision & (1 << OpenDEStateSpace::STATE_COLLISION_KNOWN_BIT)))
    {
        if (cp.collision)
            state->as<OpenDEStateSpace::StateType>()->collision &= (1 << OpenDEStateSpace::STATE_COLLISION_VALUE_BIT);
        state->as<OpenDEStateSpace::StateType>()->collision &= (1 << OpenDEStateSpace::STATE_COLLISION_KNOWN_BIT);
    }
#endif
}

#if 0


    struct CallbackParam
    {
        const control::OpenDEEnvironment *env;
        bool                              collision;
    };
    
    void nearCallback(void *data, dGeomID o1, dGeomID o2)
    {
        dBodyID b1 = dGeomGetBody(o1);
        dBodyID b2 = dGeomGetBody(o2);
        
        if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
        
        CallbackParam *cp = reinterpret_cast<CallbackParam*>(data);
        
        const unsigned int maxContacts = cp->env->getMaxContacts(o1, o2);
        if (maxContacts <= 0) return;
        
        dContact *contact = new dContact[maxContacts];
        
        for (unsigned int i = 0; i < maxContacts; ++i)
            cp->env->setupContact(o1, o2, contact[i]);
        
        if (int numc = dCollide(o1, o2, maxContacts, &contact[0].geom, sizeof(dContact)))
        {
            for (int i = 0; i < numc; ++i)
            {
                dJointID c = dJointCreateContact(cp->env->world_, cp->env->contactGroup_, contact + i);
                dJointAttach(c, b1, b2);
                bool valid = cp->env->isValidCollision(o1, o2, contact[i]);
                if (!valid)
                    cp->collision = true;
                if (cp->env->verboseContacts_)
                {
                    OMPL_DEBUG("%s contact between %s and %s", (valid ? "Valid" : "Invalid"),
                               cp->env->getGeomName(o1).c_str(), cp->env->getGeomName(o1).c_str());
                }
            }
        }
        
        delete[] contact;
    }
}

void ompl::control::OpenDEStatePropagator::propagate(const base::State *state, const Control *control, const double duration, base::State *result) const
{
    env_->mutex_.lock();
    
    // place the OpenDE world at the start state
    si_->getStateSpace()->as<OpenDEStateSpace>()->writeState(state);
    
    // apply the controls
    env_->applyControl(control->as<RealVectorControlSpace::ControlType>()->values);
    
    // created contacts as needed
    CallbackParam cp = { env_.get(), false };
    for (unsigned int i = 0 ; i < env_->collisionSpaces_.size() ; ++i)
        dSpaceCollide(env_->collisionSpaces_[i],  &cp, &nearCallback);
    
    // propagate one step forward
    dWorldQuickStep(env_->world_, (const dReal)duration);
    
    // remove created contacts
    dJointGroupEmpty(env_->contactGroup_);
    
    // read the final state from the OpenDE world
    si_->getStateSpace()->as<OpenDEStateSpace>()->readState(result);
    
    env_->mutex_.unlock();
    
    // update the collision flag for the start state, if needed
    if (!(state->as<OpenDEStateSpace::StateType>()->collision & (1 << OpenDEStateSpace::STATE_COLLISION_KNOWN_BIT)))
    {
        if (cp.collision)
            state->as<OpenDEStateSpace::StateType>()->collision &= (1 << OpenDEStateSpace::STATE_COLLISION_VALUE_BIT);
        state->as<OpenDEStateSpace::StateType>()->collision &= (1 << OpenDEStateSpace::STATE_COLLISION_KNOWN_BIT);
    }
}

#endif


