//
//  OMPLTVSStateValidityChecker.cpp
//  tvs
//
//  Created by Federico Ferri on 03/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "OMPLTVSStateValidityChecker.h"
#include <ompl/util/Exception.h>

OMPLTVSStateValidityChecker::OMPLTVSStateValidityChecker(const ompl::control::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si) {
    if(!dynamic_cast<OMPLTVSStateSpace*>(si->getStateSpace().get()))
        throw ompl::Exception("Cannot create state validity checking for OMPLTVS without OMPLTVS state space");
    osm_ = si->getStateSpace()->as<OMPLTVSStateSpace>();
}

bool OMPLTVSStateValidityChecker::isValid(const ompl::base::State *state) const {
    const OMPLTVSStateSpace::StateType *s = state->as<OMPLTVSStateSpace::StateType>();

    // if we know the value of the validity flag for this state, we return it
    if(s->collision & (1 << OMPLTVSStateSpace::STATE_VALIDITY_KNOWN_BIT))
        return s->collision & (1 << OMPLTVSStateSpace::STATE_VALIDITY_VALUE_BIT);

    // if not, we compute it:
    bool valid = false;

    if(!osm_->evaluateCollision(state)) {
        bool satBounds = osm_->satisfiesBoundsExceptRotation(s);
        if(!satBounds) {
            std::cout << "OMPLTVSStateValidityChecker::isValid(): invalid bounds" << std::endl;
        }
        valid = satBounds;
    }

    if(valid)
        s->collision &= (1 << OMPLTVSStateSpace::STATE_VALIDITY_VALUE_BIT);

    // mark the fact we know the value of the validity bit
    s->collision &= (1 << OMPLTVSStateSpace::STATE_VALIDITY_KNOWN_BIT);

    return valid;
}
