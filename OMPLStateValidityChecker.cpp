//
//  OMPLStateValidityChecker.cpp
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "OMPLStateValidityChecker.h"

OMPLStateValidityChecker::OMPLStateValidityChecker(const oc::SpaceInformationPtr &si) : ob::StateValidityChecker(si), stateSpacePtr(si->getStateSpace()->as<OMPLStateSpace>()) {
}

bool OMPLStateValidityChecker::isValid(const ob::State *state) const {
    bool valid = false;
    /*
    if (!osm_->evaluateCollision(state))
        valid = osm_->satisfiesBoundsExceptRotation(s);
    
    if (valid)
        s->collision &= (1 << OpenDEStateSpace::STATE_VALIDITY_VALUE_BIT);
    
    // mark the fact we know the value of the validity bit
    s->collision &= (1 << OpenDEStateSpace::STATE_VALIDITY_KNOWN_BIT);
    */
    return valid;
}
