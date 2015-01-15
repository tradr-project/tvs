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
        // check bounds
        bool satBounds = osm_->satisfiesBoundsExceptRotation(s);
        
        // check pose (i.e. if robot is about to flip)
        dMatrix3 R;
        const ompl::base::SO3StateSpace::StateType &s_rot = s->getBodyRotation(0);
        dQuaternion q = {s_rot.w, s_rot.x, s_rot.y, s_rot.z};
        dRfromQ(R, q);
        bool badPose = R[10] < 0.707; // angle between robot z and world z > 45 deg
        
        valid = satBounds && !badPose;
        
        if(!valid) {
            if(!satBounds) std::cout << "invalid state: exceeds bounds" << std::endl;
            if(badPose) std::cout << "invalid state: bad pose" << std::endl;
        }
    }

    if(valid)
        s->collision &= (1 << OMPLTVSStateSpace::STATE_VALIDITY_VALUE_BIT);

    // mark the fact we know the value of the validity bit
    s->collision &= (1 << OMPLTVSStateSpace::STATE_VALIDITY_KNOWN_BIT);

    return valid;
}
