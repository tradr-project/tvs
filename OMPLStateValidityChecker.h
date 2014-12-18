//
//  OMPLStateValidityChecker.h
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef __tvs__OMPLStateValidityChecker__
#define __tvs__OMPLStateValidityChecker__

#include <ompl/base/StateSpace.h>
#include <ompl/util/ClassForward.h>
#include <cstdio>
#include "OMPLStateSpace.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

OMPL_CLASS_FORWARD(OMPLStateValidityChecker);

class OMPLStateValidityChecker : public ob::StateValidityChecker {
public:
    OMPLStateValidityChecker(const oc::SpaceInformationPtr &si);
    virtual bool isValid(const ob::State *state) const;
    
protected:
    OMPLStateSpacePtr stateSpacePtr;
};

#endif /* defined(__tvs__OMPLStateValidityChecker__) */
