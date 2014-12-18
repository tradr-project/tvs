//
//  OMPLControlSpace.h
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef __tvs__OMPLControlSpace__
#define __tvs__OMPLControlSpace__

#include <ompl/base/StateSpace.h>
#include <ompl/util/ClassForward.h>
#include <cstdio>
#include "OMPLEnvironment.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

OMPL_CLASS_FORWARD(OMPLControlSpace);

class OMPLControlSpace : public oc::RealVectorControlSpace {
public:
    OMPLControlSpace(const ob::StateSpacePtr &stateSpace);
    virtual ~OMPLControlSpace();
};

#endif /* defined(__tvs__OMPLControlSpace__) */
