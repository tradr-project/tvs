//
//  OMPLStatePropagator.h
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef __tvs__OMPLStatePropagator__
#define __tvs__OMPLStatePropagator__

#include <ompl/control/SimpleSetup.h>
#include <ompl/util/Exception.h>
#include <ompl/util/Console.h>
#include <cstdio>
#include "OMPLEnvironment.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

class OMPLStatePropagator : public oc::StatePropagator {
public:
    OMPLStatePropagator(const oc::SpaceInformationPtr &si);
    virtual ~OMPLStatePropagator();
    virtual void propagate(const ob::State *state, const oc::Control *control, const double duration, ob::State *result) const;
};

#endif /* defined(__tvs__OMPLStatePropagator__) */
