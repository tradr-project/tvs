//
//  Planner.cpp
//  tvs
//
//  Created by Federico Ferri on 16/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Planner.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/config.h>
#include <iostream>
#include <ode/ode.h>
#include "OMPLTVSSimpleSetup.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

void plan(Environment *penv) {
    oc::OMPLTVSEnvironmentPtr env(new oc::OMPLTVSEnvironment(penv));
    ob::StateSpacePtr stateSpace(new oc::OMPLTVSStateSpace(env));
    oc::OMPLTVSSimpleSetup ss(stateSpace);
    ss.setGoalRegion(2, 2, 0, 0.1);
    
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-4);
    bounds.setHigh(4);
    stateSpace->as<oc::OMPLTVSStateSpace>()->setVolumeBounds(bounds);

    ss.setup();
    //ss.print();
    
    if (ss.solve(10)) {
        ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
    }
}
