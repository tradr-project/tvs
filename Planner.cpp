//
//  Planner.cpp
//  tvs
//
//  Created by Federico Ferri on 16/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Planner.h"
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/config.h>
#include <iostream>
#include <ode/ode.h>
#include "OMPLTVSSimpleSetup.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

// Define the goal we want to reach
class RigidBodyGoal : public ob::GoalRegion
{
public:
    RigidBodyGoal(const ob::SpaceInformationPtr &si) : ob::GoalRegion(si)
    {
        threshold_ = 0.5;
    }
    
    virtual double distanceGoal(const ob::State *st) const
    {
        const double *pos = st->as<oc::OMPLTVSStateSpace::StateType>()->getPosition();
        double dx = fabs(pos[0] - 30);
        double dy = fabs(pos[1] - 55);
        double dz = fabs(pos[2] - 35);
        return sqrt(dx * dx + dy * dy + dz * dz);
    }
};

void plan(Environment *penv) {
    // create the OMPLTVS environment
    oc::OMPLTVSEnvironmentPtr env(new oc::OMPLTVSEnvironment(penv));
    
    // create the state space and the control space for planning
    oc::OMPLTVSStateSpace *stateSpace = new oc::OMPLTVSStateSpace(env);
    ob::StateSpacePtr stateSpacePtr = ob::StateSpacePtr(stateSpace);
    
    // this will take care of setting a proper collision checker and the starting state for the planner as the initial OMPLTVS state
    oc::OMPLTVSSimpleSetup ss(stateSpacePtr);
    
    // set the goal we would like to reach
    ss.setGoal(ob::GoalPtr(new RigidBodyGoal(ss.getSpaceInformation())));
    
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-200);
    bounds.setHigh(200);
    stateSpace->setVolumeBounds(bounds);
    
    bounds.setLow(-20);
    bounds.setHigh(20);
    stateSpace->setLinearVelocityBounds(bounds);
    stateSpace->setAngularVelocityBounds(bounds);
    
    ss.setup();
    ss.print();
    
    if (ss.solve(10)) {
        ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
    }
}
