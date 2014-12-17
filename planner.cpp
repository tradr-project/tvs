//
//  planner.cpp
//  tvs
//
//  Created by Federico Ferri on 16/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "planner.h"
#include <iostream>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

static bool isStateValid(const ob::State *state) {
    return true;
}

static void propagateState(const ob::State *state, const oc::Control *control, const double duration, ob::State* newState) {
}

void plan() {
    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::SE3StateSpace());
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->as<ob::SE3StateSpace>()->setBounds(bounds);

    //ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    //si->setStateValidityChecker(boost::bind(&isStateValid, _1));
    //si->setStatePropagator(boost::bind(&propagateState, _1, _2, _3, _4)));
    
    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 2));
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-1);
    cbounds.setHigh(1);
    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    //oc::SpaceInformationPtr csi(new oc::SpaceInformation(space));
    //csi->setMinMaxControlDuration(1, 5);

    oc::SimpleSetup ss(cspace);
    //ss.setStateValidityChecker(boost::bind(&isStateValid, ss.getSpaceInformation().get(), _1));
    //ss.setStatePropagator(boost::bind(&propagateState, _1, _2, _3, _4));
    
    ob::ScopedState<> start(space);
    start.random();

    ob::ScopedState<> goal(space);
    goal.random();

    //ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    //pdef->setStartAndGoalStates(start, goal);

    //ob::PlannerPtr planner(new og::RRTConnect(si));
    //planner->setProblemDefinition(pdef);
    //planner->setup();

    //ob::PlannerStatus solved = planner->solve(1.0);
    
    ss.setStartAndGoalStates(start, goal);
    ss.setup();
    ob::PlannerStatus solved = ss.solve(100.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        og::PathGeometric path = ss.getSolutionPath().asGeometric();
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        path.printAsMatrix(std::cout);
    }
}

#if 0
int main(int argc, const char * argv[]) {
    plan();
    return 0;
}
#endif

