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
#include "OMPLEnvironment.h"
#include "OMPLStateSpace.h"
#include "OMPLStateValidityChecker.h"
#include "OMPLControlSpace.h"
#include "OMPLStatePropagator.h"
#include "OMPLGoalRegion.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

void plan() {
    OMPLEnvironmentPtr envPtr(new OMPLEnvironment());
    ob::StateSpacePtr stateSpacePtr = ob::StateSpacePtr(new OMPLStateSpace(envPtr));
    //ob::StateSpacePtr stateSpacePtr = ob::StateSpacePtr(new ob::SE3StateSpace());
    oc::ControlSpacePtr controlSpacePtr = oc::ControlSpacePtr(new OMPLControlSpace(stateSpacePtr));
    oc::SpaceInformationPtr spaceInformationPtr(new oc::SpaceInformation(stateSpacePtr, controlSpacePtr));
    oc::StatePropagatorPtr statePropagatorPtr(new OMPLStatePropagator(spaceInformationPtr));
    ob::StateValidityCheckerPtr stateValidityCheckerPtr(new OMPLStateValidityChecker(spaceInformationPtr));
    spaceInformationPtr->setPropagationStepSize(0.01);
    spaceInformationPtr->setMinMaxControlDuration(1, 10);
    spaceInformationPtr->setStatePropagator(statePropagatorPtr);
    spaceInformationPtr->setStateValidityChecker(ob::StateValidityCheckerPtr(stateValidityCheckerPtr));
    spaceInformationPtr->setup();
    ob::ProblemDefinitionPtr problemDefinitionPtr(new ob::ProblemDefinition(spaceInformationPtr));
    ob::GoalPtr goalPtr(new OMPLGoalRegion(spaceInformationPtr));
    problemDefinitionPtr->setGoal(goalPtr);
    ob::PlannerPtr plannerPtr(new og::KPIECE1(spaceInformationPtr));
    plannerPtr->setProblemDefinition(problemDefinitionPtr);
    ob::PlannerStatus status = plannerPtr->solve(1.0);
    if(problemDefinitionPtr->getSolutionPath())
    {
        std::cout << "solution found" << std::endl;
        problemDefinitionPtr->getSolutionPath()->print(std::cout);
    }
}

