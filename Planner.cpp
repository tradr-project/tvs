//
//  Planner.cpp
//  tvs
//
//  Created by Federico Ferri on 16/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Planner.h"
#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/config.h>
#include <iostream>
#include <ode/ode.h>
#include "OMPLEnvironment.h"
#include "OMPLStateSpace.h"
#include "OMPLGoalRegion.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

void plan() {
    oc::OpenDEEnvironmentPtr envPtr(new OMPLEnvironment());
    ob::StateSpacePtr stateSpacePtr = ob::StateSpacePtr(new OMPLStateSpace(envPtr));
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-200);
    bounds.setHigh(200);
    stateSpacePtr->as<OMPLStateSpace>()->setVolumeBounds(bounds);
    bounds.setLow(-20);
    bounds.setHigh(20);
    stateSpacePtr->as<OMPLStateSpace>()->setLinearVelocityBounds(bounds);
    stateSpacePtr->as<OMPLStateSpace>()->setAngularVelocityBounds(bounds);
    oc::ControlSpacePtr controlSpacePtr = oc::ControlSpacePtr(new oc::OpenDEControlSpace(stateSpacePtr));
    oc::SpaceInformationPtr spaceInformationPtr(new oc::SpaceInformation(stateSpacePtr, controlSpacePtr));
    oc::StatePropagatorPtr statePropagatorPtr(new oc::OpenDEStatePropagator(spaceInformationPtr));
    ob::StateValidityCheckerPtr stateValidityCheckerPtr(new oc::OpenDEStateValidityChecker(spaceInformationPtr));
    spaceInformationPtr->setPropagationStepSize(envPtr->stepSize_);
    spaceInformationPtr->setMinMaxControlDuration(envPtr->minControlSteps_, envPtr->maxControlSteps_);
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

