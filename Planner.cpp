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
        const double *pos = st->as<oc::OMPLTVSStateSpace::StateType>()->getBodyPosition(0);
        double dx = fabs(pos[0] - 30);
        double dy = fabs(pos[1] - 55);
        double dz = fabs(pos[2] - 35);
        return sqrt(dx * dx + dy * dy + dz * dz);
    }
    
};


// Define how we project a state
class RigidBodyStateProjectionEvaluator : public ob::ProjectionEvaluator
{
public:
    
    RigidBodyStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
    {
    }
    
    virtual unsigned int getDimension(void) const
    {
        return 3;
    }
    
    virtual void defaultCellSizes(void)
    {
        cellSizes_.resize(3);
        cellSizes_[0] = 1;
        cellSizes_[1] = 1;
        cellSizes_[2] = 1;
    }
    
    virtual void project(const ob::State *state, ob::EuclideanProjection &projection) const
    {
        const double *pos = state->as<oc::OMPLTVSStateSpace::StateType>()->getBodyPosition(0);
        projection[0] = pos[0];
        projection[1] = pos[1];
        projection[2] = pos[2];
    }
    
};

// Define our own space, to include a distance function we want and register a default projection
class RigidBodyStateSpace : public oc::OMPLTVSStateSpace
{
public:
    
    RigidBodyStateSpace(const oc::OMPLTVSEnvironmentPtr &env) : oc::OMPLTVSStateSpace(env)
    {
    }
    
    virtual double distance(const ob::State *s1, const ob::State *s2) const
    {
        const double *p1 = s1->as<oc::OMPLTVSStateSpace::StateType>()->getBodyPosition(0);
        const double *p2 = s2->as<oc::OMPLTVSStateSpace::StateType>()->getBodyPosition(0);
        double dx = fabs(p1[0] - p2[0]);
        double dy = fabs(p1[1] - p2[1]);
        double dz = fabs(p1[2] - p2[2]);
        return sqrt(dx * dx + dy * dy + dz * dz);
    }
    
    virtual void registerProjections(void)
    {
        registerDefaultProjection(ob::ProjectionEvaluatorPtr(new RigidBodyStateProjectionEvaluator(this)));
    }
    
};

/// @endcond

void plan() {
    // initialize OMPLTVS
    dInitODE2(0);
    
    // create the OMPLTVS environment
    oc::OMPLTVSEnvironmentPtr env(new RigidBodyEnvironment());
    
    // create the state space and the control space for planning
    RigidBodyStateSpace *stateSpace = new RigidBodyStateSpace(env);
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
    
    if (ss.solve(10))
        ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
    
    dCloseODE();
}
