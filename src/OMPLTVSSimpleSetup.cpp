//
//  OMPLTVSSimpleSetup.cpp
//  tvs
//
//  Created by Federico Ferri on 03/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "OMPLTVSSimpleSetup.h"
#include <ompl/util/Exception.h>
#include <boost/thread.hpp>

OMPLTVSGoalRegion::OMPLTVSGoalRegion(const ompl::base::SpaceInformationPtr &si, double x, double y, double z, double threshold) : ompl::base::GoalRegion(si), x_(x), y_(y), z_(z) {
    threshold_ = threshold;
}

double OMPLTVSGoalRegion::distanceGoal(const ompl::base::State *st) const {
    const double *pos = st->as<OMPLTVSStateSpace::StateType>()->getBodyPosition(0);
    double dx = fabs(pos[0] - x_);
    double dy = fabs(pos[1] - y_);
    double dz = fabs(pos[2] - z_);
    return sqrt(dx * dx + dy * dy + dz * dz);
}

OMPLTVSSimpleSetup::OMPLTVSSimpleSetup(const ompl::control::ControlSpacePtr &space) : ompl::control::SimpleSetup(space) {
    if(!dynamic_cast<OMPLTVSControlSpace*>(space.get()))
        throw ompl::Exception("OMPLTVS Control Space needed for OMPLTVS Simple Setup");
    useEnvParams();
}

OMPLTVSSimpleSetup::OMPLTVSSimpleSetup(const ompl::base::StateSpacePtr &space)
: ompl::control::SimpleSetup(ompl::control::ControlSpacePtr(new OMPLTVSControlSpace(space))) {
    useEnvParams();
}

OMPLTVSSimpleSetup::OMPLTVSSimpleSetup(const OMPLTVSEnvironmentPtr &env)
: ompl::control::SimpleSetup(ompl::control::ControlSpacePtr(new OMPLTVSControlSpace(ompl::base::StateSpacePtr(new OMPLTVSStateSpace(env))))) {
    useEnvParams();
}

void OMPLTVSSimpleSetup::useEnvParams() {
    OMPLTVSEnvironmentPtr e = getStateSpace()->as<OMPLTVSStateSpace>()->getEnvironment();
    si_->setPropagationStepSize(e->stepSize_);
    si_->setMinMaxControlDuration(e->minControlSteps_, e->maxControlSteps_);
    si_->setStatePropagator(ompl::control::StatePropagatorPtr(new OMPLTVSStatePropagator(si_)));
}

ompl::base::ScopedState<OMPLTVSStateSpace> OMPLTVSSimpleSetup::getCurrentState() const {
    ompl::base::ScopedState<OMPLTVSStateSpace> current(getStateSpace());
    getStateSpace()->as<OMPLTVSStateSpace>()->readState(current.get());
    return current;
}

void OMPLTVSSimpleSetup::setCurrentState(const ompl::base::State *state) {
    getStateSpace()->as<OMPLTVSStateSpace>()->writeState(state);
}

void OMPLTVSSimpleSetup::setCurrentState(const ompl::base::ScopedState<> &state) {
    getStateSpace()->as<OMPLTVSStateSpace>()->writeState(state.get());
}

void OMPLTVSSimpleSetup::setup() {
    if(!si_->getStateValidityChecker()) {
        OMPL_INFORM("Using default state validity checker for OMPLTVS");
        si_->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new OMPLTVSStateValidityChecker(si_)));
    }
    if(pdef_->getStartStateCount() == 0) {
        OMPL_INFORM("Using the initial state of OMPLTVS as the starting state for the planner");
        pdef_->addStartState(getCurrentState());
    }
    SimpleSetup::setup();
}

void OMPLTVSSimpleSetup::playSolutionPath(double timeFactor) const {
    if(haveSolutionPath())
        playPath(pdef_->getSolutionPath(), timeFactor);
}

void OMPLTVSSimpleSetup::playPath(const ompl::base::PathPtr &path, double timeFactor) const {
    bool ctl = false;
    if(dynamic_cast<ompl::control::PathControl*>(path.get()))
        ctl = true;
    else
        if(!dynamic_cast<ompl::geometric::PathGeometric*>(path.get()))
            throw ompl::Exception("Unknown type of path");

    const ompl::geometric::PathGeometric &pg = ctl ?
        static_cast<ompl::control::PathControl*>(path.get())->asGeometric() : *static_cast<ompl::geometric::PathGeometric*>(path.get());

    if(pg.getStateCount() > 0) {
        OMPL_DEBUG("Playing through %u states (%0.3f seconds)", (unsigned int)pg.getStateCount(),
                   timeFactor * si_->getPropagationStepSize() * (double)(pg.getStateCount() - 1));
        ompl::time::duration d = ompl::time::seconds(timeFactor * si_->getPropagationStepSize());
        getStateSpace()->as<OMPLTVSStateSpace>()->writeState(pg.getState(0));
        for(unsigned int i = 1; i < pg.getStateCount(); ++i) {
            boost::this_thread::sleep(d);
            getStateSpace()->as<OMPLTVSStateSpace>()->writeState(pg.getState(i));
        }
    }
}

ompl::base::PathPtr OMPLTVSSimpleSetup::simulateControl(const double *control, unsigned int steps) const {
    ompl::control::Control *c = si_->allocControl();
    memcpy(c->as<OMPLTVSControlSpace::ControlType>()->values, control, sizeof(double) * getControlSpace()->getDimension());
    ompl::base::PathPtr path = simulateControl(c, steps);
    si_->freeControl(c);
    return path;
}

ompl::base::PathPtr OMPLTVSSimpleSetup::simulateControl(const ompl::control::Control *control, unsigned int steps) const {
    ompl::control::PathControl *p = new ompl::control::PathControl(si_);

    ompl::base::State *s0 = si_->allocState();
    getStateSpace()->as<OMPLTVSStateSpace>()->readState(s0);
    p->getStates().push_back(s0);

    ompl::base::State *s1 = si_->allocState();
    si_->propagate(s0, control, steps, s1);
    p->getStates().push_back(s1);

    p->getControls().push_back(si_->cloneControl(control));
    p->getControlDurations().push_back(steps);
    return ompl::base::PathPtr(p);
}

ompl::base::PathPtr OMPLTVSSimpleSetup::simulate(unsigned int steps) const {
    ompl::control::Control *c = si_->allocControl();
    si_->nullControl(c);
    ompl::base::PathPtr path = simulateControl(c, steps);
    si_->freeControl(c);
    return path;
}
