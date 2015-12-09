//
//  OMPLTVSSimpleSetup.h
//  tvs
//
//  Created by Federico Ferri on 03/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef OMPL_EXTENSION_OMPLTVS_SIMPLE_SETUP_
#define OMPL_EXTENSION_OMPLTVS_SIMPLE_SETUP_

#include <ompl/control/SimpleSetup.h>
#include <ompl/base/goals/GoalRegion.h>
#include "OMPLTVSStateValidityChecker.h"
#include "OMPLTVSStatePropagator.h"
#include "OMPLTVSControlSpace.h"

class OMPLTVSGoalRegion : public ompl::base::GoalRegion {
public:
    OMPLTVSGoalRegion(const ompl::base::SpaceInformationPtr &si, double x, double y, double z, double threshold);
    virtual double distanceGoal(const ompl::base::State *st) const;
protected:
    double x_, y_, z_;
};

/** \brief Create the set of classes typically needed to solve a
    control problem when forward propagation is computed with OMPLTVS. */
class OMPLTVSSimpleSetup : public ompl::control::SimpleSetup {
public:

    /** \brief Constructor needs the control space needed for planning. */
    explicit
    OMPLTVSSimpleSetup(const ompl::control::ControlSpacePtr &space);

    /** \brief The control space is assumed to be OMPLTVSControlSpace. Constructor only needs the state space. */
    explicit
    OMPLTVSSimpleSetup(const ompl::base::StateSpacePtr &space);

    /** \brief The control space is assumed to be
        OMPLTVSControlSpace. The state space is assumed to
        be OMPLTVSStateSpace. Constructor only needs the OMPLTVS
        environment. */
    explicit
    OMPLTVSSimpleSetup(const OMPLTVSEnvironmentPtr &env);

    virtual ~OMPLTVSSimpleSetup() {}

    /** \brief Get the OMPLTVS environment associated to the state and control spaces */
    const OMPLTVSEnvironmentPtr& getEnvironment() const {return getStateSpace()->as<OMPLTVSStateSpace>()->getEnvironment();}

    /** \brief Get the current OMPLTVS state (read parameters from OMPLTVS bodies) */
    ompl::base::ScopedState<OMPLTVSStateSpace> getCurrentState() const;

    /** \brief Set the current OMPLTVS state (set parameters for OMPLTVS bodies) */
    void setCurrentState(const ompl::base::ScopedState<> &state);

    /** \brief Set the current OMPLTVS state (set parameters for OMPLTVS bodies) */
    void setCurrentState(const ompl::base::State *state);

    /** \brief Set the bounds for the planning volume */
    void setVolumeBounds(const ompl::base::RealVectorBounds &bounds) {
        getStateSpace()->as<OMPLTVSStateSpace>()->setVolumeBounds(bounds);
    }

    /** \brief Set the bounds for the linear velocity */
    void setLinearVelocityBounds(const ompl::base::RealVectorBounds &bounds) {
        getStateSpace()->as<OMPLTVSStateSpace>()->setLinearVelocityBounds(bounds);
    }

    /** \brief Set the bounds for the angular velocity */
    void setAngularVelocityBounds(const ompl::base::RealVectorBounds &bounds) {
        getStateSpace()->as<OMPLTVSStateSpace>()->setAngularVelocityBounds(bounds);
    }

    /** \brief Set the OMPLTVS world to the states that are
        contained in a given path, sequentially. Using \e
        timeFactor, the speed at which this sequence is
        iterated through is altered. */
    void playPath(const ompl::base::PathPtr &path, double timeFactor = 1.0) const;

    /** \brief Call playPath() on the solution path, if one is available */
    void playSolutionPath(double timeFactor = 1.0) const;

    /** \brief Simulate the OMPLTVS environment forward for \e steps simulation steps, using the control \e control.
        Construct a path representing this action. */
    ompl::base::PathPtr simulateControl(const double *control, unsigned int steps) const;

    /** \brief Simulate the OMPLTVS environment forward for \e steps simulation steps, using the control \e control.
        Construct a path representing this action. */
    ompl::base::PathPtr simulateControl(const ompl::control::Control *control, unsigned int steps) const;

    /** \brief Simulate the OMPLTVS environment forward for \e
        steps simulation steps, using the null control
        (ompl::control::ControlSpace::nullControl()).
        Construct a path representing this action. */
    ompl::base::PathPtr simulate(unsigned int steps) const;

    inline void setGoalRegion(double x, double y, double z, double threshold) {
        setGoal(ompl::base::GoalPtr(new OMPLTVSGoalRegion(getSpaceInformation(), x, y, z, threshold)));
    }
    
    virtual void setup();

private:
    void useEnvParams();
};

#endif
