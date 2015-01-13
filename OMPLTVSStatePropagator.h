//
//  OMPLTVSStatePropagator.h
//  tvs
//
//  Created by Federico Ferri on 03/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef OMPL_EXTENSION_OMPLTVS_STATE_PROPAGATOR_
#define OMPL_EXTENSION_OMPLTVS_STATE_PROPAGATOR_

#include <ompl/control/SpaceInformation.h>
#include "OMPLTVSEnvironment.h"

/** \brief State propagation with OMPLTVS. Only forward
    propagation is possible.

    At every propagation step, controls are applied using
    OMPLTVSEnvironment::applyControl(), contacts are computed by
    calling \b dSpaceCollide() on the spaces in
    OMPLTVSEnvironment::collisionSpaces_ and then \b
    dWorldQuickStep() is called. If the \e state argument of
    propagate() does not have its
    OMPLTVSStateSpace::StateType::collision field set, it is
    set based on the information returned by contact
    computation. Certain collisions (contacts) are allowed, as
    indicated by OMPLTVSEnvironment::isValidCollision(). */
class OMPLTVSStatePropagator : public ompl::control::StatePropagator {
public:
    /** \brief Construct a representation of OMPLTVS state propagator.
        If \e si->getStateSpace() does not cast to an
        OMPLTVSStateSpace, an exception is thrown. */
    OMPLTVSStatePropagator(const ompl::control::SpaceInformationPtr &si);

    virtual ~OMPLTVSStatePropagator() {}

    /** \brief Get the OMPLTVS environment this state propagator operates on */
    const OMPLTVSEnvironmentPtr& getEnvironment() const {return env_;}

    virtual bool canPropagateBackward() const;

    virtual void propagate(const ompl::base::State *state, const ompl::control::Control *control, const double duration, ompl::base::State *result) const;
protected:
    /** \brief The OMPLTVS environment this state propagator operates on */
    OMPLTVSEnvironmentPtr env_;
};

#endif
