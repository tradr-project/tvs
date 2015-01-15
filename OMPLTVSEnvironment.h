//
//  OMPLTVSEnvironment.h
//  tvs
//
//  Created by Federico Ferri on 03/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef OMPL_EXTENSION_OMPLTVS_ENVIRONMENT_
#define OMPL_EXTENSION_OMPLTVS_ENVIRONMENT_

#include "Environment.h"
#include "ODEUtils.h"

#include <ompl/config.h>
#include <ompl/base/State.h>
#include <ompl/control/Control.h>
#include <ompl/util/ClassForward.h>

#include <ode/ode.h>
#include <vector>
#include <boost/thread/mutex.hpp>

OMPL_CLASS_FORWARD(OMPLTVSEnvironment);

class OMPLTVSEnvironment {
public:
    Environment *env_;
    
    /** \brief The simulation step size */
    double                stepSize_;

    /** \brief The maximum number of times a control is applies in sequence */
    unsigned int          maxControlSteps_;

    /** \brief The minimum number of times a control is applies in sequence */
    unsigned int          minControlSteps_;

    /** \brief Lock to use when performing simulations in the world. (OMPLTVS simulations are NOT thread safe) */
    mutable boost::mutex  mutex_;

    OMPLTVSEnvironment(Environment *env);

    virtual ~OMPLTVSEnvironment();

    /** \brief Number of parameters (double values) needed to specify a control input */
    virtual unsigned int getControlDimension() const;

    /** \brief Get the control bounds -- the bounding box in which to sample controls */
    virtual void getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const;

    /** \brief Application of a control. This function sets
        the forces/torques/velocities for bodies in the
        simulation based on control inputs.*/
    virtual void applyControl(const double *control) const;
    
    std::vector<dLine> searchTree;
    void addToSearchTree(const ompl::base::State *s1, const ompl::control::Control *control, const double duration, const ompl::base::State *s2);
    std::ofstream searchTreeLogFile;
};

#endif
