//
//  OMPLTVSStateValidityChecker.h
//  tvs
//
//  Created by Federico Ferri on 03/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef OMPL_EXTENSION_OMPLTVS_STATE_VALIDITY_CHECKER_
#define OMPL_EXTENSION_OMPLTVS_STATE_VALIDITY_CHECKER_

#include "OMPLTVSStateSpace.h"
#include <ompl/control/SpaceInformation.h>

/** \brief The simplest state validity checker: all states are valid */
class OMPLTVSStateValidityChecker : public ompl::base::StateValidityChecker {
public:
    /** \brief Constructor */
    OMPLTVSStateValidityChecker(const ompl::control::SpaceInformationPtr &si);

    /** \brief A state is considered valid if it is within bounds and not in collision */
    virtual bool isValid(const ompl::base::State *state) const;

protected:
    /** \brief The corresponding OMPLTVS state space */
    OMPLTVSStateSpace *osm_;
};

#endif
