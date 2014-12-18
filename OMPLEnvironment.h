//
//  OMPLEnvironment.h
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef __tvs__OMPLEnvironment__
#define __tvs__OMPLEnvironment__

#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <ompl/util/ClassForward.h>
#include <cstdio>
#include "Environment.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

OMPL_CLASS_FORWARD(OMPLEnvironment);

class OMPLEnvironment {
public:
    OMPLEnvironment();
    virtual ~OMPLEnvironment();
    virtual unsigned int getControlDimension(void) const;
    virtual void getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const;
    virtual void applyControl(const double *control) const;
    virtual bool isValidCollision(dGeomID g1, dGeomID g2, const dContact& contact) const;

protected:
    // the simulation world
    Environment *env;
};

#endif /* defined(__tvs__OMPLEnvironment__) */
