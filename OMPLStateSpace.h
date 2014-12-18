//
//  OMPLStateSpace.h
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef __tvs__OMPLStateSpace__
#define __tvs__OMPLStateSpace__

#include <ompl/base/StateSpace.h>
#include <ompl/util/ClassForward.h>
#include <cstdio>
#include "OMPLEnvironment.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

OMPL_CLASS_FORWARD(OMPLStateSpace);

class OMPLStateSpace : public ob::CompoundStateSpace {
public:
    OMPLStateSpace(const OMPLEnvironmentPtr &env);
    virtual ~OMPLStateSpace();

    //virtual double distance(const ob::State *s1, const ob::State *s2) const;
    //virtual void registerProjections();

protected:
    OMPLEnvironmentPtr env;
    ob::RealVectorBounds volumeBounds;
    ob::RealVectorBounds linearVelocityBounds;
    ob::RealVectorBounds angularVelocityBounds;
};

#endif /* defined(__tvs__OMPLStateSpace__) */
