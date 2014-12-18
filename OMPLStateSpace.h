//
//  OMPLStateSpace.h
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef __tvs__OMPLStateSpace__
#define __tvs__OMPLStateSpace__

#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <ompl/util/ClassForward.h>
#include <cstdio>
#include "OMPLEnvironment.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

OMPL_CLASS_FORWARD(OMPLStateSpace);

class OMPLStateSpace : public oc::OpenDEStateSpace {
public:
    OMPLStateSpace(const oc::OpenDEEnvironmentPtr &env);
    virtual double distance(const ob::State *s1, const ob::State *s2) const;
    virtual void registerProjections();
    //virtual bool evaluateCollision(const ob::State *source) const;
};

#endif /* defined(__tvs__OMPLStateSpace__) */
