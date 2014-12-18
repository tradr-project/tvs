//
//  OMPLStateProjectionEvaluator.h
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef __tvs__OMPLStateProjectionEvaluator__
#define __tvs__OMPLStateProjectionEvaluator__

#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <cstdio>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

class OMPLStateProjectionEvaluator : public ob::ProjectionEvaluator {
public:
    OMPLStateProjectionEvaluator(const ob::StateSpace *space);
    virtual unsigned int getDimension(void) const;
    virtual void defaultCellSizes();
    virtual void project(const ob::State *state, ob::EuclideanProjection &projection) const;
};

#endif /* defined(__tvs__OMPLStateProjectionEvaluator__) */
