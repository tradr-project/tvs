//
//  OMPLStateProjectionEvaluator.cpp
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "OMPLStateProjectionEvaluator.h"

OMPLStateProjectionEvaluator::OMPLStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space) {
}

unsigned int OMPLStateProjectionEvaluator::getDimension(void) const {
    return 3;
}

void OMPLStateProjectionEvaluator::defaultCellSizes() {
    cellSizes_.resize(3);
    cellSizes_[0] = 1;
    cellSizes_[1] = 1;
    cellSizes_[2] = 1;
}

void OMPLStateProjectionEvaluator::project(const ob::State *state, ob::EuclideanProjection &projection) const {
    const double *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
    projection[0] = pos[0];
    projection[1] = pos[1];
    projection[2] = pos[2];
}
