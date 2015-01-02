//
//  OMPLStateProjectionEvaluator.cpp
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "OMPLStateProjectionEvaluator.h"
#include "OMPLStateSpace.h"

OMPLStateProjectionEvaluator::OMPLStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space) {
    std::vector<double> cellSizes(3);
    cellSizes[0] = 0.25;
    cellSizes[1] = 0.25;
    cellSizes[2] = 0.25;
    setCellSizes(cellSizes);
}

unsigned int OMPLStateProjectionEvaluator::getDimension(void) const {
    return 3;
}

void OMPLStateProjectionEvaluator::project(const ob::State *state, ob::EuclideanProjection &projection) const {
    projection[0] = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
    projection[1] = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
    projection[2] = state->as<ob::RealVectorStateSpace::StateType>()->values[2];
}
