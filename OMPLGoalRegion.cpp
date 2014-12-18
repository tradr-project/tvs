//
//  OMPLGoalRegion.cpp
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "OMPLGoalRegion.h"

OMPLGoalRegion::OMPLGoalRegion(const ob::SpaceInformationPtr &si) : ob::GoalRegion(si) {
    threshold_ = 0.5;
}

double OMPLGoalRegion::distanceGoal(const ob::State *st) const {
    const double *pos = st->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
    double dx = fabs(pos[0] - 30);
    double dy = fabs(pos[1] - 55);
    double dz = fabs(pos[2] - 35);
    return sqrt(dx * dx + dy * dy + dz * dz);
}
