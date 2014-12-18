//
//  OMPLGoalRegion.h
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef __tvs__OMPLGoalRegion__
#define __tvs__OMPLGoalRegion__

#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <ompl/base/goals/GoalRegion.h>
#include <cstdio>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

class OMPLGoalRegion : public ob::GoalRegion {
public:
    OMPLGoalRegion(const ob::SpaceInformationPtr &si);
    virtual double distanceGoal(const ob::State *st) const;
};

#endif /* defined(__tvs__OMPLGoalRegion__) */
