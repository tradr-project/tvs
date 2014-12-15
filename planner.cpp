#include <iostream>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/ControlSpaceTypes.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

static bool isStateValid(const ob::State *state) {
    return true;
}

static void propagateState(const ob::State *state, const oc::Control *control, const double boh, ob::State* newState) {
}

#if 0
void plan() {
    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::SE3StateSpace());
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->as<ob::SE3StateSpace>()->setBounds(bounds);

    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    si->setStateValidityChecker(boost::bind(&isStateValid, _1));
    si->setStatePropagator(boost::bind(&propagateState, _1, _2, _3, _4)));
    
    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 2));
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-1);
    cbounds.setHigh(1);
    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    oc::SpaceInformationPtr csi(new oc::SpaceInformation(space));
    csi->setMinMaxControlDuration(1, 5);

    ob::ScopedState<> start(space);
    start.random();

    ob::ScopedState<> goal(space);
    goal.random();

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start, goal);

    ob::PlannerPtr planner(new og::RRTConnect(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    ob::PlannerStatus solved = planner->solve(1.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        path->print(std::cout);
    }
}

int main(int argc, const char * argv[]) {
    plan();
    return 0;
}
#endif

