//
//  OMPLTVSEnvironment.cpp
//  tvs
//
//  Created by Federico Ferri on 03/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "OMPLTVSEnvironment.h"
#include "OMPLTVSStateSpace.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

OMPLTVSEnvironment::OMPLTVSEnvironment(Environment *env)
: stepSize_(0.5), maxControlSteps_(5), minControlSteps_(1), env_(env) {
    std::string bakName = "searchTree", bakExt = ".csv";
    if(boost::filesystem::exists(bakName + bakExt)) {
        // backup existing file
        // find latest backup:
        int b = 0;
        std::stringstream ss;
        do {
            ss.str(std::string()); // clear ss
            ss << bakName << "." << b << bakExt;
            b++;
        } while(boost::filesystem::exists(ss.str()));
        boost::filesystem::rename(bakName + bakExt, ss.str());
    }
    searchTreeLogFile.open("searchTree.csv" /* , std::ios_base::app */);
}

OMPLTVSEnvironment::~OMPLTVSEnvironment() {
}

unsigned int OMPLTVSEnvironment::getControlDimension(void) const {
    return 2;
}

void OMPLTVSEnvironment::getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const {
    static double maxVel = 5.0;
    lower.resize(2);
    lower[0] = -maxVel;
    lower[1] = -maxVel;
    
    upper.resize(2);
    upper[0] = maxVel;
    upper[1] = maxVel;
}

void OMPLTVSEnvironment::applyControl(const double *control) const {
    env_->v->setTrackVelocities(control[0], control[1]);
}

void OMPLTVSEnvironment::addToSearchTree(const ompl::base::State *state, const ompl::control::Control *control, const double duration, const ompl::base::State *result) {
    searchTree.resize(searchTree.size() + 1);
    const dReal *a = state->as<OMPLTVSStateSpace::StateType>()->getBodyPosition(0);
    const dReal *b = result->as<OMPLTVSStateSpace::StateType>()->getBodyPosition(0);
    memcpy(searchTree.back().a, a, sizeof(dReal) * 3);
    memcpy(searchTree.back().b, b, sizeof(dReal) * 3);
    
    searchTreeLogFile
    << a[0] << "," << a[1] << "," << a[2] << ","
    << control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0] << ","
    << control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1] << ","
    << duration << ","
    << b[0] << " " << b[1] << " " << b[2]
    << std::endl;
    searchTreeLogFile.flush();
}
