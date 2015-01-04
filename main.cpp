//
//  main.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "Environment.h"
#include "OMPLTVSSimpleSetup.h"

Environment *environment;

int nstep = 0;

void start() {
    static float xyz[3] = {6.3286,-5.9263,1.7600};
    static float hpr[3] = {102.5000,-16.0000,0.0000};
    dsSetViewpoint(xyz,hpr);
}

void step(int pause) {
    environment->draw();
    if(!pause) {
        environment->step(0.01, 4);
        nstep++;
    }
}

void stop() {
}

void command(int cmd) {
    const dReal V = 6.0;
    switch(cmd) {
    case 'd': environment->v->setTrackVelocities( V, -V); break;
    case 'a': environment->v->setTrackVelocities(-V,  V); break;
    case 'w': environment->v->setTrackVelocities(-V, -V); break;
    case 's': environment->v->setTrackVelocities( V,  V); break;
    case 'e': environment->v->setTrackVelocities( 0, -V); break;
    case 'q': environment->v->setTrackVelocities(-V,  0); break;
    case ' ': environment->v->setTrackVelocities( 0,  0); break;
    }
}

//#include "fe.c"

int main(int argc, char **argv) {
    //feenableexcept(FE_INVALID | FE_OVERFLOW);

    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);

    environment = new Environment();
    environment->create();

#if 0
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &step;
    fn.stop = &stop;
    fn.command = &command;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    dsSimulationLoop(argc, argv, 800, 600, &fn);
#else
    ompl::control::OMPLTVSEnvironmentPtr env(new ompl::control::OMPLTVSEnvironment(environment));
    ompl::base::StateSpacePtr stateSpace(new ompl::control::OMPLTVSStateSpace(env));
    ompl::control::OMPLTVSSimpleSetup ss(stateSpace);
    ss.setGoalRegion(2, 2, 0, 0.1);
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-4);
    bounds.setHigh(4);
    stateSpace->as<ompl::control::OMPLTVSStateSpace>()->setVolumeBounds(bounds);
    ss.setup();
    if (ss.solve(10)) {
        ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
    }
#endif
    
    environment->destroy();
    delete environment;
    
    dCloseODE();

    return 0;
}

