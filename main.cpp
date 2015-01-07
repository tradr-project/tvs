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
ompl::control::PathControl *path;
ompl::control::OMPLTVSStateSpace *ss;

enum {SIMULATOR, PLANNER} mode = SIMULATOR;

void start() {
    static float xyz[3] = {6.3286,-5.9263,1.7600};
    static float hpr[3] = {102.5000,-16.0000,0.0000};
    dsSetViewpoint(xyz,hpr);
}

void step(int pause) {
    environment->draw();
    if(mode == SIMULATOR && !pause) {
        environment->step(0.01, 4);
        nstep++;
    }
}

void stop() {
}

void setFrame(int d) {
    int newnstep = nstep + d;
    if(newnstep < 0 || newnstep >= path->getStateCount() || newnstep == nstep)
        return;
    nstep = newnstep;
    std::cout << "STEP " << (1+nstep) << "/" << path->getStateCount() << std::endl;
    
    ompl::base::State *state = path->getState(nstep);
    ss->writeState(state);
}

void command(int cmd) {
    if(mode == SIMULATOR) {
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
    } else if(mode == PLANNER) {
        switch(cmd) {
            case '+':case '=': setFrame(1); break;
            case '-':case '_': setFrame(-1); break;
        }
    }
}

//#include "fe.c"

int main(int argc, char **argv) {
    //feenableexcept(FE_INVALID | FE_OVERFLOW);

    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);

    environment = new Environment();
    environment->create();

    bool run = true;
    
    if(mode == PLANNER) {
        ompl::control::OMPLTVSEnvironmentPtr env(new ompl::control::OMPLTVSEnvironment(environment));
        ompl::base::StateSpacePtr stateSpace(ss = new ompl::control::OMPLTVSStateSpace(env));
        ompl::control::OMPLTVSSimpleSetup setup(stateSpace);
        setup.setGoalRegion(2, 2, 0.301, 0.01);
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(-4);
        bounds.setHigh(4);
        stateSpace->as<ompl::control::OMPLTVSStateSpace>()->setVolumeBounds(bounds);
        setup.setup();
        if (setup.solve(60)) {
            ompl::control::PathControl path1 = setup.getSolutionPath();
            path = &path1;
            setFrame(0);
            //path.printAsMatrix(std::cout);
        } else {
            std::cout << "SOLUTION NOT FOUND" << std::endl;
            run = false;
        }
    }
    
    if(run) {
        dsFunctions fn;
        fn.version = DS_VERSION;
        fn.start = &start;
        fn.step = &step;
        fn.stop = &stop;
        fn.command = &command;
        fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
        dsSimulationLoop(argc, argv, 800, 600, &fn);
    }
    
    environment->destroy();
    delete environment;
    
    dCloseODE();

    return 0;
}
