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

void start() {
    static float xyz[3] = {6.3286,-5.9263,1.7600};
    static float hpr[3] = {102.5000,-16.0000,0.0000};
    dsSetViewpoint(xyz,hpr);
}

void step(int pause) {
    environment->draw();
#if 0
    if(!pause) {
        environment->step(0.01, 4);
        nstep++;
    }
#else
    // playback solution:
#endif
}

void stop() {
}

void setFrame(int d) {
    int newnstep = nstep + d;
    if(newnstep < 0 || newnstep >= path->getStateCount() || newnstep == nstep)
        return;
    nstep = newnstep;
    std::cout << "STEP " << nstep << "/" << path->getStateCount() << std::endl;
    
    ompl::base::State *state = path->getState(nstep);
    const double *p = state->as<ompl::control::OMPLTVSStateSpace::StateType>()->getPosition();
    ompl::base::SO3StateSpace::StateType& orientation = state->as<ompl::control::OMPLTVSStateSpace::StateType>()->getRotation();
    dQuaternion q;
    q[0] = orientation.w;
    q[1] = orientation.x;
    q[2] = orientation.y;
    q[3] = orientation.z;
    environment->v->setPosition(p);
    environment->v->setQuaternion(q);
}

void command(int cmd) {
    const dReal V = 6.0;
    switch(cmd) {
#if 0
    case 'd': environment->v->setTrackVelocities( V, -V); break;
    case 'a': environment->v->setTrackVelocities(-V,  V); break;
    case 'w': environment->v->setTrackVelocities(-V, -V); break;
    case 's': environment->v->setTrackVelocities( V,  V); break;
    case 'e': environment->v->setTrackVelocities( 0, -V); break;
    case 'q': environment->v->setTrackVelocities(-V,  0); break;
    case ' ': environment->v->setTrackVelocities( 0,  0); break;
#else
    case '+':case '=': setFrame(1); break;
    case '-':case '_': setFrame(-1); break;
#endif
    }
}

//#include "fe.c"

int main(int argc, char **argv) {
    //feenableexcept(FE_INVALID | FE_OVERFLOW);

    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);

    environment = new Environment();
    environment->create();

    ompl::control::OMPLTVSEnvironmentPtr env(new ompl::control::OMPLTVSEnvironment(environment));
    ompl::base::StateSpacePtr stateSpace(new ompl::control::OMPLTVSStateSpace(env));
    ompl::control::OMPLTVSSimpleSetup ss(stateSpace);
    ss.setGoalRegion(2, 2, 0, 0.1);
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-4);
    bounds.setHigh(4);
    stateSpace->as<ompl::control::OMPLTVSStateSpace>()->setVolumeBounds(bounds);
    ss.setup();
    if (ss.solve(20)) {
        ompl::control::PathControl path1 = ss.getSolutionPath();
        path = &path1;
        /*
        for(int i = 0; i < path.getStateCount(); i++) {
            if(i > 0) {
                const double *control = path.getControl(i - 1)->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
                double duration = path.getControlDuration(i - 1);
                std::cout << "  > (" << control[0] << ", " << control[1] << ") for " << duration << "s" << std::endl;
            }
            ompl::base::State *state = path.getState(i);
            const double *position = state->as<ompl::control::OMPLTVSStateSpace::StateType>()->getPosition();
            ompl::base::SO3StateSpace::StateType& orientation = state->as<ompl::control::OMPLTVSStateSpace::StateType>()->getRotation();
            std::cout << "position (" << position[0] << ", " << position[1] << ", " << position[2] << ") orientation (" << orientation.w << ", " << orientation.x << ", " << orientation.y << ", " << orientation.z << ")" << std::endl;
        }
        //path.printAsMatrix(std::cout);
         */
        ompl::geometric::PathGeometric pathg = path1.asGeometric();
        pathg.printAsMatrix(std::cout);
        
        environment->destroy();
        delete environment;
        environment = new Environment();
        environment->create();

        // play back solution
        dsFunctions fn;
        fn.version = DS_VERSION;
        fn.start = &start;
        fn.step = &step;
        fn.stop = &stop;
        fn.command = &command;
        fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
        dsSimulationLoop(argc, argv, 800, 600, &fn);
    } else {
        std::cout << "SOLUTION NOT FOUND" << std::endl;
    }
    
    environment->destroy();
    delete environment;
    
    dCloseODE();

    return 0;
}

