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
ompl::control::OMPLTVSEnvironmentPtr ompl_env;

enum {SIMULATOR, PLANNER} mode = PLANNER;

void start() {
    static float xyz[3] = {9.3812,4.5702,3.1600}; // {6.3286,-5.9263,1.7600};
    static float hpr[3] = {-142.5000,-34.5000,0.0000}; // {102.5000,-16.0000,0.0000};
    dsSetViewpoint(xyz,hpr);
    static dVector3 p = {2.08086,3.39581,0.102089};
    static dQuaternion q = {-0.229659,-0.00088334,0.00010361,0.973271};
    environment->v->setPosition(p);
    environment->v->setQuaternion(q);
}

void step(int pause) {
    environment->draw();
    
    if(mode == PLANNER) {
        // draw search tree
        for(std::vector<dLine>::iterator i = ompl_env->searchTree.begin(); i != ompl_env->searchTree.end(); i++) {
            dsDrawLineD(i->a, i->b);
        }
    }
    
    if(mode == SIMULATOR && !pause) {
        environment->step(0.01, 4);
        nstep++;
    }
}

void stop() {
}

void setFrame(int d) {
    if(!path) return;
    int newnstep = nstep + d;
    if(newnstep < 0 || newnstep >= path->getStateCount() || newnstep == nstep)
        return;
    nstep = newnstep;
    std::cout << "STEP " << (1+nstep) << "/" << path->getStateCount() << std::endl;
    
    ompl::base::State *state = path->getState(nstep);
    ss->writeState(state);
}

void printInfo() {
    const dReal *p = environment->v->getPosition();
    const dReal *q = environment->v->getQuaternion();
    std::cout << "position: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
    std::cout << "orientation: " << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << std::endl;
}

void command(int cmd) {
    if(mode == SIMULATOR) {
        const dReal V = 6.0;
        switch(cmd) {
            case 'd': environment->v->setTrackVelocities( V, -V); break;
            case 'a': environment->v->setTrackVelocities(-V,  V); break;
            case 'w': environment->v->setTrackVelocities(-V, -V); break;
            case 's': environment->v->setTrackVelocities( V,  V); break;
            case 'e': environment->v->setTrackVelocities(-0.25*V, -V); break;
            case 'q': environment->v->setTrackVelocities(-V, -0.25*V); break;
            case ' ': environment->v->setTrackVelocities( 0,  0); break;
            case 'p': printInfo(); break;
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

    if(mode == PLANNER) {
        static dVector3 p = {2.08086,3.39581,0.102089};
        static dQuaternion q = {-0.229659,-0.00088334,0.00010361,0.973271};
        environment->v->setPosition(p);
        environment->v->setQuaternion(q);
        ompl_env = ompl::control::OMPLTVSEnvironmentPtr(new ompl::control::OMPLTVSEnvironment(environment));
        ompl::base::StateSpacePtr stateSpace(ss = new ompl::control::OMPLTVSStateSpace(ompl_env));
        ompl::control::OMPLTVSSimpleSetup setup(stateSpace);
        setup.setGoalRegion(7.74, 0.95, 1.4, 0.4);
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(0, -2); bounds.setHigh(0, 8);
        bounds.setLow(1,  0); bounds.setHigh(1, 9);
        bounds.setLow(2,  0); bounds.setHigh(2, 2);
        stateSpace->as<ompl::control::OMPLTVSStateSpace>()->setVolumeBounds(bounds);
        setup.setup();
        if (setup.solve(600)) {
            path = new ompl::control::PathControl(setup.getSolutionPath());
            std::cout << "SOLUTION LENGTH: " << path->getStateCount() << std::endl;
            setFrame(0);
            //path->printAsMatrix(std::cout);
        } else {
            std::cout << "SOLUTION NOT FOUND" << std::endl;
            path = 0L;
        }
    }
    
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &step;
    fn.stop = &stop;
    fn.command = &command;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    dsSimulationLoop(argc, argv, 800, 600, &fn);
    
    if(path) delete path;
    environment->destroy();
    delete environment;
    
    dCloseODE();

    return 0;
}
