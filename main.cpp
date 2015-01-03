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
#include "Planner.h"

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

#if 0
    environment = new Environment();
    environment->create();

    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &step;
    fn.stop = &stop;
    fn.command = &command;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    dsSimulationLoop(argc, argv, 800, 600, &fn);
    
    environment->destroy();
    delete environment;
    
#else
    plan();
#endif
    
    dCloseODE();

    return 0;
}

