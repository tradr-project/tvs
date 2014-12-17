//
//  main.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "world.h"

World *world;
const int simulationStepsPerFrame = 4;
int nstep = 0;

void start() {
    static float xyz[3] = {6.3286,-5.9263,1.7600};
    static float hpr[3] = {102.5000,-16.0000,0.0000};
    dsSetViewpoint(xyz,hpr);
}

void step(int pause) {
    world_draw(world);
    if(!pause) {
        world_step(world, 0.01, 4);
        nstep++;
    }
}

void stop() {
}

void command(int cmd) {
    const dReal V = 5;

#define SetVel(trk,vv) dJointSetHingeParam(world->v->trk##Track->wheel2Joint, dParamVel, vv)
#define MapKey(k,vr,vl) case k: SetVel(right, vr); SetVel(left, vl); break;

    switch(cmd) {
    MapKey('a',  V, -V);
    MapKey('d', -V,  V);
    MapKey('w', -V, -V);
    MapKey('s',  V,  V);
    MapKey('q',  0, -V);
    MapKey('e', -V,  0);
    MapKey(' ',  0,  0);
    }
    
#undef MapKey
#undef SetVel
}

//#include "fe.c"

int main(int argc, char **argv) {
    //feenableexcept(FE_INVALID | FE_OVERFLOW);
    
    world = world_init();
    world_create(world);

    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &step;
    fn.stop = &stop;
    fn.command = &command;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    dsSimulationLoop(argc, argv, 800, 600, &fn);

    world_deinit(world);
    world_destroy(world);

    return 0;
}

