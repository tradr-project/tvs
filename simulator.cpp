//
//  simulator.cpp
//  tvs
//
//  Created by Federico Ferri on 13/01/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "Environment.h"

Environment *environment;

#ifdef HAVE_JOYSTICK
#include "joystick.cpp"
#endif

void initRobotPose() {
    static dVector3 p = {2.08086,3.39581,0.102089};
    static dQuaternion q = {-0.229659,-0.00088334,0.00010361,0.973271};
    environment->v->setPosition(p);
    environment->v->setQuaternion(q);
    environment->v->setTrackVelocities(0, 0);
}

void lookAt(dReal eye_x, dReal eye_y, dReal eye_z, dReal x, dReal y, dReal z) {
    // TODO
}

void start() {
#ifdef HAVE_JOYSTICK
    if(environment->config.joystick.enabled)
        joy_open(environment->config.joystick.device.c_str());
#endif
    static float xyz[3] = {9.3812,4.5702,3.1600}; // {6.3286,-5.9263,1.7600};
    static float hpr[3] = {-142.5000,-34.5000,0.0000}; // {102.5000,-16.0000,0.0000};
    dsSetViewpoint(xyz,hpr);
}

void step(int pause) {
#ifdef HAVE_JOYSTICK
    if(environment->config.joystick.enabled)
        joy_poll();
    environment->v->setTrackVelocities(joy_r,joy_l);
#endif
    environment->draw();
    if(!pause) environment->step();
}

void stop() {
#ifdef HAVE_JOYSTICK
    if(environment->config.joystick.enabled)
        joy_close();
#endif
}

void printInfo() {
    const dReal *p = environment->v->getPosition();
    const dReal *q = environment->v->getQuaternion();
    const dReal *R = environment->v->getRotation();
    std::cout << "position: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
    std::cout << "orientation: " << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << std::endl;
    std::cout << "z: " << R[2] << ", " << R[6] << ", " << R[10] << std::endl;
}

void command(int cmd) {
    const dReal V = environment->config.world.max_track_speed;
    switch(cmd) {
        case 'd': environment->v->setTrackVelocities( V, -V); break;
        case 'a': environment->v->setTrackVelocities(-V,  V); break;
        case 'w': environment->v->setTrackVelocities(-V, -V); break;
        case 's': environment->v->setTrackVelocities( V,  V); break;
        case 'e': environment->v->setTrackVelocities(-0.25*V, -V); break;
        case 'q': environment->v->setTrackVelocities(-V, -0.25*V); break;
        case ' ': environment->v->setTrackVelocities( 0,  0); break;
        case 'p': printInfo(); break;
        case 'r': initRobotPose(); break;
    }
}

int main(int argc, char **argv) {
    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);

    environment = new Environment();
    environment->create();

    // set initial robot pose:
    initRobotPose();

    // run simulation loop & visualization
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &step;
    fn.stop = &stop;
    fn.command = &command;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    dsSimulationLoop(argc, argv, 800, 600, &fn);
    
    // quit -> cleanup
    environment->destroy();
    delete environment;
    
    dCloseODE();

    return 0;
}
