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
#include <SDL.h>
#include <SDL_joystick.h>

Environment *environment;
SDL_Joystick *joystick;

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
    if(environment->config.joystick.enabled) {
        if(SDL_NumJoysticks() > 0) {
            std::cout << "Available joysticks:" << std::endl;
            for(int i=0; i < SDL_NumJoysticks(); i++)
                std::cout << i << ": " << SDL_JoystickNameForIndex(i) << std::endl;
            SDL_JoystickEventState(SDL_ENABLE);
            joystick = SDL_JoystickOpen(0);
        } else {
            std::cout << "No joysticks available." << std::endl;
        }
    }
    static float xyz[3] = {9.3812,4.5702,3.1600}; // {6.3286,-5.9263,1.7600};
    static float hpr[3] = {-142.5000,-34.5000,0.0000}; // {102.5000,-16.0000,0.0000};
    dsSetViewpoint(xyz,hpr);
}

void step(int pause) {
    if(environment->config.joystick.enabled) {
        SDL_Event event;
        while(SDL_PollEvent(&event)) {
            switch(event.type) {
                case SDL_JOYAXISMOTION:
                    std::cout << "joystick, axis " << static_cast<int>(event.jaxis.axis) << ", value " << event.jaxis.value << std::endl;
                    //environment->v->setTrackVelocities(joy_r,joy_l);
                    break;
            }
        }
    }
    environment->draw();
    if(!pause) environment->step();
}

void stop() {
    if(environment->config.joystick.enabled) {
        SDL_JoystickClose(joystick);
        SDL_JoystickEventState(SDL_DISABLE);
    }
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
    if(SDL_Init(SDL_INIT_JOYSTICK)) {
        std::cout << "SDL initialization failed." << std::endl;
        return 1;
    }
    SDL_GameControllerAddMappingsFromFile(CONFIG_PATH "/gamecontrollerdb.txt");
    
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
    
    SDL_Quit();

    return 0;
}
