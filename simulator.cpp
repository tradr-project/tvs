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
dReal vel_left = 0.0, vel_right = 0.0;
bool following = false;
dReal kbd_gain = 1.0;

void initRobotPose() {
    static dVector3 p = {2.08086,3.39581,0.102089};
    static dQuaternion q = {-0.229659,-0.00088334,0.00010361,0.973271};
    environment->v->setPosition(p);
    environment->v->setQuaternion(q);
    vel_left = vel_right = 0.0;
#ifdef USE_TRACKED_VEHICLE
    environment->v->leftTrack->velocity.reset();
    environment->v->rightTrack->velocity.reset();
#else // SKID STEERING VEHICLE:
    environment->v->velocityLeft.reset();
    environment->v->velocityRight.reset();
#endif
}

void follow(dReal x, dReal y, dReal z) {
    float xyz[3], hpr[3];
    dsGetViewpoint(xyz, hpr);
    
    // ~lookAt:
    dReal dx = x - xyz[0], dy = y - xyz[1], dz = z - xyz[2];
    hpr[0] = atan2(dy, dx) * 180.0 / M_PI;
    hpr[1] = atan2(dz, hypot(dy, dx)) * 180.0 / M_PI;
    hpr[2] = 0;

    // ~followRobot:
    dReal d = hypot(dz, hypot(dx, dy));
    dReal dmin = 3.5, dmax = 4.3, k = 0.2;
    if(d > dmax) {
        dReal a = (d - dmax) * k;
        xyz[0] += dx * a;
        xyz[1] += dy * a;
        //xyz[2] += dz * a;
    } else if(d < dmin) {
        dReal a = (d - dmin) * k;
        xyz[0] += dx * a;
        xyz[1] += dy * a;
        //xyz[2] += dz * a;
    }
    
    dsSetViewpoint(xyz,hpr);
}

void start() {
    if(environment->config.joystick.enabled) {
        if(SDL_NumJoysticks() > 0) {
            std::cout << "Available joysticks:" << std::endl;
            for(int i=0; i < SDL_NumJoysticks(); i++)
                std::cout << i << ": " << SDL_JoystickNameForIndex(i) << std::endl;
            SDL_JoystickEventState(SDL_ENABLE);
            joystick = SDL_JoystickOpen(environment->config.joystick.device);
        } else {
            std::cout << "No joysticks available." << std::endl;
        }
    }
    static float xyz[3] = {9.3812,4.5702,2.8600}; // {6.3286,-5.9263,1.7600};
    static float hpr[3] = {-142.5000,-34.5000,0.0000}; // {102.5000,-16.0000,0.0000};
    dsSetViewpoint(xyz,hpr);
}

void step(int pause) {
    if(environment->config.joystick.enabled) {
        SDL_Event event;
        while(SDL_PollEvent(&event)) {
            switch(event.type) {
                case SDL_JOYAXISMOTION:
                    const dReal V = environment->config.world.max_track_speed * environment->config.joystick.gain;
                    if(event.jaxis.axis == 1)
                        vel_right = V * event.jaxis.value / 32768.0;
                    if(event.jaxis.axis == 3)
                        vel_left = V * event.jaxis.value / 32768.0;
                    //std::cout << "    " << joy_l << "    " << joy_r << std::endl;
                    break;
            }
        }
    }
    if(following) {
        const dReal *p = environment->v->getPosition();
        follow(p[0], p[1], p[2]);
    }
#ifdef USE_TRACKED_VEHICLE
    environment->v->setTrackVelocities(kbd_gain * vel_left, kbd_gain * vel_right);
#else // SKID STEERING VEHICLE:
    environment->v->setWheelVelocities(kbd_gain * vel_left, kbd_gain * vel_right);
#endif
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
        case 'd': vel_left = V; vel_right = -V; break;
        case 'a': vel_left = -V; vel_right = V; break;
        case 'w': vel_left = -V; vel_right = -V; break;
        case 's': vel_left = V; vel_right = V; break;
        case 'e': vel_left *= 0.33; break;
        case 'q': vel_right *= 0.33; break;
        case ' ': vel_left = 0; vel_right = 0; break;
        case 'f': following ^= 1; break;
        case 'c': environment->config.show_contact_points ^= 1; break;
        case 'p': printInfo(); break;
        case 'r': initRobotPose(); break;
    }
    if(cmd >= '1' && cmd <= '9') kbd_gain = (cmd - '1' + 1) / 5.0;
}

int main(int argc, char **argv) {
    if(SDL_Init(SDL_INIT_JOYSTICK)) {
        std::cout << "SDL initialization failed: " << SDL_GetError() << std::endl;
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
