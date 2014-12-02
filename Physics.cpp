//
//  Physics.cpp
//  tvs
//
//  Created by Federico Ferri on 01/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Physics.h"

#include <iostream>
#include <cstring>

Physics::Physics() : track(0.2, 0.15, 0.6, 30, 0.01, 0.06, 0.18) {
    stepNum = 0;
}

void Physics::init() {
    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);
    
    world = dWorldCreate();
    space = dSimpleSpaceCreate(0);
    contactGroup = dJointGroupCreate(0);
    dWorldSetGravity(world, 0, -9.81 * 0.2, 0);
    dWorldSetERP(world, 0.2);
    dWorldSetCFM(world, 1e-5);
    dWorldSetContactMaxCorrectingVel(world, 0.9);
    dWorldSetContactSurfaceLayer(world, 0.001);
    dWorldSetAutoDisableFlag(world, 1);
    
    planeGeom = dCreatePlane(space, 0, 1, 0, 0); // (a, b, c)' (x, y, z) = d
    
    track.createAll(world, space);
}

void Physics::destroy() {
    dJointGroupDestroy(contactGroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
}

void Physics::nearCallbackWrapper(void *data, dGeomID o1, dGeomID o2) {
    if(data)
        ((Physics *)data)->nearCallback(o1, o2);
}

void Physics::nearCallback(dGeomID o1, dGeomID o2) {
    int i;
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    dContact contact[MAX_CONTACTS];
    for(i = 0; i < MAX_CONTACTS; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = 1; //dInfinity;
        contact[i].surface.mu2 = 0;
        contact[i].surface.bounce = 0.5;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.001;
        
        contactsCache.push_back(contact[i].geom);
    }
    if(int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact))) {
        for(i = 0; i < numc; i++) {
            dJointID c = dJointCreateContact(world, contactGroup, contact + i);
            dJointAttach(c, b1, b2);
        }
    }
}

void Physics::step() {
    std::cout << "about to perform simulation step " << stepNum << std::endl;
    
    contactsCache.clear();
    
    track.rotate(0.0005);
    
    dSpaceCollide(space, this, &Physics::nearCallbackWrapper);
    dWorldQuickStep(world, 0.05);
    dJointGroupEmpty(contactGroup);
    
    track.test();
    
    stepNum++;
}