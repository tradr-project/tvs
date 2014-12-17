//
//  world.cpp
//  tvs
//
//  Created by Federico Ferri on 17/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "world.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <drawstuff/drawstuff.h>

#define MAX_CONTACTS 10

static const dVector3 center = {3,3,0};
static const dVector3 extents = {7,7,7};
static const dReal limit = 8.0;

World::World() {
    this->v = new TrackedVehicle(0.3, 0.8, 0.2, 0.5, 0, 0, 0.301+0.4);
    this->pcl = new PointCloud("pcd_0000.ds.0.3.xyz");
    this->pcl->filterFar(center, limit);
    this->pcl->point_radius = 0.3 * sqrt(3) / 2.0;

    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);
}

World::~World() {
    dJointGroupDestroy(this->contactGroup);
    dSpaceDestroy(this->space);
    dWorldDestroy(this->world);
    dCloseODE();
}

void World::create() {
    this->world = dWorldCreate();
    this->space = dQuadTreeSpaceCreate(0, center, extents, 6);
    this->contactGroup = dJointGroupCreate(0);
    dWorldSetGravity(this->world, 0, 0, -9.81);
    //dWorldSetERP(this->world, 0.7);
    //dWorldSetCFM(this->world, 1e-5);
    //dWorldSetContactMaxCorrectingVel(this->world, 0.9);
    //dWorldSetContactSurfaceLayer(this->world, 0.001);
    dWorldSetAutoDisableFlag(this->world, 1);

    this->planeGeom = dCreatePlane(this->space, 0, 0, 1, 0); // (a, b, c)' (x, y, z) = d
    dGeomSetCategoryBits(this->planeGeom, 0x4);
    dGeomSetCollideBits(this->planeGeom, 0x2);

    v->create(this);
    pcl->create(this);
}

void World::destroy() {
    v->destroy();
    pcl->destroy();
}

static inline bool is_terrain(dGeomID g) {
    return dGeomGetClass(g) == dPlaneClass
        || dGeomGetClass(g) == dHeightfieldClass
        || dGeomGetClass(g) == dSphereClass;
}

static void nearCallbackWrapper(void *data, dGeomID o1, dGeomID o2) {
    reinterpret_cast<World *>(data)->nearCallback(o1, o2);
}

void World::nearCallback(dGeomID o1, dGeomID o2) {
    int i;
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    dContact contact[MAX_CONTACTS];
    for(i = 0; i < MAX_CONTACTS; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactMu2 | dContactFDir1;
        contact[i].surface.bounce = 0.5;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.0001;
    }
    int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
    for(i = 0; i < numc; i++) {
        const dReal *v;
        
        assert(dGeomGetClass(o1) == dBoxClass ||
               dGeomGetClass(o2) == dBoxClass);
        
        if (dGeomGetClass(o1) == dBoxClass)
            v = dBodyGetLinearVel(b1);
        else
            v = dBodyGetLinearVel(b2);
        
        dCalcVectorCross3(contact[i].fdir1, contact[i].geom.normal, v);
        dSafeNormalize3(contact[i].fdir1);
        
        if (is_terrain(o1) || is_terrain(o2)) {
            contact[i].surface.mu = 2.0*2.618;
            contact[i].surface.mu2 = 0.5*2.618;
        } else if (dGeomGetClass(o1) == dCylinderClass ||
                   dGeomGetClass(o2) == dCylinderClass) {
            contact[i].surface.mu = contact[i].surface.mu2 = dInfinity;
        } else {
            printf ("%d, %d\n", dGeomGetClass(o1), dGeomGetClass(o2));
            assert(0);
        }
        
        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void World::step(dReal stepSize, int simulationStepsPerFrame) {
    size_t i;
    for(i = 0; i < simulationStepsPerFrame; i++) {
        // find collisions and add contact joints
        dSpaceCollide(this->space, this, &nearCallbackWrapper);
        // step the simulation
        dWorldQuickStep(this->world, stepSize / (dReal)simulationStepsPerFrame);
        // remove all contact joints
        dJointGroupEmpty(this->contactGroup);
    }
}

void World::draw() {
    this->pcl->draw();
    this->v->draw();
}

