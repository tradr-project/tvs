//
//  world.c
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

World * world_init() {
    World *w = (World *)malloc(sizeof(World));

    PointCloud *pcl_full = point_cloud_read("pcd_0000.ds.0.3.xyz");
    w->pcl = point_cloud_filter_far(pcl_full, center, limit);
    point_cloud_destroy(pcl_full);
    w->pcl->point_radius = 0.3 * sqrt(3) / 2.0;

    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);

    w->v = tracked_vehicle_init(0.3, 0.8, 0.2, 0.5, 0, 0, 0.301+0.4);

    return w;
}

void world_create(World *w) {
    w->world = dWorldCreate();
    w->space = dQuadTreeSpaceCreate(0, center, extents, 6);
    w->contactGroup = dJointGroupCreate(0);
    dWorldSetGravity(w->world, 0, 0, -9.81);
    //dWorldSetERP(w->world, 0.7);
    //dWorldSetCFM(w->world, 1e-5);
    //dWorldSetContactMaxCorrectingVel(w->world, 0.9);
    //dWorldSetContactSurfaceLayer(w->world, 0.001);
    dWorldSetAutoDisableFlag(w->world, 1);

    w->planeGeom = dCreatePlane(w->space, 0, 0, 1, 0); // (a, b, c)' (x, y, z) = d
    dGeomSetCategoryBits(w->planeGeom, 0x4);
    dGeomSetCollideBits(w->planeGeom, 0x2);

    tracked_vehicle_create(w->v, w->world, w->space);

    point_cloud_create_geom(w->pcl, w->world, w->space);
}

static int is_terrain(dGeomID g) {
    return dGeomGetClass(g) == dPlaneClass
        || dGeomGetClass(g) == dHeightfieldClass
        || dGeomGetClass(g) == dSphereClass;
}

static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
    World *w = (World *)data;
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
        
        dJointID c = dJointCreateContact(w->world, w->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void world_step(World *w, dReal stepSize, int simulationStepsPerFrame) {
    size_t i;
    for(i = 0; i < simulationStepsPerFrame; i++) {
        // find collisions and add contact joints
        dSpaceCollide(w->space, w, &nearCallback);
        // step the simulation
        dWorldQuickStep(w->world, stepSize / (dReal)simulationStepsPerFrame);
        // remove all contact joints
        dJointGroupEmpty(w->contactGroup);
    }
}

void world_destroy(World *w) {
    point_cloud_destroy(w->pcl);
    tracked_vehicle_destroy(w->v);
}

void world_deinit(World *w) {
    point_cloud_deinit(w->pcl);
    tracked_vehicle_deinit(w->v);

    dJointGroupDestroy(w->contactGroup);
    dSpaceDestroy(w->space);
    dWorldDestroy(w->world);
    dCloseODE();

    free(w);
}

void world_draw(World *w) {
    point_cloud_draw(w->pcl);
    tracked_vehicle_draw(w->v);
}

