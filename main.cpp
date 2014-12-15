//
//  main.c
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
#include "tracked_vehicle.h"
#include "point_cloud.h"

#define SIMPLE              1
#define HASH                2
#define QUADTREE            3
#define SPACE_TYPE          QUADTREE
#define MAX_CONTACTS        10

dWorldID world;
dSpaceID space;
dJointGroupID contactGroup;

dGeomID planeGeom;

TrackedVehicle *v;
PointCloud *pcl;

int is_terrain(dGeomID o) {
    return dGeomGetClass(o) == dPlaneClass
        || dGeomGetClass(o) == dHeightfieldClass
        || dGeomGetClass(o) == dSphereClass;
}

void nearCallback(void *data, dGeomID o1, dGeomID o2) {
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
        
        dJointID c = dJointCreateContact(world, contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void start() {
    static float xyz[3] = {6.3286,-5.9263,1.7600};
    static float hpr[3] = {102.5000,-16.0000,0.0000};
    dsSetViewpoint(xyz,hpr);
}

const int simulationStepsPerFrame = 4;
int nstep = 0;

void draw() {
    tracked_vehicle_draw(v);
    point_cloud_draw(pcl);
}

void step(int pause) {
    draw();

    if(!pause) {
        size_t i;
        for(i = 0; i < simulationStepsPerFrame; i++) {
            // find collisions and add contact joints
            dSpaceCollide(space, 0, &nearCallback);
            // step the simulation
            dWorldQuickStep(world, 0.01 / (dReal)simulationStepsPerFrame);
            // remove all contact joints
            dJointGroupEmpty(contactGroup);
        }
    }
}

void stop() {
}

void command(int cmd) {
    const dReal V = 5;

#define SetVel(trk,vv) dJointSetHingeParam(v->trk##Track->wheel2Joint, dParamVel, vv)
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
    
    const dVector3 center = {3,3,0};
    const dReal limit = 8.0;
    PointCloud *pcl_full = point_cloud_read("pcd_0000.ds.0.3.xyz");
    pcl = point_cloud_filter_far(pcl_full, center, limit);
    point_cloud_destroy(pcl_full);
    pcl->point_radius = 0.3 * sqrt(3) / 2.0;

    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);

    world = dWorldCreate();
    
#if SPACE_TYPE == SIMPLE
    space = dSimpleSpaceCreate(0);
#elif SPACE_TYPE == HASH
    space = dHashSpaceCreate(0);
#elif SPACE_TYPE == QUADTREE
    const dVector3 extents = {7,7,7};
    space = dQuadTreeSpaceCreate(0, center, extents, 6);
#else
#error Bad SPACE_TYPE
#endif
    
    contactGroup = dJointGroupCreate(0);
    
    dWorldSetGravity(world, 0, 0, -9.81);
    //dWorldSetERP(world, 0.7);
    //dWorldSetCFM(world, 1e-5);
    //dWorldSetContactMaxCorrectingVel(world, 0.9);
    //dWorldSetContactSurfaceLayer(world, 0.001);
    dWorldSetAutoDisableFlag(world, 1);

    planeGeom = dCreatePlane(space, 0, 0, 1, 0); // (a, b, c)' (x, y, z) = d
    dGeomSetCategoryBits(planeGeom, 0x4);
    dGeomSetCollideBits(planeGeom, 0x2);

    v = tracked_vehicle_init(0.3, 0.8, 0.2, 0.5, 0, 0, 0.301+0.4);
    tracked_vehicle_create(v, world, space);
    
    point_cloud_create_geom(pcl, world, space);

    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &step;
    fn.stop = &stop;
    fn.command = &command;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    dsSimulationLoop(argc, argv, 800, 600, &fn);

    tracked_vehicle_deinit(v);
    point_cloud_deinit(pcl);

    dJointGroupDestroy(contactGroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}

