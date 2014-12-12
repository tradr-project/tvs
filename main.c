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

#define MAX_CONTACTS 10

dWorldID world;
dSpaceID space;
dJointGroupID contactGroup;

dGeomID planeGeom;

TrackedVehicle *v;
//Heightfield *hf;
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

        contact[i].surface.bounce = 0.5;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.0001;
    }
    int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
    for(i = 0; i < numc; i++) {
        dJointID c = dJointCreateContact(world, contactGroup, contact + i);
        dJointAttach(c, b1, b2);
    }
}

void start() {
    static float xyz[3] = {2.0f,-2.0f,1.7600f};
    static float hpr[3] = {140.000f,-17.0000f,0.0000f};
    dsSetViewpoint(xyz,hpr);
}

const int simulationStepsPerFrame = 4;
int nstep = 0;

void draw() {
    tracked_vehicle_draw(v);
    //heightfield_draw(hf);
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
    switch(cmd) {
        case 'a':
            dJointSetHingeParam(v->rightTrack->wheel2Joint, dParamVel, V);
            dJointSetHingeParam(v->leftTrack->wheel2Joint, dParamVel, -V);
            break;
        case 'd':
            dJointSetHingeParam(v->rightTrack->wheel2Joint, dParamVel, -V);
            dJointSetHingeParam(v->leftTrack->wheel2Joint, dParamVel, V);
            break;
        case 'w':
            dJointSetHingeParam(v->rightTrack->wheel2Joint, dParamVel, -V);
            dJointSetHingeParam(v->leftTrack->wheel2Joint, dParamVel, -V);
            break;
        case 's':
            dJointSetHingeParam(v->rightTrack->wheel2Joint, dParamVel, V);
            dJointSetHingeParam(v->leftTrack->wheel2Joint, dParamVel, V);
            break;
        case ' ':
            dJointSetHingeParam(v->rightTrack->wheel2Joint, dParamVel, 0);
            dJointSetHingeParam(v->leftTrack->wheel2Joint, dParamVel, 0);
            break;
        default:
            printf("cmd=%d\n", cmd);
            break;
    }
}

int main(int argc, char **argv) {
    const dVector3 center = {3,3,0};
    const dReal limit = 4.0;
    PointCloud *pcl_full = point_cloud_read("pcd_0000.ds.0.3.xyz");
    pcl = point_cloud_filter_far(pcl_full, center, limit);
    point_cloud_destroy(pcl_full);
    pcl->point_radius = 0.13;

    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);

    world = dWorldCreate();
    
    //space = dSimpleSpaceCreate(0);
    
    space = dHashSpaceCreate(0);
    
    //const dVector3 extents = {7,7,7};
    //space = dQuadTreeSpaceCreate(0, center, extents, 6);
    
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

    v = tracked_vehicle_create(world, space, 0.3, 0.8, 0.2, 0.5, 0, 0, 0.301+0.4);
    //hf = heightfield_create(world, space, 4.0, 8.0, 15, 31, 0.4);
    point_cloud_create_geom(pcl, world, space);

    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &step;
    fn.stop = &stop;
    fn.command = &command;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    dsSimulationLoop(argc, argv, 800, 600, &fn);

    tracked_vehicle_destroy(v);
    //heightfield_destroy(hf);
    point_cloud_destroy(pcl);

    dJointGroupDestroy(contactGroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    
    free(pcl);

    return 0;
}

