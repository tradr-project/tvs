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

int y_pressed=0;
int h_pressed=0;
int u_pressed=0;
int j_pressed=0;
int i_pressed=0;
int k_pressed=0;
int o_pressed=0;
int l_pressed=0;

// Vehicle parameters

const dReal wheelRadius=0.3;
const dReal wheelBase=0.8;
const dReal flipWheelRadius=0.075;
const dReal flipWheelBase=0.8;
const dReal trackWidth=0.2;
const dReal flipWidth = 0.15;
const dReal vehicleWidth = 0.5;
const dReal xOffset=-2;
const dReal yOffset=0;
const dReal zOffset=0.3; //0.301


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
            contact[i].surface.mu2 = 0.25*2.618;
        } else if (dGeomGetClass(o1) == dCylinderClass ||
                   dGeomGetClass(o2) == dCylinderClass) {
            contact[i].surface.mu = contact[i].surface.mu2 = dInfinity;
        }
        else if (dGeomGetCategoryBits(o1) == 0x10 ||
        		 dGeomGetCategoryBits(o2) == 0x10){
            contact[i].surface.mu = contact[i].surface.mu2 = 0;
        }
        else {
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

const int simulationStepsPerFrame = 5; // adatta
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

        const dReal* Rb = dBodyGetRotation(v->leftTrack->trackBody);
        const dReal* p = dBodyGetPosition(v->leftTrack->trackBody);

        /*
        printf("Posizione %f, %f, %f \n",p[0],p[1],p[2]);
        printf("Asse x: %f, %f, %f \n",Rb[0],Rb[4],Rb[8]);
        printf("Asse y: %f, %f, %f \n",Rb[1],Rb[5],Rb[9]);
        printf("Asse z: %f, %f, %f \n",Rb[2],Rb[6],Rb[10]);
		*/
    }
}

void stop() {
}

void command(int cmd) {
    const dReal lV = 3;
    const dReal aV = M_PI/2;

    switch(cmd){

    // Track % Flips Velocities

    case 'w': dJointSetHingeParam(v->leftTrack->wheel2Joint,dParamVel,-lV);
    		  dJointSetHingeParam(v->rightTrack->wheel2Joint,dParamVel,-lV);

    		  // The back-wheel of the flips is wheel1
    		  dJointSetHingeParam(v->leftFlip->wheel1Joint,dParamVel,-lV);
    		  dJointSetHingeParam(v->rightFlip->wheel1Joint,dParamVel,-lV);

    		  dJointSetHingeParam(v->leftBackFlip->wheel2Joint,dParamVel,-lV);
    		  dJointSetHingeParam(v->rightBackFlip->wheel2Joint,dParamVel,-lV);
    		  break;

    case 's': dJointSetHingeParam(v->leftTrack->wheel2Joint,dParamVel,lV);
    		  dJointSetHingeParam(v->rightTrack->wheel2Joint,dParamVel,lV);

    		  // The back-wheel of the flips is wheel1
    		  dJointSetHingeParam(v->leftFlip->wheel1Joint,dParamVel,lV);
    		  dJointSetHingeParam(v->rightFlip->wheel1Joint,dParamVel,lV);

    		  dJointSetHingeParam(v->leftBackFlip->wheel2Joint,dParamVel,lV);
    		  dJointSetHingeParam(v->rightBackFlip->wheel2Joint,dParamVel,lV);
    		  break;

    case 'd': dJointSetHingeParam(v->leftTrack->wheel2Joint,dParamVel,lV);
    		  dJointSetHingeParam(v->rightTrack->wheel2Joint,dParamVel,-lV);

    		  // The back-wheel of the flips is wheel1
    		  dJointSetHingeParam(v->leftFlip->wheel1Joint,dParamVel,lV);
    		  dJointSetHingeParam(v->rightFlip->wheel1Joint,dParamVel,-lV);

    		  dJointSetHingeParam(v->leftBackFlip->wheel2Joint,dParamVel,lV);
    		  dJointSetHingeParam(v->rightBackFlip->wheel2Joint,dParamVel,-lV);
    		  break;

    case 'a': dJointSetHingeParam(v->leftTrack->wheel2Joint,dParamVel,-lV);
    		  dJointSetHingeParam(v->rightTrack->wheel2Joint,dParamVel,lV);

    		  // The back-wheel of the flips is wheel1
    		  dJointSetHingeParam(v->leftFlip->wheel1Joint,dParamVel,-lV);
    		  dJointSetHingeParam(v->rightFlip->wheel1Joint,dParamVel,lV);

    		  dJointSetHingeParam(v->leftBackFlip->wheel2Joint,dParamVel,-lV);
    		  dJointSetHingeParam(v->rightBackFlip->wheel2Joint,dParamVel,lV);
    		  break;

    // Flip angle

    case 'y':
		  	  if(y_pressed==1){
		  		  dJointSetHingeParam(v->leftFlipJoint,dParamVel,0);
		  		  y_pressed=0;
		  	  }
		  	  else{
		  		  dJointSetHingeParam(v->leftFlipJoint,dParamVel,aV);
		  		  y_pressed=1;
		  	  }

    		  //dJointSetAMotorAngle(v->leftFlipJoint,0,M_PI/2);

    		  //dJointSetAMotorParam(v->leftFlipJoint,dParamVel,V);
    		  break;

    case 'h':
    			  if(h_pressed==1){
    		  		  dJointSetHingeParam(v->leftFlipJoint,dParamVel,+0);
    		  		  h_pressed=0;
    		  	  }
    		  	  else{
    		  		  dJointSetHingeParam(v->leftFlipJoint,dParamVel,-aV);
    		  		  h_pressed=1;
    		  	  }
    		 break;

    case 'u':
    			  if(u_pressed==1){
    		  		  dJointSetHingeParam(v->rightFlipJoint,dParamVel,+0);
    		  		  u_pressed=0;
    		  	  }
    		  	  else{
    		  		  dJointSetHingeParam(v->rightFlipJoint,dParamVel,aV);
    		  		  u_pressed=1;
    		  	  }
    		 break;

    case 'j':
    			  if(j_pressed==1){
    		  		  dJointSetHingeParam(v->rightFlipJoint,dParamVel,+0);
    		  		  j_pressed=0;
    		  	  }
    		  	  else{
    		  		  dJointSetHingeParam(v->rightFlipJoint,dParamVel,-aV);
    		  		  j_pressed=1;
    		  	  }
    		 break;


    case 'i':
		  	  if(i_pressed==1){
		  		  dJointSetHingeParam(v->leftBackFlipJoint,dParamVel,0);
		  		  i_pressed=0;
		  	  }
		  	  else{
		  		  dJointSetHingeParam(v->leftBackFlipJoint,dParamVel,-aV);
		  		  i_pressed=1;
		  	  }

    		  //dJointSetAMotorAngle(v->leftFlipJoint,0,M_PI/2);

    		  //dJointSetAMotorParam(v->leftFlipJoint,dParamVel,V);
    		  break;

    case 'k':
    			  if(k_pressed==1){
    		  		  dJointSetHingeParam(v->leftBackFlipJoint,dParamVel,+0);
    		  		  k_pressed=0;
    		  	  }
    		  	  else{
    		  		  dJointSetHingeParam(v->leftBackFlipJoint,dParamVel,aV);
    		  		  k_pressed=1;
    		  	  }
    		 break;

    case 'o':
		  	  if(o_pressed==1){
		  		  dJointSetHingeParam(v->rightBackFlipJoint,dParamVel,0);
		  		  o_pressed=0;
		  	  }
		  	  else{
		  		  dJointSetHingeParam(v->rightBackFlipJoint,dParamVel,-aV);
		  		  o_pressed=1;
		  	  }

    		  //dJointSetAMotorAngle(v->leftFlipJoint,0,M_PI/2);

    		  //dJointSetAMotorParam(v->leftFlipJoint,dParamVel,V);
    		  break;

    case 'l':
    			  if(l_pressed==1){
    		  		  dJointSetHingeParam(v->rightBackFlipJoint,dParamVel,+0);
    		  		  l_pressed=0;
    		  	  }
    		  	  else{
    		  		  dJointSetHingeParam(v->rightBackFlipJoint,dParamVel,aV);
    		  		  l_pressed=1;
    		  	  }
    		 break;


    case ' ': dJointSetHingeParam(v->leftTrack->wheel2Joint,dParamVel,0);
    		  dJointSetHingeParam(v->rightTrack->wheel2Joint,dParamVel,0);

    		  dJointSetHingeParam(v->leftFlip->wheel1Joint,dParamVel,0);
    		  dJointSetHingeParam(v->rightFlip->wheel1Joint,dParamVel,0);

    		  dJointSetHingeParam(v->leftBackFlip->wheel2Joint,dParamVel,0);
    		  dJointSetHingeParam(v->rightBackFlip->wheel2Joint,dParamVel,0);

    		  dJointSetHingeParam(v->leftFlipJoint,dParamVel,0);
    		  dJointSetHingeParam(v->leftBackFlipJoint,dParamVel,0);
    		  dJointSetHingeParam(v->rightFlipJoint,dParamVel,0);
    		  dJointSetHingeParam(v->rightBackFlipJoint,dParamVel,0);

    		  y_pressed=0;
    		  h_pressed=0;
    		  u_pressed=0;
    		  j_pressed=0;
    		  i_pressed=0;
    		  k_pressed=0;
    		  o_pressed=0;
    		  l_pressed=0;

    		  break;
    }
/*
#define SetVel(trk,vv) dJointSetHingeParam(v->trk##Track->wheel2Joint, dParamVel, vv)
#define SetFlipVel(flip,fvv) dJointSetUniversalParam(v->leftFlipJoint,dParamVel,fvv)
//#define MapKey(k,vr,vl) case k: SetVel(right, vr); SetVel(left, vl); break;
#define MapKey(k,vr,vl) case k: SetVel(right, vr); SetVel(left, vl); SetFlipVel(right,vr);SetFlipVel(left,vl); break;
//#define MapKey(k,vr,vl) case k: SetFlipVel(right,vr);SetFlipVel(left,vl); break;
#define FlipMapKey(k,vfr,vfl) case k: SetFlipVel(right,vfr); SetFlipVel(left,vfl); break;

    switch(cmd) {
    MapKey('a',  V, -V);
    MapKey('d', -V,  V);
    MapKey('w', -V, -V);
    MapKey('s',  V,  V);
    MapKey('q',  0, -V);
    MapKey('e', -V,  0);
    MapKey(' ',  0,  0);
    FlipMapKey('u', 0,-V);
    //FlipMapKey(' ',0,0);

    }
    
#undef MapKey
#undef SetVel*/
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
    dWorldSetCFM(world, 1e-5);
    //dWorldSetContactMaxCorrectingVel(world, 0.9);
    //dWorldSetContactSurfaceLayer(world, 0.001);
    dWorldSetAutoDisableFlag(world, 1);

    planeGeom = dCreatePlane(space, 0, 0, 1, 0); // (a, b, c)' (x, y, z) = d
    dGeomSetCategoryBits(planeGeom, 0x4);
    dGeomSetCollideBits(planeGeom, 0x2);

    v = tracked_vehicle_init(wheelRadius,wheelBase,flipWheelRadius,flipWheelBase, trackWidth, flipWidth, vehicleWidth, xOffset, yOffset, zOffset);
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

