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
#include <stdio.h>
#include "csvparser.h"

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

// Control parameters for the flippers
int position_mode=0;
int y_pressed=0;
int h_pressed=0;
int u_pressed=0;
int j_pressed=0;
int i_pressed=0;
int k_pressed=0;
int o_pressed=0;
int l_pressed=0;


// Python Script parameters
int test;
char* output_filename;

// Maneuvers indicators, if 1 the maneuver is in progress/started/finished
int maneuver_in_progress=0;
int maneuver_started=0;

// String of the maneuver to perform {a1/a2/b1/...} and its duration (5s or 10s)
char* maneuver_to_perform;
float maneuver_duration;

// Vehicle parameters
const dReal wheelRadius=0.2; //0.3 // 0.073
const dReal wheelBase=0.8; //0.8 //0.5
const dReal flipWheelRadius=0.055; // 0.075 //0.016
const dReal flipWheelBase=0.8; //0.8 //0.33
const dReal trackWidth=0.2; //0.2 //0.092
const dReal flipWidth = 0.15; //0.15 //0.049
const dReal vehicleWidth = 0.5; //0.5 //0.241
const dReal xOffset=0;
const dReal yOffset=0;
const dReal zOffset=0.301; //0.301

// CFM, ERP, u parameter array of all the possible values

const float CFM[10] = {1e-9,1e-8,1e-7,1e-6,1e-5,1e-4,1e-3,1e-3,1e-1,1};
int cfm = 4;

const float ERP[10] = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8};
int erp = 4;

const float mux[5] = {2.0*2.618/4.,2.0*2.618/2.,2.0*2.618,2.0*2.618*2,2.0*2.618*4};
int mu_x=2;

const float muy[5] = {0.25*2.618/4.,0.25*2.618/2.,0.25*2.618,0.25*2.618*2,0.25*2.618*4};
int mu_y=2;

// variables to store the parameters given as input
float CFM_v;
float ERP_v;
float ux_v;
float uy_v;

// scaling parameter of the velocities and the mass
float vel_scaling;
float mass_scaling;

// Current Rotation and Position variables
dReal *Rv;
dReal *pv;

// File pointer to the csv
FILE *fp_csv;

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
            contact[i].surface.mu = ux_v;
            contact[i].surface.mu2 = uy_v;

        } else if (dGeomGetClass(o1) == dCylinderClass ||
                   dGeomGetClass(o2) == dCylinderClass) {
            contact[i].surface.mu = contact[i].surface.mu2 = dInfinity;
        }
        else if (dGeomGetCategoryBits(o1) == 0x10 ||
        		 dGeomGetCategoryBits(o2) == 0x10){
            contact[i].surface.mu = contact[i].surface.mu2 = 0; // friction between the planes of the tracks
        }
        else if (dGeomGetCategoryBits(o1) == 0x8 ||
        		 dGeomGetCategoryBits(o2) == 0x8){
            contact[i].surface.mu = contact[i].surface.mu2 = 0; // friction between the planes of the flippers
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
    // if the simulator is in test mode immediately executes the input maneuver and writes the results on a csv file
    if(test==1){
		printf("writing data to %s\n",output_filename);
		fp_csv= fopen (output_filename, "w+");
    }
}

const int simulationStepsPerFrame = 12;
int nstep = 0;
double simstep = 0.01;

double simtime = 0.00;
double maneuvertime=0.0;
double initialwait = 0.6;

void draw() {
    tracked_vehicle_draw(v);
    //point_cloud_draw(pcl);
}

void step(int pause) {
    draw();
    if(!pause) {
        size_t i;
        for(i = 0; i < simulationStepsPerFrame; i++) {
            // find collisions and add contact joints
            dSpaceCollide(space, 0, &nearCallback);
            // step the simulation
            dWorldQuickStep(world, simstep / (dReal)simulationStepsPerFrame);
            // remove all contact joints
            dJointGroupEmpty(contactGroup);
        }
        simtime += simstep;

        /*
        const dReal* Rb = dBodyGetRotation(v->leftTrack->trackBody);
        const dReal* p = dBodyGetPosition(v->leftTrack->trackBody);

        printf("Posizione %f, %f, %f \n",p[0],p[1],p[2]);
        printf("Asse x: %f, %f, %f \n",Rb[0],Rb[4],Rb[8]);
        printf("Asse y: %f, %f, %f \n",Rb[1],Rb[5],Rb[9]);
        printf("Asse z: %f, %f, %f \n",Rb[2],Rb[6],Rb[10]);
		*/

        if(simtime >= initialwait && !maneuver_in_progress) {
            fprintf(stdout,"start record!\n");
            doManeuver(maneuver_to_perform);
        }


        if(maneuver_started==1){
            Rv = dBodyGetRotation(v->vehicleBody);
            pv = dBodyGetPosition(v->vehicleBody);
            maneuver_started=0;
            printf("Initial position: %f, %f, %f\n",pv[0],pv[1],pv[2]);
            fprintf(fp_csv,"%f,%f,%f,",pv[0],pv[1],pv[2]);
            fprintf(fp_csv,"%f,%f,%f\n",pv[0]+0.05,pv[1],pv[2]); // to track the orientation of the robot
        }
        if(maneuver_in_progress==1){
            maneuvertime += simstep;
            fprintf(fp_csv,"%f,%f,%f,",pv[0],pv[1],pv[2]);
            fprintf(fp_csv,"%f,%f,%f\n",pv[0]+0.05,pv[1],pv[2]); // to track the orientation of the robot
            printf(".");
            fflush(stdout);
        }
        if(maneuvertime>maneuver_duration && maneuver_in_progress==1){
              printf("\n");
              maneuver_in_progress=0;
              dJointSetHingeParam(v->leftTrack->wheel2Joint,dParamVel,0);
              dJointSetHingeParam(v->rightTrack->wheel2Joint,dParamVel,0);

              dJointSetHingeParam(v->leftFlip->wheel1Joint,dParamVel,0);
              dJointSetHingeParam(v->rightFlip->wheel1Joint,dParamVel,0);

              dJointSetHingeParam(v->leftBackFlip->wheel2Joint,dParamVel,0);
              dJointSetHingeParam(v->rightBackFlip->wheel2Joint,dParamVel,0);

              dJointSetHingeParam(v->leftFlipJoint,dParamVel,0);
              dJointSetHingeParam(v->leftBackFlipJoint,dParamVel,0);
              dJointSetHingeParam(v->rightFlipJoint,dParamVel,0);
              dJointSetHingeParam(v->rightBackFlipJoint,dParamVel,0);

              dReal *ep = dBodyGetPosition(v->vehicleBody);
              dReal *eRv = dBodyGetRotation(v->vehicleBody);
              printf("Ending position: %f, %f, %f\n",ep[0],ep[1],ep[2]);
              fprintf(fp_csv,"%f,%f,%f,",ep[0],ep[1],ep[2]);
              fprintf(fp_csv,"%f,%f,%f\n",pv[0]+0.05,pv[1],pv[2]);

              if(test==1){
                  printf("Simulation finished. \n");
                  exit(0);
              }
        }
    }
}

void stop() {
}

dReal computeLeftTrackVelocity(dReal v, dReal w, TrackedVehicle* tv, float scaling){
	// given a linear velocity (v) and an angular velocity (w) it computes the track velocity
	dReal r=tv->leftTrack->m->radius2;
	dReal d=tv->leftTrack->m->distance;
	dReal vl = scaling*((v-(3/2.)*w*d)/r);
	printf("vl: %f\n",vl);
	return vl;
}

dReal computeRightTrackVelocity(dReal v, dReal w, TrackedVehicle* tv, float scaling){
	// given a linear velocity (v) and an angular velocity (w) it computes the track velocity
	dReal r=tv->leftTrack->m->radius2;
	dReal d=tv->leftTrack->m->distance;
	dReal vr = scaling*((2*v-w*d)/(2*r));
	printf("vr: %f\n",vr);
	return vr;

}

void printCenter(TrackedVehicle* tv){
	dReal *pos = dBodyGetPosition(tv->vehicleBody);
	printf("X: %f, Y: %f, Z: %f",pos[0],pos[1],pos[2]);
}

void forward(dReal lV){
  // the robot moves forward
  dJointSetHingeParam(v->leftTrack->wheel2Joint,dParamVel,-lV);
  dJointSetHingeParam(v->rightTrack->wheel2Joint,dParamVel,-lV);
  // The back-wheel of the flips is wheel1
  dJointSetHingeParam(v->leftFlip->wheel1Joint,dParamVel,-lV);
  dJointSetHingeParam(v->rightFlip->wheel1Joint,dParamVel,-lV);
  dJointSetHingeParam(v->leftBackFlip->wheel2Joint,dParamVel,-lV);
  dJointSetHingeParam(v->rightBackFlip->wheel2Joint,dParamVel,-lV);
}

doManeuver(char* name){
	// executes the input maneuver scaled by vel_scaling
	dReal lin=0;
	dReal ang=0;
	maneuver_started=1;
	maneuver_in_progress = 1;

	if(strcmp(name,"e")==0){
		lin = 0.2;
		ang = 0;
	}
	else if(strcmp(name,"a1")==0){
		lin=M_PI/10;
		ang=M_PI/10;
	}
	else if(strcmp(name,"a2")==0){
		lin=M_PI/10;
		ang=-M_PI/10;
	}

	else if(strcmp(name,"b1")==0){
		lin=M_PI/10;
		ang=M_PI/5;
	}
	else if(strcmp(name,"b2")==0){
		lin=M_PI/10;
		ang=-M_PI/5;
	}
	else if(strcmp(name,"c1")==0){
		lin=0;
		ang=2*M_PI/5.;
	}
	else if(strcmp(name,"c2")==0){
		lin=0;
		ang=-2*M_PI/5.;
	}
	else if(strcmp(name,"d1")==0){
		lin=0;
		ang=2*M_PI/10.;
	}
	else if(strcmp(name,"d2")==0){
		lin=0;
		ang=-2*M_PI/10.;
	}
	else if(strcmp(name,"f1")==0){
		lin=M_PI/10;
		ang=M_PI/20;
	}
	else if(strcmp(name,"f2")==0){
		lin=M_PI/10;
		ang=-M_PI/20;
	}

	lin*=vel_scaling;
	ang*=vel_scaling;

	dReal lv = computeLeftTrackVelocity(lin,ang,v,1);
	dReal rv = computeRightTrackVelocity(lin,ang,v,1);

	dJointSetHingeParam(v->leftTrack->wheel2Joint,dParamVel,lv);
	dJointSetHingeParam(v->leftFlip->wheel1Joint,dParamVel,lv);
	dJointSetHingeParam(v->leftBackFlip->wheel2Joint,dParamVel,lv);

	dJointSetHingeParam(v->rightTrack->wheel2Joint,dParamVel,rv);
	dJointSetHingeParam(v->rightFlip->wheel1Joint,dParamVel,rv);
	dJointSetHingeParam(v->rightBackFlip->wheel2Joint,dParamVel,rv);
}

void command(int cmd) {
	// manual commands to move the robot
    const dReal lV = 3;
    const dReal aV = M_PI/2;
    double t;
    dReal lf,rf,lbf,rbf;

    switch(cmd){

    case 'z': doManeuver(maneuver_to_perform);
    		  //printCenter(v);
    		  break;


    case 'w': forward(lV);
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
    case '0': 	lf = dJointGetHingeAngle(v->leftFlipJoint);
    			rf = dJointGetHingeAngle(v->rightFlipJoint);
    			lbf = dJointGetHingeAngle(v->leftBackFlipJoint);
    			rbf = dJointGetHingeAngle(v->rightBackFlipJoint);
    			printf("%f,%f,%f,%f\n",lf,rf,lbf,rbf);
    			break;
    case '1':	setFlipAngle(v->leftFlipJoint,M_PI/2);
				setFlipAngle(v->rightFlipJoint,M_PI/2);
				setFlipAngle(v->leftBackFlipJoint,M_PI/2);
				setFlipAngle(v->rightBackFlipJoint,M_PI/2);
				break;

    case '2':	setFlipAngle(v->leftFlipJoint,M_PI/4);
				setFlipAngle(v->rightFlipJoint,M_PI/4);
				setFlipAngle(v->leftBackFlipJoint,M_PI/4);
				setFlipAngle(v->rightBackFlipJoint,M_PI/4);
				break;

    case 'y':
		  	  if(y_pressed==1){
		  		  dJointSetHingeParam(v->leftFlipJoint,dParamVel,0);
		  		  y_pressed=0;
		  	  }
		  	  else{
		  		  dJointSetHingeParam(v->leftFlipJoint,dParamVel,aV);
		  		  y_pressed=1;
		  	  }
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

void scale_mass(float scaling, TrackedVehicle* v){
	v->density *= scaling;

	v->leftTrack->density *= scaling;
	v->rightTrack->density *= scaling;

	v->leftFlip->density *= scaling;
	v->rightFlip->density *= scaling;

	v->leftBackFlip->density *= scaling;
	v->rightBackFlip->density *= scaling;
}

int main(int argc, char **argv) {
    //feenableexcept(FE_INVALID | FE_OVERFLOW);
	if(argc!=1 && strcmp(argv[1],"-test")==0){
		if(argc < 9){
			fprintf(stderr,"usage: -test <maneuver> <vel_scaling> <mass_scaling> <CFM> <ERP> <ux> <uy> <output_filename>\n");
			exit(0);
		}
		printf("You are in test mode. Testing maneuver %s \n",argv[2]);
		test=1;
		maneuver_to_perform = argv[2];
		vel_scaling = strtof(argv[3],NULL);
		mass_scaling = strtof(argv[4],NULL);
		CFM_v = strtof(argv[5],NULL);
		ERP_v = strtof(argv[6],NULL);
		ux_v = strtof(argv[7],NULL);
		uy_v = strtof(argv[8],NULL);
		output_filename = argv[9];

		if(strcmp(maneuver_to_perform,"d1")==0 || strcmp(maneuver_to_perform,"d2")==0){
			maneuver_duration=9.999;
		}
		else{
			maneuver_duration=4.999;
		}
		printf("maneuver duration: %f\n",maneuver_duration);
	}
	else{ // if not in the test mode the parameters are set to some default values of the arrays
		CFM_v = CFM[cfm];
		ERP_v = ERP[erp];
		ux_v = mux[mu_x];
		uy_v = muy[mu_y];
		vel_scaling = 1;
		mass_scaling = 1;
		maneuver_to_perform="e";
		maneuver_duration=4.999;
		output_filename = 0;
	}
    const dVector3 center = {3,3,0};
    const dReal limit = 8.0;

    // Point cloud stuff
    PointCloud *pcl_full = point_cloud_read("pcd_0000.ds.0.3.xyz");
    pcl = point_cloud_filter_far(pcl_full, center, limit);
    point_cloud_destroy(pcl_full);
    pcl->point_radius = 0.3 * sqrt(3) / 2.0;

    // ODE stuff

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
    dWorldSetERP(world, ERP_v);
    dWorldSetCFM(world, CFM_v);
    dWorldSetContactMaxCorrectingVel(world, 0.9);
    dWorldSetContactSurfaceLayer(world, 0.001);
    dWorldSetAutoDisableFlag(world, 1);

    planeGeom = dCreatePlane(space, 0, 0, 1, 0); // (a, b, c)' (x, y, z) = d
    dGeomSetCategoryBits(planeGeom, 0x4);
    dGeomSetCollideBits(planeGeom, 0x2);

    v = tracked_vehicle_init(wheelRadius,wheelBase,flipWheelRadius,flipWheelBase, trackWidth, flipWidth, vehicleWidth, xOffset, yOffset, zOffset);
    tracked_vehicle_create(v, world, space);
    scale_mass(mass_scaling,v);
    //point_cloud_create_geom(pcl, world, space);
    printf("density: %f\n",v->density);
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &step;
    fn.stop = &stop;
    fn.command = &command;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    dsSimulationLoop(argc, argv, 800, 600, &fn);

    tracked_vehicle_deinit(v);
    //point_cloud_deinit(pcl);

    dJointGroupDestroy(contactGroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}

