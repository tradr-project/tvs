//
//  track.c
//  tvs
//
//
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <drawstuff/drawstuff.h>
#include "track.h"
#include "track_kinematic_model.h"
#include "backflip.h"



//TODO: Rendi i giunti dei motori

BackFlip * back_flip_init(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserHeight_, dReal flipDepth_, dReal xOffset, dReal yOffset, dReal zOffset) {
    BackFlip *t = (BackFlip *)malloc(sizeof(BackFlip));
    t->m = track_kinematic_model_init(radius1_, radius2_, distance_, numGrousers_, grouserHeight_, 0, flipDepth_);
    t->density = 1.0;
    t->grouserBody = (dBodyID *)malloc(numGrousers_ * sizeof(dBodyID));
    t->grouserGeom = (dGeomID *)malloc(numGrousers_ * sizeof(dGeomID));
    t->grouserJoint = (dJointID *)malloc(numGrousers_ * sizeof(dJointID));
    t->grouserMass = (dMass *)malloc(numGrousers_ * sizeof(dMass));
    t->xOffset = xOffset;
    t->yOffset = yOffset;
    t->zOffset = zOffset;
    return t;

}

void back_flip_create(BackFlip * f, Track *t, dWorldID world, dSpaceID space){


	f->flipBody = dBodyCreate(world);
    dMassSetBox(&f->flipMass, f->density, f->m->distance, f->m->radius2, f->m->flipDepth);
    dBodySetMass(f->flipBody, &f->flipMass);

    // Planes

       dReal planeYCompliance = 0.015;

       //first couple

       dReal track_l = f->m->distance+(f->m->radius1+f->m->radius2)+0.20;
       dMatrix3 planeR;
       dRSetIdentity(planeR);
       f->planeGeom1 = dCreateBox(space,track_l,0.01,2*(f->m->radius1+f->m->radius2)); // era 1.25
       dGeomSetBody(f->planeGeom1,f->flipBody);
       dGeomSetCategoryBits(f->planeGeom1,0x10);
       dGeomSetCollideBits(f->planeGeom1,0x2);
       //dGeomSetOffsetPosition(f->planeGeom,f->xOffset+f->m->distance/2,f->yOffset+f->m->trackDepth/2,f->zOffset);
       dGeomSetOffsetPosition(f->planeGeom1,f->xOffset+f->m->distance/2,f->yOffset-(f->m->flipDepth/2+planeYCompliance),f->zOffset);
       dGeomSetRotation(f->planeGeom1,planeR);

       // second couple

       //dReal track_l = f->m->distance+(f->m->radius1+f->m->radius2)+0.10;
       //dMatrix3 planeR;
       //dRSetIdentity(planeR);
       f->planeGeom2 = dCreateBox(space,track_l,0.01,2*(f->m->radius1+f->m->radius2));
       dGeomSetBody(f->planeGeom2,f->flipBody);
       dGeomSetCategoryBits(f->planeGeom2,0x10);
       dGeomSetCollideBits(f->planeGeom2,0x2);
       dGeomSetOffsetPosition(f->planeGeom2,f->xOffset+f->m->distance/2,f->yOffset+f->m->flipDepth/2+planeYCompliance,f->zOffset);
       dGeomSetRotation(f->planeGeom2,planeR);

    // Wheel 1

    f->wheel1Geom = dCreateCylinder(space, f->m->radius1, f->m->flipDepth);
	dGeomSetCategoryBits(f->wheel1Geom, 0x9);
	dGeomSetCollideBits(f->wheel1Geom, 0x2);
	dMassSetCylinder(&f->wheel1Mass, f->density, 3, f->m->radius1, f->m->flipDepth);
	f->wheel1Body = dBodyCreate(world);
	dBodySetMass(f->wheel1Body, &f->wheel1Mass);
	dGeomSetBody(f->wheel1Geom, f->wheel1Body);
	//dGeomSetBody(f->wheel1Geom,f->wheel1Body);
	dBodySetPosition(f->wheel1Body, f->xOffset, f->yOffset, f->zOffset);
	dMatrix3 wheel1R;
	dRFromZAxis(wheel1R, 0, 1, 0);
	dBodySetRotation(f->wheel1Body, wheel1R);
	f->wheel1Joint = dJointCreateHinge(world, 0);
	//f->wheel1Joint = dJointCreateAMotor(world,0);
	dJointAttach(f->wheel1Joint, f->flipBody, f->wheel1Body);

	dJointSetHingeAnchor(f->wheel1Joint, f->xOffset, f->yOffset, f->zOffset);
	dJointSetHingeAxis(f->wheel1Joint, 0, 1, 0);
    dJointSetHingeParam(f->wheel1Joint,dParamBounce,0);



	// Wheel 2

    f->wheel2Geom = dCreateCylinder(space, f->m->radius2, f->m->flipDepth);
    dGeomSetCategoryBits(f->wheel2Geom, 0x9);
    dGeomSetCollideBits(f->wheel2Geom, 0x2);
    dMassSetCylinder(&f->wheel2Mass, f->density, 3, f->m->radius2, f->m->flipDepth);
    f->wheel2Body = dBodyCreate(world);
    dBodySetMass(f->wheel2Body, &f->wheel2Mass);
    dGeomSetBody(f->wheel2Geom, f->wheel2Body);
	//dGeomSetBody(f->wheel2Geom,t->wheel2Body);

    dBodySetPosition(f->wheel2Body, f->xOffset+f->m->distance, f->yOffset, f->zOffset);
    dMatrix3 wheel2R;
    dRFromZAxis(wheel2R, 0, 1, 0);
    dBodySetRotation(f->wheel2Body, wheel2R);
    f->wheel2Joint = dJointCreateHinge(world, 0);
	//f->wheel2Joint = dJointCreateAMotor(world,0);
    dJointAttach(f->wheel2Joint, f->flipBody, f->wheel2Body);
	//f->wheel2Joint = dJointCreateAMotor(world,0);
	//dJointAttach(f->wheel2Joint, t->trackBody, f->wheel2Body);
    dJointSetHingeAnchor(f->wheel2Joint, f->xOffset+f->m->distance, f->yOffset, f->zOffset);
	dJointSetHingeAxis(f->wheel2Joint, 0, 1, 0);
    dJointSetHingeParam(f->wheel2Joint, dParamFMax, 10);
    dJointSetHingeParam(f->wheel2Joint,dParamBounce,0);


    // grouser shrink/grow factor
    const dReal gf = 1.03;
    size_t i;
    const dReal grouser_shift=0;
    for(i = 0; i < f->m->numGrousers; i++) {
        f->grouserGeom[i] = dCreateBox(space, f->m->grouserHeight, f->m->flipDepth, gf * f->m->grouserWidth);
        dGeomSetCategoryBits(f->grouserGeom[i], 0x2);
        dGeomSetCollideBits(f->grouserGeom[i], 0x1 | 0x4 | 0x10);
        dMassSetBox(&f->grouserMass[i], 10 * f->density, f->m->grouserHeight, f->m->flipDepth, gf * f->m->grouserWidth);
        f->grouserBody[i] = dBodyCreate(world);
        dBodySetMass(f->grouserBody[i], &f->grouserMass[i]);
        dGeomSetBody(f->grouserGeom[i], f->grouserBody[i]);
        dVector3 pos; dMatrix3 R;
        track_kinematic_model_compute_grouser_transform_3D(f->m, i, pos, R);
        dBodySetPosition(f->grouserBody[i], f->xOffset + pos[0]+grouser_shift, f->yOffset + pos[1], f->zOffset + pos[2]);
        dBodySetRotation(f->grouserBody[i], R);

    }

    for(i = 0; i < f->m->numGrousers; i++) {
        size_t j = (i + 1) % f->m->numGrousers;
        dReal px, pz, qx, qz, a, dx, dz;
        track_kinematic_model_get_point_on_path(f->m, i / (dReal)f->m->numGrousers, &px, &pz, &a);
        dx = cos(a - M_PI_2);
        dz = sin(a - M_PI_2);
        qx = px - f->m->grouserWidth * gf * 0.5 * dx;
        qz = pz - f->m->grouserWidth * gf * 0.5 * dz;
        px = px + f->m->grouserWidth * gf * 0.5 * dx;
        pz = pz + f->m->grouserWidth * gf * 0.5 * dz;
        f->grouserJoint[i] = dJointCreateHinge(world, 0);
        dJointAttach(f->grouserJoint[i], f->grouserBody[i], f->grouserBody[j]);
        dJointSetHingeAnchor(f->grouserJoint[i], f->xOffset + px, f->yOffset, f->zOffset + pz);
        dJointSetHingeAxis(f->grouserJoint[i], 0, 1, 0);

    }


}

void back_flip_destroy(BackFlip *t){

}

void back_flip_deinit(BackFlip *t){
    track_kinematic_model_deinit(t->m);
    free(t->grouserBody);
    free(t->grouserGeom);
    free(t->grouserJoint);
    free(t->grouserMass);
    free(t);
}

void back_flip_draw(BackFlip *t, int draw_planes){
	{
    const dReal *pos = dGeomGetPosition(t->wheel1Geom);
    const dReal *R = dGeomGetRotation(t->wheel1Geom);
    dReal radius, length;
    dGeomCylinderGetParams(t->wheel1Geom, &radius, &length);
    dsDrawCylinderD(pos, R, length, radius);
	}

{

    const dReal *pos = dGeomGetPosition(t->wheel2Geom);
    const dReal *R = dGeomGetRotation(t->wheel2Geom);
    dReal radius, length;
    dGeomCylinderGetParams(t->wheel2Geom, &radius, &length);
    dsDrawCylinderD(pos, R, length, radius);
}

size_t i;
for(i = 0; i < t->m->numGrousers; i++) {
    const dReal *pos = dGeomGetPosition(t->grouserGeom[i]);
    const dReal *R = dGeomGetRotation(t->grouserGeom[i]);
    dReal sides[3];
    dGeomBoxGetLengths(t->grouserGeom[i], sides);
    dsDrawBoxD(pos, R, sides);
}
// No need to draw the planes

if(draw_planes){
	{ // first couple of planes

		const dReal *pos = dGeomGetPosition(t->planeGeom1);
		const dReal *R = dGeomGetRotation(t->planeGeom1);
		dReal sides[3];
		dGeomBoxGetLengths(t->planeGeom1,sides);
		dsDrawBoxD(pos,R,sides);
	}

	{ // second couple of planes
		const dReal *pos = dGeomGetPosition(t->planeGeom2);
		const dReal *R = dGeomGetRotation(t->planeGeom2);
		dReal sides[3];
		dGeomBoxGetLengths(t->planeGeom2,sides);
		dsDrawBoxD(pos,R,sides);
	}
	}

}

