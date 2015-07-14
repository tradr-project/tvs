//
//  tracked_vehicle.c
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "tracked_vehicle.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <drawstuff/drawstuff.h>

TrackedVehicle * tracked_vehicle_init(dReal wheelRadius_, dReal wheelBase_, dReal flipWheelRadius, dReal flipWheelBase, dReal trackWidth_, dReal flipWidth_, dReal vehicleWidth_, dReal xOffset, dReal yOffset, dReal zOffset) {
    TrackedVehicle *v = (TrackedVehicle *)malloc(sizeof(TrackedVehicle));

    v->density = 1.0;
    v->width = vehicleWidth_;
    const size_t numGrousers = 31; //30
    const dReal grouserHeight = 0.0028; //0.01
    dReal w = v->width + 2 * trackWidth_;
    v->leftTrack = track_init(wheelRadius_, wheelRadius_, wheelBase_, numGrousers, grouserHeight, trackWidth_, xOffset, yOffset - 0.5 * w, zOffset);
    v->rightTrack = track_init(wheelRadius_, wheelRadius_, wheelBase_, numGrousers, grouserHeight, trackWidth_, xOffset, yOffset + 0.5 * w, zOffset);

    v->leftFlip = flip_init(wheelRadius_, flipWheelRadius, flipWheelBase, numGrousers, grouserHeight, flipWidth_, xOffset+v->leftTrack->m->distance, (yOffset - 0.625*w)-v->leftTrack->m->trackDepth/2, zOffset);
    v->rightFlip = flip_init(wheelRadius_, flipWheelRadius, flipWheelBase, numGrousers, grouserHeight, flipWidth_, xOffset+v->rightTrack->m->distance, (yOffset + 0.625*w) + v->rightTrack->m->trackDepth/2, zOffset);

    v->leftBackFlip = back_flip_init(flipWheelRadius,wheelRadius_,flipWheelBase,numGrousers,grouserHeight,flipWidth_,xOffset-v->leftTrack->m->distance,(yOffset - 0.625*w)-v->leftTrack->m->trackDepth/2, zOffset);
    v->rightBackFlip = back_flip_init(flipWheelRadius,wheelRadius_, flipWheelBase, numGrousers, grouserHeight, flipWidth_,xOffset-v->rightTrack->m->distance, (yOffset + 0.625*w) + v->rightTrack->m->trackDepth/2, zOffset);

    v->xOffset = xOffset;
    v->yOffset = yOffset;
    v->zOffset = zOffset;

    return v;
}

void tracked_vehicle_create(TrackedVehicle *v, dWorldID world, dSpaceID space) {

    track_create(v->leftTrack, world, space);
    track_create(v->rightTrack, world, space);

    flip_create(v->leftFlip,v->leftTrack,world,space);
    flip_create(v->rightFlip,v->rightTrack,world,space);

    back_flip_create(v->leftBackFlip,v->leftTrack,world,space);
    back_flip_create(v->rightBackFlip,v->rightTrack,world,space);


    v->vehicleBody = dBodyCreate(world);
    v->vehicleGeom = dCreateBox(space, v->leftTrack->m->distance, v->width, v->leftTrack->m->radius1);
    dMassSetBox(&v->vehicleMass, v->density, v->leftTrack->m->distance, v->width, v->leftTrack->m->radius1);
    dGeomSetCategoryBits(v->vehicleGeom, 0x0);
    dGeomSetCollideBits(v->vehicleGeom, 0x0);
    dBodySetMass(v->vehicleBody, &v->vehicleMass);
    dBodySetPosition(v->vehicleBody, v->xOffset, v->yOffset, v->zOffset);
    dGeomSetBody(v->vehicleGeom, v->vehicleBody);

    v->leftTrackJoint = dJointCreateFixed(world, 0);
    v->rightTrackJoint = dJointCreateFixed(world, 0);

    // HINGE Flip Joint creation

    v->leftFlipJoint = dJointCreateHinge(world,0);
    v->rightFlipJoint = dJointCreateHinge(world,0);
    v->leftBackFlipJoint = dJointCreateHinge(world,0);
    v->rightBackFlipJoint = dJointCreateHinge(world,0);

    dJointAttach(v->leftTrackJoint, v->vehicleBody, v->leftTrack->trackBody);
    dJointAttach(v->rightTrackJoint, v->vehicleBody, v->rightTrack->trackBody);

    dJointAttach(v->leftFlipJoint,v->leftTrack->trackBody,v->leftFlip->flipBody);
    dJointAttach(v->leftBackFlipJoint,v->leftTrack->trackBody,v->leftBackFlip->flipBody);

    dJointAttach(v->rightFlipJoint,v->rightTrack->trackBody,v->rightFlip->flipBody);
    dJointAttach(v->rightBackFlipJoint,v->rightTrack->trackBody,v->rightBackFlip->flipBody);

    // HINGE Flip Anchor and Parameters

    dJointSetHingeAnchor(v->leftFlipJoint,v->leftFlip->xOffset,v->leftFlip->yOffset,v->leftFlip->zOffset);
    dJointSetHingeAxis(v->leftFlipJoint,0,1,0);
    dJointSetHingeParam(v->leftFlipJoint,dParamFMax,10);
    dJointSetHingeParam(v->leftFlipJoint, dParamLoStop, -0.25*M_PI);
    dJointSetHingeParam(v->leftFlipJoint, dParamHiStop, 0.50*M_PI);
    dJointSetHingeParam(v->leftFlipJoint,dParamBounce,0);


    dJointSetHingeAnchor(v->rightFlipJoint,v->rightFlip->xOffset,v->rightFlip->yOffset,v->rightFlip->zOffset);
    dJointSetHingeAxis(v->rightFlipJoint,0,1,0);
    dJointSetHingeParam(v->rightFlipJoint,dParamFMax,10);
    dJointSetHingeParam(v->rightFlipJoint,dParamBounce,0);
    dJointSetHingeParam(v->rightFlipJoint, dParamLoStop, -0.25*M_PI);
    dJointSetHingeParam(v->rightFlipJoint, dParamHiStop, 0.50*M_PI);


    dJointSetHingeAnchor(v->rightBackFlipJoint,v->rightBackFlip->xOffset+v->rightBackFlip->m->distance,v->rightBackFlip->yOffset,v->rightBackFlip->zOffset);
    dJointSetHingeAxis(v->rightBackFlipJoint,0,1,0);
    dJointSetHingeParam(v->rightBackFlipJoint,dParamFMax,10);
    dJointSetHingeParam(v->rightBackFlipJoint, dParamLoStop, -0.25*M_PI);
    dJointSetHingeParam(v->rightBackFlipJoint, dParamHiStop, 0.5*M_PI);


    dJointSetHingeAnchor(v->leftBackFlipJoint,v->leftBackFlip->xOffset+v->leftBackFlip->m->distance,v->leftBackFlip->yOffset,v->leftBackFlip->zOffset);
    dJointSetHingeAxis(v->leftBackFlipJoint,0,1,0);
    dJointSetHingeParam(v->leftBackFlipJoint,dParamFMax,10);
    dJointSetHingeParam(v->leftBackFlipJoint, dParamLoStop, -0.25*M_PI);
    dJointSetHingeParam(v->leftBackFlipJoint, dParamHiStop, 0.5*M_PI);

    // Fixed track joints
    dJointSetFixed(v->leftTrackJoint);
    dJointSetFixed(v->rightTrackJoint);
}

void tracked_vehicle_destroy(TrackedVehicle *v) {
    track_destroy(v->leftTrack);
    track_destroy(v->rightTrack);
}

void tracked_vehicle_deinit(TrackedVehicle *v) {
    track_deinit(v->leftTrack);
    track_deinit(v->rightTrack);
    free(v);
}

void setFlipAngle(dJointID flipjoint,dReal DesiredPosition){
	// it sets the flip to a desired angle
	dReal Gain = 0.75;
	dReal MaxForce = 14;

	dReal TruePosition = dJointGetHingeAngle(flipjoint);
	dReal Error = TruePosition - DesiredPosition;

	dReal DesiredVelocity = -Error * Gain;
	printf("Desired vel: %f\n",DesiredVelocity);

	dJointSetHingeParam(flipjoint, dParamFMax, MaxForce);
	dJointSetHingeParam(flipjoint, dParamVel, DesiredVelocity);

}


void tracked_vehicle_draw(TrackedVehicle *v) {
    {
        const dReal *pos = dGeomGetPosition(v->vehicleGeom);
        const dReal *R = dGeomGetRotation(v->vehicleGeom);
        dReal sides[3];
        dGeomBoxGetLengths(v->vehicleGeom, sides);
        dsDrawBoxD(pos, R, sides);

    }

    // draws the track if the second parameter is 1 it also draw the planes of the tracks/flippers
    track_draw(v->leftTrack,0);
    track_draw(v->rightTrack,0);

    flip_draw(v->leftFlip,0);
    flip_draw(v->rightFlip,0);

    back_flip_draw(v->leftBackFlip,0);
    back_flip_draw(v->rightBackFlip,0);


}

