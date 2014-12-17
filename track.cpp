//
//  track.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "track.h"
#include "world.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <drawstuff/drawstuff.h>

Track::Track(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserHeight_, dReal trackDepth_, dReal xOffset, dReal yOffset, dReal zOffset) {
    this->m = new TrackKinematicModel(radius1_, radius2_, distance_, numGrousers_, grouserHeight_, trackDepth_);
    this->density = 1.0;
    this->grouserBody = new dBodyID[numGrousers_];
    this->grouserGeom = new dGeomID[numGrousers_];
    this->grouserJoint = new dJointID[numGrousers_];
    this->grouserMass = new dMass[numGrousers_];
    this->xOffset = xOffset;
    this->yOffset = yOffset;
    this->zOffset = zOffset;
}

Track::~Track() {
    delete [] this->grouserBody;
    delete [] this->grouserGeom;
    delete [] this->grouserJoint;
    delete [] this->grouserMass;
    delete this->m;
}

void Track::create(World *world) {
    this->trackBody = dBodyCreate(world->world);
    dMassSetBox(&this->trackMass, this->density, this->m->distance, this->m->radius2, this->m->trackDepth);
    dBodySetMass(this->trackBody, &this->trackMass);

    this->wheel1Geom = dCreateCylinder(world->space, this->m->radius1, this->m->trackDepth);
    dGeomSetCategoryBits(this->wheel1Geom, 0x1);
    dGeomSetCollideBits(this->wheel1Geom, 0x2);
    dMassSetCylinder(&this->wheel1Mass, this->density, 3, this->m->radius1, this->m->trackDepth);
    this->wheel1Body = dBodyCreate(world->world);
    dBodySetMass(this->wheel1Body, &this->wheel1Mass);
    dGeomSetBody(this->wheel1Geom, this->wheel1Body);
    dBodySetPosition(this->wheel1Body, this->xOffset, this->yOffset, this->zOffset);
    dMatrix3 wheel1R;
    dRFromZAxis(wheel1R, 0, 1, 0);
    dBodySetRotation(this->wheel1Body, wheel1R);
    this->wheel1Joint = dJointCreateHinge(world->world, 0);
    dJointAttach(this->wheel1Joint, this->trackBody, this->wheel1Body);
    dJointSetHingeAnchor(this->wheel1Joint, this->xOffset, this->yOffset, this->zOffset);
    dJointSetHingeAxis(this->wheel1Joint, 0, 1, 0);

    this->wheel2Geom = dCreateCylinder(world->space, this->m->radius2, this->m->trackDepth);
    dGeomSetCategoryBits(this->wheel2Geom, 0x1);
    dGeomSetCollideBits(this->wheel2Geom, 0x2);
    dMassSetCylinder(&this->wheel2Mass, this->density, 3, this->m->radius2, this->m->trackDepth);
    this->wheel2Body = dBodyCreate(world->world);
    dBodySetMass(this->wheel2Body, &this->wheel2Mass);
    dGeomSetBody(this->wheel2Geom, this->wheel2Body);
    dBodySetPosition(this->wheel2Body, this->xOffset + this->m->distance, this->yOffset, this->zOffset);
    dMatrix3 wheel2R;
    dRFromZAxis(wheel2R, 0, 1, 0);
    dBodySetRotation(this->wheel2Body, wheel2R);
    this->wheel2Joint = dJointCreateHinge(world->world, 0);
    dJointAttach(this->wheel2Joint, this->trackBody, this->wheel2Body);
    dJointSetHingeAnchor(this->wheel2Joint, this->xOffset + this->m->distance, this->yOffset, this->zOffset);
    dJointSetHingeAxis(this->wheel2Joint, 0, 1, 0);

    dJointSetHingeParam(this->wheel2Joint, dParamFMax, 10);

    // grouser shrink/grow factor
    const dReal f = 1.03;
    size_t i;

    for(i = 0; i < this->m->numGrousers; i++) {
        this->grouserGeom[i] = dCreateBox(world->space, this->m->grouserHeight, this->m->trackDepth, f * this->m->grouserWidth);
        dGeomSetCategoryBits(this->grouserGeom[i], 0x2);
        dGeomSetCollideBits(this->grouserGeom[i], 0x1 | 0x4);
        dMassSetBox(&this->grouserMass[i], 10 * this->density, this->m->grouserHeight, this->m->trackDepth, f * this->m->grouserWidth);
        this->grouserBody[i] = dBodyCreate(world->world);
        dBodySetMass(this->grouserBody[i], &this->grouserMass[i]);
        dGeomSetBody(this->grouserGeom[i], this->grouserBody[i]);
        dVector3 pos; dMatrix3 R;
        this->m->computeGrouserTransform3D(i, pos, R);
        dBodySetPosition(this->grouserBody[i], this->xOffset + pos[0], this->yOffset + pos[1], this->zOffset + pos[2]);
        dBodySetRotation(this->grouserBody[i], R);

        // Disregard for now.
        // if(i == 0) {
        //     t->guideJoint = dJointCreateDHinge(world, 0);
        //     dJointAttach(t->guideJoint, t->wheel1Body, t->grouserBody[i]);
        //     dJointSetDHingeAxis(t->guideJoint, 0, 1, 0);
        //     dJointSetDHingeAnchor1(t->guideJoint, xOffset, yOffset, zOffset);
        //     dJointSetDHingeAnchor2(t->guideJoint, xOffset + pos[0], yOffset + pos[1], zOffset + pos[2]);
        // }
    }

    for(i = 0; i < this->m->numGrousers; i++) {
        size_t j = (i + 1) % this->m->numGrousers;
        dReal px, pz, qx, qz, a, dx, dz;
        this->m->getPointOnPath(i / (dReal)this->m->numGrousers, &px, &pz, &a);
        dx = cos(a - M_PI_2);
        dz = sin(a - M_PI_2);
        qx = px - this->m->grouserWidth * f * 0.5 * dx;
        qz = pz - this->m->grouserWidth * f * 0.5 * dz;
        px = px + this->m->grouserWidth * f * 0.5 * dx;
        pz = pz + this->m->grouserWidth * f * 0.5 * dz;
        this->grouserJoint[i] = dJointCreateHinge(world->world, 0);
        dJointAttach(this->grouserJoint[i], this->grouserBody[i], this->grouserBody[j]);
        dJointSetHingeAnchor(this->grouserJoint[i], this->xOffset + px, this->yOffset, this->zOffset + pz);
        dJointSetHingeAxis(this->grouserJoint[i], 0, 1, 0);
    }
}

void Track::destroy() {
    // TODO
}

void Track::draw() {
    {
        const dReal *pos = dGeomGetPosition(this->wheel1Geom);
        const dReal *R = dGeomGetRotation(this->wheel1Geom);
        dReal radius, length;
        dGeomCylinderGetParams(this->wheel1Geom, &radius, &length);
        dsDrawCylinderD(pos, R, length, radius);
    }

    {
        const dReal *pos = dGeomGetPosition(this->wheel2Geom);
        const dReal *R = dGeomGetRotation(this->wheel2Geom);
        dReal radius, length;
        dGeomCylinderGetParams(this->wheel2Geom, &radius, &length);
        dsDrawCylinderD(pos, R, length, radius);
    }

    size_t i;
    for(i = 0; i < this->m->numGrousers; i++) {
        const dReal *pos = dGeomGetPosition(this->grouserGeom[i]);
        const dReal *R = dGeomGetRotation(this->grouserGeom[i]);
        dReal sides[3];
        dGeomBoxGetLengths(this->grouserGeom[i], sides);
        dsDrawBoxD(pos, R, sides);
    }
}

