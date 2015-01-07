//
//  Track.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Track.h"
#include "Environment.h"
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <drawstuff/drawstuff.h>

#define DRIVING_WHEEL_FRONT

Track::Track(const std::string& name_, dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserHeight_, dReal trackDepth_, dReal xOffset, dReal yOffset, dReal zOffset) : name(name_) {
    this->m = new TrackKinematicModel(radius1_, radius2_, distance_, numGrousers_, grouserHeight_, trackDepth_);
    this->density = 1.0;
    this->grouserBody = new dBodyID[numGrousers_];
    this->grouserGeom = new dGeomID[numGrousers_];
    this->grouserJoint = new dJointID[numGrousers_];
    this->grouserMass = new dMass[numGrousers_];
    this->xOffset = xOffset;
    this->yOffset = yOffset;
    this->zOffset = zOffset;
    this->numGrousers = numGrousers_;
}

Track::~Track() {
    delete [] this->grouserBody;
    delete [] this->grouserGeom;
    delete [] this->grouserJoint;
    delete [] this->grouserMass;
    delete this->m;
}

void Track::create(Environment *environment) {
    this->trackBody = dBodyCreate(environment->world);
    dMassSetBox(&this->trackMass, this->density, this->m->distance, this->m->radius[1], this->m->trackDepth);
    dBodySetMass(this->trackBody, &this->trackMass);
    dBodySetPosition(this->trackBody, this->xOffset, this->yOffset, this->zOffset);

    for(int w = 0; w < 2; w++) {
        this->wheelGeom[w] = dCreateCylinder(environment->space, this->m->radius[w], this->m->trackDepth);
        environment->setGeomName(this->wheelGeom[w], this->name + ".wheel" + boost::lexical_cast<std::string>(w));
        dGeomSetCategoryBits(this->wheelGeom[w], Category::WHEEL);
        dGeomSetCollideBits(this->wheelGeom[w], Category::GROUSER);
        dMassSetCylinder(&this->wheelMass[w], this->density, 3, this->m->radius[w], this->m->trackDepth);
        this->wheelBody[w] = dBodyCreate(environment->world);
        dBodySetMass(this->wheelBody[w], &this->wheelMass[w]);
        dGeomSetBody(this->wheelGeom[w], this->wheelBody[w]);
        dBodySetPosition(this->wheelBody[w], this->xOffset + w * this->m->distance, this->yOffset, this->zOffset);
        dMatrix3 wheelR;
        dRFromZAxis(wheelR, 0, 1, 0);
        dBodySetRotation(this->wheelBody[w], wheelR);
        this->wheelJoint[w] = dJointCreateHinge(environment->world, 0);
        dJointAttach(this->wheelJoint[w], this->trackBody, this->wheelBody[w]);
        dJointSetHingeAnchor(this->wheelJoint[w], this->xOffset + w * this->m->distance, this->yOffset, this->zOffset);
        dJointSetHingeAxis(this->wheelJoint[w], 0, 1, 0);
        // this guide should avoid tracks slipping out of their designed place
        dReal gh = 2 * (0.2 + std::max(this->m->radius[0], this->m->radius[1]));
        dReal gw = gh + this->m->distance;
        this->guideGeom[w] = dCreateBox(environment->space, gw, 0.01, gh);
        dGeomSetCategoryBits(this->guideGeom[w], Category::G_GUIDE);
        dGeomSetCollideBits(this->guideGeom[w], Category::GROUSER);
        dGeomSetBody(this->guideGeom[w], this->trackBody);
        dGeomSetOffsetPosition(this->guideGeom[w], 0.5 * this->m->distance, (0.02 + this->m->trackDepth) * (w - 0.5), 0.0);
    }
    
#ifdef DRIVING_WHEEL_FRONT
    dJointSetHingeParam(this->wheelJoint[0], dParamFMax, 2.5);
#endif
#ifdef DRIVING_WHEEL_BACK
    dJointSetHingeParam(this->wheelJoint[1], dParamFMax, 2.5);
#endif

    // grouser shrink/grow factor
    const dReal f = 1.03;

    for(size_t i = 0; i < this->m->numGrousers; i++) {
        this->grouserGeom[i] = dCreateBox(environment->space, this->m->grouserHeight, this->m->trackDepth, f * this->m->grouserWidth);
        environment->setGeomName(this->grouserGeom[i], this->name + ".grouser" + boost::lexical_cast<std::string>(i));
        dGeomSetCategoryBits(this->grouserGeom[i], Category::GROUSER);
        dGeomSetCollideBits(this->grouserGeom[i], Category::TERRAIN | Category::WHEEL | Category::OBSTACLE);
        dMassSetBox(&this->grouserMass[i], 10 * this->density, this->m->grouserHeight, this->m->trackDepth, f * this->m->grouserWidth);
        this->grouserBody[i] = dBodyCreate(environment->world);
        dBodySetMass(this->grouserBody[i], &this->grouserMass[i]);
        dGeomSetBody(this->grouserGeom[i], this->grouserBody[i]);
        dVector3 pos; dMatrix3 R;
        this->m->computeGrouserTransform3D(i, pos, R);
        dBodySetPosition(this->grouserBody[i], this->xOffset + pos[0], this->yOffset + pos[1], this->zOffset + pos[2]);
        dBodySetRotation(this->grouserBody[i], R);
    }

    for(size_t i = 0; i < this->m->numGrousers; i++) {
        size_t j = (i + 1) % this->m->numGrousers;
        dReal px, pz, qx, qz, a, dx, dz;
        this->m->getPointOnPath(i / (dReal)this->m->numGrousers, &px, &pz, &a);
        dx = cos(a - M_PI_2);
        dz = sin(a - M_PI_2);
        qx = px - this->m->grouserWidth * f * 0.5 * dx;
        qz = pz - this->m->grouserWidth * f * 0.5 * dz;
        px = px + this->m->grouserWidth * f * 0.5 * dx;
        pz = pz + this->m->grouserWidth * f * 0.5 * dz;
        this->grouserJoint[i] = dJointCreateHinge(environment->world, 0);
        dJointAttach(this->grouserJoint[i], this->grouserBody[i], this->grouserBody[j]);
        dJointSetHingeAnchor(this->grouserJoint[i], this->xOffset + px, this->yOffset, this->zOffset + pz);
        dJointSetHingeAxis(this->grouserJoint[i], 0, 1, 0);
    }
}

void Track::destroy() {
    // TODO
}

void Track::draw() {
    for(int w = 0; w < 2; w++) {
        const dReal *pos = dGeomGetPosition(this->wheelGeom[w]);
        const dReal *R = dGeomGetRotation(this->wheelGeom[w]);
        dReal radius, length;
        dGeomCylinderGetParams(this->wheelGeom[w], &radius, &length);
        dsDrawCylinderD(pos, R, length, radius);
    }
    
#ifdef DEBUG_DRAW_GROUSER_GUIDES
    for(int w = 0; w < 2; w++) {
        const dReal *pos = dGeomGetPosition(this->guideGeom[w]);
        const dReal *R = dGeomGetRotation(this->guideGeom[w]);
        dReal sides[3];
        dGeomBoxGetLengths(this->guideGeom[w], sides);
        dsDrawBoxD(pos, R, sides);
    }
#endif

    for(size_t i = 0; i < this->m->numGrousers; i++) {
        const dReal *pos = dGeomGetPosition(this->grouserGeom[i]);
        const dReal *R = dGeomGetRotation(this->grouserGeom[i]);
        dReal sides[3];
        dGeomBoxGetLengths(this->grouserGeom[i], sides);
        dsDrawBoxD(pos, R, sides);
    }
}

void Track::setVelocity(dReal velocity) {
#ifdef DRIVING_WHEEL_FRONT
    dJointSetHingeParam(this->wheelJoint[0], dParamVel, velocity);
#endif
#ifdef DRIVING_WHEEL_BACK
    dJointSetHingeParam(this->wheelJoint[1], dParamVel, velocity);
#endif
}
