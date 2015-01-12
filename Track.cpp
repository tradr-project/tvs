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

Track::Track(const std::string& name_, dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal linkThickness_, dReal grouserHeight_, dReal trackDepth_, dReal xOffset, dReal yOffset, dReal zOffset) : name(name_) {
    this->m = new TrackKinematicModel(radius1_, radius2_, distance_, numGrousers_, grouserHeight_, trackDepth_);
    this->density = 1.0;
    this->linkBody = new dBodyID[numGrousers_];
    this->linkGeom = new dGeomID[numGrousers_];
    this->grouserGeom = new dGeomID[numGrousers_];
    this->linkJoint = new dJointID[numGrousers_];
    this->linkMass = new dMass[numGrousers_];
    this->xOffset = xOffset;
    this->yOffset = yOffset;
    this->zOffset = zOffset;
    this->numGrousers = numGrousers_;
    this->linkThickness = linkThickness_;
    this->grouserHeight = grouserHeight_;
}

Track::~Track() {
    delete [] this->linkBody;
    delete [] this->linkGeom;
    delete [] this->grouserGeom;
    delete [] this->linkJoint;
    delete [] this->linkMass;
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
        environment->setGeomName(this->guideGeom[w], this->name + ".grouser_guide" + boost::lexical_cast<std::string>(w));
        dGeomSetCategoryBits(this->guideGeom[w], Category::G_GUIDE);
        dGeomSetCollideBits(this->guideGeom[w], Category::GROUSER);
        dGeomSetBody(this->guideGeom[w], this->trackBody);
        dGeomSetOffsetPosition(this->guideGeom[w], 0.5 * this->m->distance, (0.02 + this->m->trackDepth) * (w - 0.5), 0.0);
    }
    
    const dReal fMax = 5.0;
#ifdef DRIVING_WHEEL_FRONT
    dJointSetHingeParam(this->wheelJoint[0], dParamFMax, fMax);
#endif
#ifdef DRIVING_WHEEL_BACK
    dJointSetHingeParam(this->wheelJoint[1], dParamFMax, fMax);
#endif

    // grouser shrink/grow factor
    const dReal f = 1.03;

    for(size_t i = 0; i < this->m->numGrousers; i++) {
        this->linkBody[i] = dBodyCreate(environment->world);
        dMassSetBox(&this->linkMass[i], 10 * this->density, this->m->grouserHeight, this->m->trackDepth, f * this->m->grouserWidth);
        dBodySetMass(this->linkBody[i], &this->linkMass[i]);

        dVector3 pos; dMatrix3 R;
        this->m->computeGrouserTransform3D(i, pos, R);

        this->linkGeom[i] = dCreateBox(environment->space, this->linkThickness, this->m->trackDepth, f * this->m->grouserWidth);
        environment->setGeomName(this->linkGeom[i], this->name + ".grouser" + boost::lexical_cast<std::string>(i));
        dGeomSetCategoryBits(this->linkGeom[i], Category::GROUSER);
        dGeomSetCollideBits(this->linkGeom[i], Category::TERRAIN | Category::WHEEL | Category::OBSTACLE);
        dGeomSetBody(this->linkGeom[i], this->linkBody[i]);

        this->grouserGeom[i] = dCreateBox(environment->space, this->m->grouserHeight, this->m->trackDepth, this->linkThickness);
        environment->setGeomName(this->grouserGeom[i], this->name + ".grouserTooth" + boost::lexical_cast<std::string>(i));
        dGeomSetCategoryBits(this->grouserGeom[i], Category::GROUSER);
        dGeomSetCollideBits(this->grouserGeom[i], Category::TERRAIN | Category::WHEEL | Category::OBSTACLE);
        dGeomSetBody(this->grouserGeom[i], this->linkBody[i]);
        dGeomSetOffsetPosition(this->grouserGeom[i], 0.5 * (this->linkThickness + this->m->grouserHeight), 0, 0);

        dBodySetPosition(this->linkBody[i], this->xOffset + pos[0], this->yOffset + pos[1], this->zOffset + pos[2]);
        dBodySetRotation(this->linkBody[i], R);
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
        this->linkJoint[i] = dJointCreateHinge(environment->world, 0);
        dJointAttach(this->linkJoint[i], this->linkBody[i], this->linkBody[j]);
        dJointSetHingeAnchor(this->linkJoint[i], this->xOffset + px, this->yOffset, this->zOffset + pz);
        dJointSetHingeAxis(this->linkJoint[i], 0, 1, 0);
    }
}

void Track::destroy() {
    dBodyDestroy(this->trackBody);
    for(int w = 0; w < 2; w++) {
        dBodyDestroy(this->wheelBody[w]);
        dGeomDestroy(this->wheelGeom[w]);
        dGeomDestroy(this->guideGeom[w]);
    }
    for(size_t i = 0; i < this->m->numGrousers; i++) {
        dBodyDestroy(this->linkBody[i]);
        dGeomDestroy(this->linkGeom[i]);
        dGeomDestroy(this->grouserGeom[i]);
    }
}

void Track::draw() {
    dsSetColor(1, 1, 0);
    for(int w = 0; w < 2; w++) {
        const dReal *pos = dGeomGetPosition(this->wheelGeom[w]);
        const dReal *R = dGeomGetRotation(this->wheelGeom[w]);
        dReal radius, length;
        dGeomCylinderGetParams(this->wheelGeom[w], &radius, &length);
        dsDrawCylinderD(pos, R, length, radius);
    }

    dsSetColor(1, 0, 0);
    for(size_t i = 0; i < this->m->numGrousers; i++) {
        const dReal *pos = dGeomGetPosition(this->linkGeom[i]);
        const dReal *R = dGeomGetRotation(this->linkGeom[i]);
        dReal sides[3];
        dGeomBoxGetLengths(this->linkGeom[i], sides);
        dsDrawBoxD(pos, R, sides);
    }
    for(size_t i = 0; i < this->m->numGrousers; i++) {
        const dReal *pos = dGeomGetPosition(this->grouserGeom[i]);
        const dReal *R = dGeomGetRotation(this->grouserGeom[i]);
        dReal sides[3];
        dGeomBoxGetLengths(this->grouserGeom[i], sides);
        dsDrawBoxD(pos, R, sides);
    }
    
#ifdef DEBUG_DRAW_GROUSER_GUIDES
    dsSetColorAlpha(0, 1, 0, 0.3);
    for(int w = 0; w < 2; w++) {
        const dReal *pos = dGeomGetPosition(this->guideGeom[w]);
        const dReal *R = dGeomGetRotation(this->guideGeom[w]);
        dReal sides[3];
        dGeomBoxGetLengths(this->guideGeom[w], sides);
        dsDrawBoxD(pos, R, sides);
    }
#endif
}

void Track::setVelocity(dReal velocity) {
#ifdef DRIVING_WHEEL_FRONT
    dJointSetHingeParam(this->wheelJoint[0], dParamVel, velocity);
#endif
#ifdef DRIVING_WHEEL_BACK
    dJointSetHingeParam(this->wheelJoint[1], dParamVel, velocity);
#endif
}
