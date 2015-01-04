//
//  Environment.cpp
//  tvs
//
//  Created by Federico Ferri on 17/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Environment.h"
#include "ODEUtils.h"
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <iostream>
#include <drawstuff/drawstuff.h>

static const dVector3 center = {3,3,0};
static const dVector3 extents = {7,7,7};
static const dReal limit = 8.0;

Environment::Environment() {
    this->v = new TrackedVehicle(0.3, 0.8, 0.2, 0.5, 0, 0, 0.301+0.4);
#if defined(USE_PCL)
    this->pcl = new PointCloud("pcd_0000.ds.0.3.xyz");
    this->pcl->filterFar(center, limit);
    this->pcl->point_radius = 0.3 * sqrt(3) / 2.0;
#else
    this->pcl = 0L;
#endif
}

Environment::~Environment() {
    dJointGroupDestroy(this->contactGroup);
    dSpaceDestroy(this->space);
    dWorldDestroy(this->world);
}

void Environment::create() {
    this->world = dWorldCreate();
    this->space = dQuadTreeSpaceCreate(0, center, extents, 6);
    this->contactGroup = dJointGroupCreate(0);
    dWorldSetGravity(this->world, 0, 0, -9.81);
    //dWorldSetERP(this->world, 0.7);
    //dWorldSetCFM(this->world, 1e-5);
    //dWorldSetContactMaxCorrectingVel(this->world, 0.9);
    //dWorldSetContactSurfaceLayer(this->world, 0.001);
    dWorldSetAutoDisableFlag(this->world, 1);

    this->planeGeom = dCreatePlane(this->space, 0, 0, 1, 0); // (a, b, c)' (x, y, z) = d
    dGeomSetCategoryBits(this->planeGeom, Category::TERRAIN);
    dGeomSetCollideBits(this->planeGeom, Category::GROUSER | Category::OBSTACLE);

    v->create(this);
    if(this->pcl) pcl->create(this);
    
#if 0
    dVector3 sides = {1,1,1};
    dGeomID g = dCreateBox(this->space, sides[0], sides[1], sides[2]);
    dMass m;
    dMassSetBox(&m, 1, sides[0], sides[1], sides[2]);
    dBodyID b = dBodyCreate(this->world);
    dBodySetMass(b, &m);
    dGeomSetBody(g, b);
    dBodySetPosition(b, 1, 1, 1);
    dMatrix3 R;
    dRFromAxisAndAngle(R, 1, 2, 2, 0.3);
    dBodySetRotation(b, R);
#endif
}

void Environment::destroy() {
    this->v->destroy();
    if(this->pcl) this->pcl->destroy();
}

std::string Environment::getGeomName(dGeomID geom) const
{
    std::map<dGeomID, std::string>::const_iterator it = geomNames.find(geom);
    if (it == geomNames.end())
        return boost::lexical_cast<std::string>(reinterpret_cast<unsigned long>(geom));
    else
        return it->second;
}

void Environment::setGeomName(dGeomID geom, const std::string &name)
{
    geomNames[geom] = name;
}

int Environment::getMaxContacts(dGeomID o1, dGeomID o2) {
    return 10;
}

static void nearCallbackWrapper(void *data, dGeomID o1, dGeomID o2) {
    reinterpret_cast<Environment *>(data)->nearCallback(o1, o2);
}

bool Environment::isCatPair(unsigned long cat1, unsigned long cat2, dGeomID *o1, dGeomID *o2) {
    unsigned long catBits1 = dGeomGetCategoryBits(*o1);
    unsigned long catBits2 = dGeomGetCategoryBits(*o2);

    if((catBits1 & cat1) && (catBits2 & cat2)) {
        return true;
    }
    if((catBits1 & cat2) && (catBits2 & cat1)) {
        // swap o1 and o2
        dGeomID tmp = *o1;
        *o1 = *o2;
        *o2 = tmp;
        return true;
    }
    return false;
}

void Environment::nearCallback(dGeomID o1, dGeomID o2) {
    if(isCatPair(Category::WHEEL, Category::GROUSER, &o1, &o2))
        nearCallbackWheelGrouser(o1, o2);
    else if(isCatPair(Category::GROUSER, Category::TERRAIN, &o1, &o2))
        nearCallbackGrouserTerrain(o1, o2);
    else
        nearCallbackDefault(o1, o2);
}

void Environment::nearCallbackWheelGrouser(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    int maxc = getMaxContacts(o1, o2);
    dContact contact[maxc];
    int numc = dCollide(o1, o2, maxc, &contact[0].geom, sizeof(dContact));
    for(size_t i = 0; i < numc; i++) {
        const dReal *v = dBodyGetLinearVel(b2); // grouser vel
        dCalcVectorCross3(contact[i].fdir1, contact[i].geom.normal, v);
        dSafeNormalize3(contact[i].fdir1);
        contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactMu2 | dContactFDir1;
        contact[i].surface.bounce = 0.5;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.0001;
        contact[i].surface.mu = dInfinity;
        contact[i].surface.mu2 = dInfinity;
        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void Environment::nearCallbackGrouserTerrain(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    int maxc = getMaxContacts(o1, o2);
    dContact contact[maxc];
    int numc = dCollide(o1, o2, maxc, &contact[0].geom, sizeof(dContact));
    for(size_t i = 0; i < numc; i++) {
        const dReal *v = dBodyGetLinearVel(b1); // grouser vel
        dCalcVectorCross3(contact[i].fdir1, contact[i].geom.normal, v);
        dSafeNormalize3(contact[i].fdir1);
        contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactMu2 | dContactFDir1;
        contact[i].surface.bounce = 0.5;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.0001;
        contact[i].surface.mu = 5.0;
        contact[i].surface.mu2 = 1.3;
        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void Environment::nearCallbackDefault(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    int maxc = getMaxContacts(o1, o2);
    dContact contact[maxc];
    int numc = dCollide(o1, o2, maxc, &contact[0].geom, sizeof(dContact));
    for(size_t i = 0; i < numc; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.bounce = 0.5;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.0001;
        contact[i].surface.mu = 5.0;
        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
        this->badCollision = true;
    }
}

void Environment::step(dReal stepSize, int simulationStepsPerFrame) {
    badCollision = false;
    for(size_t i = 0; i < simulationStepsPerFrame; i++) {
        // find collisions and add contact joints
        dSpaceCollide(this->space, this, &nearCallbackWrapper);
        // step the simulation
        dWorldQuickStep(this->world, stepSize / (dReal)simulationStepsPerFrame);
        // remove all contact joints
        dJointGroupEmpty(this->contactGroup);
    }
}

void Environment::draw() {
    if(this->pcl) this->pcl->draw();
    this->v->draw();
}

