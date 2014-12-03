//
//  main.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include <vector>
#include <iostream>
#include <ode/ode.h>
#include "drawstuff.h"

class TrackKinematicModel {
private:
    dReal radius1, radius2, distance;
    size_t numGrousers;
    dReal grouserWidth, grouserHeight, trackDepth;
    dReal radiusDiff, pDistance, theta;
    dReal p1x, p1y, p2x, p2y;
    dReal arc1Length, arc2Length, totalLength;
    dReal density;
    static const size_t sections = 4;
    dReal dlimits[sections];
    std::vector<dBodyID> grouserBody;
    std::vector<dGeomID> grouserGeom;
    std::vector<dJointID> grouserJoint;
    std::vector<dMass> grouserMass;
public:
    dBodyID trackBody, wheel1Body, wheel2Body;
    dMass trackMass, wheel1Mass, wheel2Mass;
    dGeomID wheel1Geom, wheel2Geom;
    dJointID wheel1Joint, wheel2Joint;
public:
    TrackKinematicModel(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserHeight_, dReal trackDepth_);
    inline dReal scale(dReal v, dReal vmin, dReal vmax) {return v * (vmax - vmin) + vmin;}
    void getPointOnPath(dReal u, dReal *x_, dReal *y_, dReal *theta_);
    void computeGrouserTransform3D(size_t i, dReal *pos, dReal *R);
    void create(dWorldID world, dSpaceID space);
    void draw();
};

TrackKinematicModel::TrackKinematicModel(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserHeight_, dReal trackDepth_)
: radius1(radius1_), radius2(radius2_), distance(distance_),
  numGrousers(numGrousers_), grouserHeight(grouserHeight_), trackDepth(trackDepth_),
  grouserBody(numGrousers_), grouserGeom(numGrousers_),
  grouserJoint(numGrousers_), grouserMass(numGrousers_),
  radiusDiff(radius1 - radius2), pDistance(sqrt(pow(distance, 2) - pow(radiusDiff, 2))),
  theta(atan2(pDistance, radiusDiff)),
  p1x(radius1 * cos(theta)), p1y(radius1 * sin(theta)),
  p2x(distance + radius2 * cos(theta)), p2y(radius2 * sin(theta)),
  arc1Length(radius1 * 2 * (M_PI - theta)), arc2Length(radius2 * 2 * theta), totalLength(0.0),
  density(100.0)
{
    dlimits[0] = arc1Length;
    dlimits[1] = pDistance;
    dlimits[2] = arc2Length;
    dlimits[3] = pDistance;
    for(size_t i = 0; i < sections; i++)
        totalLength += dlimits[i];
    grouserWidth = totalLength / (double)numGrousers;
}

void TrackKinematicModel::getPointOnPath(dReal u, dReal *x_, dReal *y_, dReal *theta_) {
    // compute which piece of the path u touches
    // and compute local parameter v
    size_t section = -1;
    dReal v = NAN;
    dReal u1 = (u - floor(u)) * totalLength;
    for(size_t i = 0; i < sections; i++) {
        if(u1 < dlimits[i]) {
            v = u1 / dlimits[i];
            section = i;
            break;
        }
        u1 -= dlimits[i];
    }
    // compute point and angle on path
    switch(section) {
        case 0:
            *theta_ = scale(v, 2 * M_PI - theta, theta);
            *x_ = radius1 * cos(*theta_);
            *y_ = radius1 * sin(*theta_);
            break;
        case 1:
            *theta_ = theta;
            *x_ = scale(v, p1x, p2x);
            *y_ = scale(v, p1y, p2y);
            break;
        case 2:
            *theta_ = scale(v, theta, -theta);
            *x_ = distance + radius2 * cos(*theta_);
            *y_ = radius2 * sin(*theta_);
            break;
        case 3:
            *theta_ = -theta;
            *x_ = scale(v, p2x, p1x);
            *y_ = scale(v, -p2y, -p1y);
            break;
    }
}

void TrackKinematicModel::computeGrouserTransform3D(size_t i, dReal *pos, dReal *R) {
    // 2d track is on XZ plane, with roller axes in Y direction
    pos[1] = 0;
    dReal theta;
    getPointOnPath(i / (dReal)numGrousers, &pos[0], &pos[2], &theta);
    dRFromAxisAndAngle(R, 0, -1, 0, theta);
}

void TrackKinematicModel::create(dWorldID world, dSpaceID space) {
    trackBody = dBodyCreate(world);
    dMassSetBox(&trackMass, density, distance, radius2, trackDepth);
    dBodySetMass(trackBody, &trackMass);
    
    const dReal zOffset = fmax(radius1, radius2);
    
    wheel1Geom = dCreateCylinder(space, radius1, trackDepth);
    dMassSetCylinder(&wheel1Mass, density, 3, radius1, trackDepth);
    wheel1Body = dBodyCreate(world);
    dBodySetMass(wheel1Body, &wheel1Mass);
    dGeomSetBody(wheel1Geom, wheel1Body);
    dBodySetPosition(wheel1Body, 0, 0, zOffset);
    dMatrix3 wheel1R;
    dRFromZAxis(wheel1R, 0, 1, 0);
    dBodySetRotation(wheel1Body, wheel1R);
    wheel1Joint = dJointCreateHinge(world, 0);
    dJointAttach(wheel1Joint, trackBody, wheel1Body);
    dJointSetHingeAnchor(wheel1Joint, 0, 0, zOffset);
    dJointSetHingeAxis(wheel1Joint, 0, 1, 0);
    
    wheel2Geom = dCreateCylinder(space, radius2, trackDepth);
    dMassSetCylinder(&wheel2Mass, density, 3, radius2, trackDepth);
    wheel2Body = dBodyCreate(world);
    dBodySetMass(wheel2Body, &wheel2Mass);
    dGeomSetBody(wheel2Geom, wheel2Body);
    dBodySetPosition(wheel2Body, distance, 0, zOffset);
    dMatrix3 wheel2R;
    dRFromZAxis(wheel2R, 0, 1, 0);
    dBodySetRotation(wheel2Body, wheel2R);
    wheel2Joint = dJointCreateHinge(world, 0);
    dJointAttach(wheel2Joint, trackBody, wheel2Body);
    dJointSetHingeAnchor(wheel2Joint, distance, 0, zOffset);
    dJointSetHingeAxis(wheel2Joint, 0, 1, 0);
    
    // grouser shrink/grow factor
    const dReal f = 0.9;
    
    for(size_t i = 0; i < numGrousers; i++) {
        grouserGeom[i] = dCreateBox(space, grouserHeight, trackDepth, f * grouserWidth);
        dMassSetBox(&grouserMass[i], density, grouserHeight, trackDepth, f * grouserWidth);
        grouserBody[i] = dBodyCreate(world);
        dBodySetMass(grouserBody[i], &grouserMass[i]);
        dGeomSetBody(grouserGeom[i], grouserBody[i]);
        dVector3 pos; dMatrix3 R;
        computeGrouserTransform3D(i, pos, R);
        dBodySetPosition(grouserBody[i], pos[0], pos[1], zOffset + pos[2]);
        dBodySetRotation(grouserBody[i], R);
    }
    
    for(size_t i = 0; i < numGrousers; i++) {
        size_t j = (i + 1) % numGrousers;
        dReal px, pz, qx, qz, a, dx, dz;
        getPointOnPath(i / (dReal)numGrousers, &px, &pz, &a);
        dx = cos(a - M_PI_2); dz = sin(a - M_PI_2);
        qx = px - grouserWidth * f * 0.5 * dx; qz = pz - grouserWidth * f * 0.5 * dz;
        px = px + grouserWidth * f * 0.5 * dx; pz = pz + grouserWidth * f * 0.5 * dz;
        grouserJoint[i] = dJointCreateHinge(world, 0);
        dJointAttach(grouserJoint[i], grouserBody[i], grouserBody[j]);
        dJointSetHingeAnchor(grouserJoint[i], px, 0, zOffset + pz);
        dJointSetHingeAxis(grouserJoint[i], 0, 1, 0);
    }
}

void TrackKinematicModel::draw() {
    {
        const dReal *pos = dGeomGetPosition(wheel1Geom);
        const dReal *R = dGeomGetRotation(wheel1Geom);
        dReal radius, length;
        dGeomCylinderGetParams(wheel1Geom, &radius, &length);
        dsDrawCylinderD(pos, R, length, radius);
    }
    
    {
        const dReal *pos = dGeomGetPosition(wheel2Geom);
        const dReal *R = dGeomGetRotation(wheel2Geom);
        dReal radius, length;
        dGeomCylinderGetParams(wheel2Geom, &radius, &length);
        dsDrawCylinderD(pos, R, length, radius);
    }
    
    for(int i = 0; i < numGrousers; i++) {
        const dReal *pos = dGeomGetPosition(grouserGeom[i]);
        const dReal *R = dGeomGetRotation(grouserGeom[i]);
        dReal sides[3];
        dGeomBoxGetLengths(grouserGeom[i], sides);
        dsDrawBoxD(pos, R, sides);
    }
}

#define MAX_CONTACTS 10

dWorldID world;
dSpaceID space;
dJointGroupID contactGroup;

dGeomID planeGeom;

TrackKinematicModel track(0.25, 0.18, 0.6, 30, 0.01, 0.2);

void nearCallback(void *data, dGeomID o1, dGeomID o2) {
    int i;
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    dContact contact[MAX_CONTACTS];
    for(i = 0; i < MAX_CONTACTS; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = dInfinity;
        contact[i].surface.mu2 = 0.0;
        contact[i].surface.bounce = 0.5;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.001;
    }
    if(int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact))) {
        for(i = 0; i < numc; i++) {
            dJointID c = dJointCreateContact(world, contactGroup, contact + i);
            dJointAttach(c, b1, b2);
        }
    }
}

void start() {
    static float xyz[3] = {2.0f,-2.0f,1.7600f};
    static float hpr[3] = {140.000f,-17.0000f,0.0000f};
    dsSetViewpoint(xyz,hpr);
}

void step(int pause) {
    if(!pause) {
        //dBodyAddTorque(wheel1Body, 0.0, 20.0, 0.0);
        //dBodyAddTorque(wheel2Body, 0.0, 2.0, 0.0);
        //dBodyAddForce(trackBody, 2.5, 0, 0);
        const dReal t = 1.2;
        //dJointAddHingeTorque(wheel1Joint, t);
        dJointAddHingeTorque(track.wheel2Joint, t);
        
        // find collisions and add contact joints
        dSpaceCollide(space, 0, &nearCallback);
        // step the simulation
        dWorldQuickStep(world, 0.01);
        // remove all contact joints
        dJointGroupEmpty(contactGroup);
    }
    track.draw();
}

void stop() {
}

int main(int argc, char **argv) {
    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);
    
    world = dWorldCreate();
    //space = dSimpleSpaceCreate(0);
    space = dHashSpaceCreate(0);
    contactGroup = dJointGroupCreate(0);
    dWorldSetGravity(world, 0, 0, -9.81);
    //dWorldSetERP(world, 0.2);
    dWorldSetCFM(world, 1e-5);
    //dWorldSetContactMaxCorrectingVel(world, 0.9);
    //dWorldSetContactSurfaceLayer(world, 0.001);
    dWorldSetAutoDisableFlag(world, 1);
    
    planeGeom = dCreatePlane(space, 0, 0, 1, 0); // (a, b, c)' (x, y, z) = d
    
    track.create(world, space);

    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &step;
    fn.stop = &stop;
    fn.command = 0;
    fn.path_to_textures = "/Users/user/ode-0.12-drawstuff/drawstuff/textures";
    dsSimulationLoop(argc, argv, 800, 600, &fn);
    
    dJointGroupDestroy(contactGroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    
    return 0;
}
