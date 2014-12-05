//
//  main.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include <vector>
#include <iostream>
#include <map>
#include <ode/ode.h>
#include "drawstuff.h"

std::map<dGeomID,int> geomClass;
const int CL_GROUSER = 1;
const int CL_WHEEL = 2;
const int CL_TERRAIN = 3;

class TrackKinematicModel {
public:
    dReal radius1, radius2, distance;
    size_t numGrousers;
    dReal grouserWidth, grouserHeight, trackDepth;
    dReal radiusDiff, pDistance, theta;
    dReal p1x, p1y, p2x, p2y;
    dReal arc1Length, arc2Length, totalLength;
    static const size_t sections = 4;
    dReal dlimits[sections];
    
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
  radiusDiff(radius1 - radius2), pDistance(sqrt(pow(distance, 2) - pow(radiusDiff, 2))),
  theta(atan2(pDistance, radiusDiff)),
  p1x(radius1 * cos(theta)), p1y(radius1 * sin(theta)),
  p2x(distance + radius2 * cos(theta)), p2y(radius2 * sin(theta)),
  arc1Length(radius1 * 2 * (M_PI - theta)), arc2Length(radius2 * 2 * theta), totalLength(0.0)
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

class Track {
public:
    TrackKinematicModel m;
    dReal density;
    dBodyID trackBody, wheel1Body, wheel2Body;
    dMass trackMass, wheel1Mass, wheel2Mass;
    dGeomID wheel1Geom, wheel2Geom;
    dJointID wheel1Joint, wheel2Joint;
    std::vector<dBodyID> grouserBody;
    std::vector<dGeomID> grouserGeom;
    std::vector<dJointID> grouserJoint;
    std::vector<dMass> grouserMass;
    
    Track(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserHeight_, dReal trackDepth_);
    void create(dWorldID world, dSpaceID space, dReal xOffset, dReal yOffset, dReal zOffset);
    void draw();
};

Track::Track(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserHeight_, dReal trackDepth_)
: m(radius1_, radius2_, distance_, numGrousers_, grouserHeight_, trackDepth_),
  density(10.0),
  grouserBody(numGrousers_), grouserGeom(numGrousers_),
  grouserJoint(numGrousers_), grouserMass(numGrousers_)
{
}

void Track::create(dWorldID world, dSpaceID space, dReal xOffset, dReal yOffset, dReal zOffset) {
    trackBody = dBodyCreate(world);
    dMassSetBox(&trackMass, density, m.distance, m.radius2, m.trackDepth);
    dBodySetMass(trackBody, &trackMass);
    
    wheel1Geom = dCreateCylinder(space, m.radius1, m.trackDepth);
    dMassSetCylinder(&wheel1Mass, density, 3, m.radius1, m.trackDepth);
    wheel1Body = dBodyCreate(world);
    dBodySetMass(wheel1Body, &wheel1Mass);
    dGeomSetBody(wheel1Geom, wheel1Body);
    dBodySetPosition(wheel1Body, xOffset, yOffset, zOffset);
    dMatrix3 wheel1R;
    dRFromZAxis(wheel1R, 0, 1, 0);
    dBodySetRotation(wheel1Body, wheel1R);
    wheel1Joint = dJointCreateHinge(world, 0);
    dJointAttach(wheel1Joint, trackBody, wheel1Body);
    dJointSetHingeAnchor(wheel1Joint, xOffset, yOffset, zOffset);
    dJointSetHingeAxis(wheel1Joint, 0, 1, 0);
#ifdef SET_FINITE_ROTATION
    dBodySetFiniteRotationMode(wheel1Body, 1);
    dBodySetFiniteRotationAxis(wheel1Body, 0, 1, 0);
#endif
    
    wheel2Geom = dCreateCylinder(space, m.radius2, m.trackDepth);
    dMassSetCylinder(&wheel2Mass, density, 3, m.radius2, m.trackDepth);
    wheel2Body = dBodyCreate(world);
    dBodySetMass(wheel2Body, &wheel2Mass);
    dGeomSetBody(wheel2Geom, wheel2Body);
    dBodySetPosition(wheel2Body, xOffset + m.distance, yOffset, zOffset);
    dMatrix3 wheel2R;
    dRFromZAxis(wheel2R, 0, 1, 0);
    dBodySetRotation(wheel2Body, wheel2R);
    wheel2Joint = dJointCreateHinge(world, 0);
    dJointAttach(wheel2Joint, trackBody, wheel2Body);
    dJointSetHingeAnchor(wheel2Joint, xOffset + m.distance, yOffset, zOffset);
    dJointSetHingeAxis(wheel2Joint, 0, 1, 0);
#ifdef SET_FINITE_ROTATION
    dBodySetFiniteRotationMode(wheel2Body, 1);
    dBodySetFiniteRotationAxis(wheel2Body, 0, 1, 0);
#endif
    
    geomClass[wheel1Geom] = CL_WHEEL;
    geomClass[wheel2Geom] = CL_WHEEL;
    
    // grouser shrink/grow factor
    const dReal f = 1.0;
    
    for(size_t i = 0; i < m.numGrousers; i++) {
        grouserGeom[i] = dCreateBox(space, m.grouserHeight, m.trackDepth, f * m.grouserWidth);
        dMassSetBox(&grouserMass[i], density, m.grouserHeight, m.trackDepth, f * m.grouserWidth);
        grouserBody[i] = dBodyCreate(world);
        dBodySetMass(grouserBody[i], &grouserMass[i]);
        dGeomSetBody(grouserGeom[i], grouserBody[i]);
        dVector3 pos; dMatrix3 R;
        m.computeGrouserTransform3D(i, pos, R);
        dBodySetPosition(grouserBody[i], xOffset + pos[0], yOffset + pos[1], zOffset + pos[2]);
        dBodySetRotation(grouserBody[i], R);
        
        geomClass[grouserGeom[i]] = CL_GROUSER;
    }
    
    for(size_t i = 0; i < m.numGrousers; i++) {
        size_t j = (i + 1) % m.numGrousers;
        dReal px, pz, qx, qz, a, dx, dz;
        m.getPointOnPath(i / (dReal)m.numGrousers, &px, &pz, &a);
        dx = cos(a - M_PI_2); dz = sin(a - M_PI_2);
        qx = px - m.grouserWidth * f * 0.5 * dx; qz = pz - m.grouserWidth * f * 0.5 * dz;
        px = px + m.grouserWidth * f * 0.5 * dx; pz = pz + m.grouserWidth * f * 0.5 * dz;
        grouserJoint[i] = dJointCreateHinge(world, 0);
        dJointAttach(grouserJoint[i], grouserBody[i], grouserBody[j]);
        dJointSetHingeAnchor(grouserJoint[i], xOffset + px, yOffset, zOffset + pz);
        dJointSetHingeAxis(grouserJoint[i], 0, 1, 0);
    }
}

void Track::draw() {
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
    
    for(int i = 0; i < m.numGrousers; i++) {
        const dReal *pos = dGeomGetPosition(grouserGeom[i]);
        const dReal *R = dGeomGetRotation(grouserGeom[i]);
        dReal sides[3];
        dGeomBoxGetLengths(grouserGeom[i], sides);
        dsDrawBoxD(pos, R, sides);
    }
}

class TrackedVehicle {
public:
    Track leftTrack, rightTrack;
    dReal density;
    dBodyID vehicleBody;
    dMass vehicleMass;
    dGeomID vehicleGeom;
    dJointID leftTrackJoint, rightTrackJoint;
    dReal width;
    
    TrackedVehicle(dReal wheelRadius_, dReal wheelBase_, dReal trackWidth_, dReal vehicleWidth_);
    void create(dWorldID world, dSpaceID space, dReal xOffset, dReal yOffset, dReal zOffset);
    void draw();
};

TrackedVehicle::TrackedVehicle(dReal wheelRadius_, dReal wheelBase_, dReal trackWidth_, dReal vehicleWidth_)
: leftTrack(wheelRadius_, wheelRadius_, wheelBase_, 30, 0.01, trackWidth_),
  rightTrack(wheelRadius_, wheelRadius_, wheelBase_, 30, 0.01, trackWidth_),
  width(vehicleWidth_), density(1.0)
{
}

void TrackedVehicle::create(dWorldID world, dSpaceID space, dReal xOffset, dReal yOffset, dReal zOffset) {
    vehicleBody = dBodyCreate(world);
    vehicleGeom = dCreateBox(space, leftTrack.m.distance, width, leftTrack.m.radius1);
    dMassSetBox(&vehicleMass, density, leftTrack.m.distance, width, leftTrack.m.radius1);
    dBodySetMass(vehicleBody, &vehicleMass);
    dBodySetPosition(vehicleBody, xOffset, yOffset, zOffset);
    dGeomSetBody(vehicleGeom, vehicleBody);
    //dGeomSetPosition(vehicleGeom, leftTrack.m.distance, width, leftTrack.m.radius1);
    dReal w = width + 2 * leftTrack.m.trackDepth,
        wb = leftTrack.m.distance;
    leftTrack.create(world, space, xOffset - wb * 0.5, yOffset + 0.5 * w, zOffset);
    rightTrack.create(world, space, xOffset - wb * 0.5, yOffset - 0.5 * w, zOffset);
    leftTrackJoint = dJointCreateFixed(world, 0);
    rightTrackJoint = dJointCreateFixed(world, 0);
    dJointAttach(leftTrackJoint, vehicleBody, leftTrack.trackBody);
    dJointAttach(rightTrackJoint, vehicleBody, rightTrack.trackBody);
    dJointSetFixed(leftTrackJoint);
    dJointSetFixed(rightTrackJoint);
}

void TrackedVehicle::draw() {
    {
        const dReal *pos = dGeomGetPosition(vehicleGeom);
        const dReal *R = dGeomGetRotation(vehicleGeom);
        dReal sides[3];
        dGeomBoxGetLengths(vehicleGeom, sides);
        dsDrawBoxD(pos, R, sides);
    }
    
    leftTrack.draw();
    rightTrack.draw();
}

#define MAX_CONTACTS 10

dWorldID world;
dSpaceID space;
dJointGroupID contactGroup;

dGeomID planeGeom;

TrackedVehicle v(0.3, 0.8, 0.2, 0.5);

dReal leftTorque = 0.0, rightTorque = 0.0;



void nearCallback(void *data, dGeomID o1, dGeomID o2) {
    int i;
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    dContact contact[MAX_CONTACTS];
    dReal mu = 1.0, mu2 =  0.0;
    if((geomClass[o1] == CL_WHEEL && geomClass[o2] == CL_GROUSER) ||
       (geomClass[o2] == CL_WHEEL && geomClass[o1] == CL_GROUSER)) {
        mu = dInfinity;
        mu2 = dInfinity;
    } else if((geomClass[o1] == CL_TERRAIN && geomClass[o2] == CL_GROUSER) ||
              (geomClass[o2] == CL_TERRAIN && geomClass[o1] == CL_GROUSER)) {
        mu = 5.0;
        mu2 = 0.0;
    }
    for(i = 0; i < MAX_CONTACTS; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = mu;
        contact[i].surface.mu2 = mu2;
        contact[i].surface.bounce = 0.5;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.0001;
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
        dJointAddHingeTorque(v.leftTrack.wheel1Joint, leftTorque);
        dJointAddHingeTorque(v.leftTrack.wheel2Joint, leftTorque);
        dJointAddHingeTorque(v.rightTrack.wheel1Joint, rightTorque);
        dJointAddHingeTorque(v.rightTrack.wheel2Joint, rightTorque);
        
        // find collisions and add contact joints
        dSpaceCollide(space, 0, &nearCallback);
        // step the simulation
        dWorldQuickStep(world, 0.01);
        // remove all contact joints
        dJointGroupEmpty(contactGroup);
    }
    v.draw();
}

void stop() {
}

void command(int cmd) {
    const dReal t = 0.6;
    switch(cmd) {
        case 'q': leftTorque = t; break;
        case 'a': leftTorque = 0.0; break;
        case 'z': leftTorque = -t; break;
        case 'w': rightTorque = t; break;
        case 's': rightTorque = 0.0; break;
        case 'x': rightTorque = -t; break;
        default: std::cout << "cmd=" << cmd << std::endl; break;
    }
}

int main(int argc, char **argv) {
    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);
    
    world = dWorldCreate();
    //space = dSimpleSpaceCreate(0);
    space = dHashSpaceCreate(0);
    contactGroup = dJointGroupCreate(0);
    dWorldSetGravity(world, 0, 0, -9.81);
    dWorldSetERP(world, 0.02);
    dWorldSetCFM(world, 1e-5);
    //dWorldSetContactMaxCorrectingVel(world, 0.9);
    //dWorldSetContactSurfaceLayer(world, 0.001);
    dWorldSetAutoDisableFlag(world, 1);
    
    planeGeom = dCreatePlane(space, 0, 0, 1, 0); // (a, b, c)' (x, y, z) = d
    
    geomClass[planeGeom] = CL_TERRAIN;
    
    v.create(world, space, 0, 0, 0.301);

    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &step;
    fn.stop = &stop;
    fn.command = &command;
    fn.path_to_textures = "textures";
    dsSimulationLoop(argc, argv, 800, 600, &fn);
    
    dJointGroupDestroy(contactGroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    
    return 0;
}
