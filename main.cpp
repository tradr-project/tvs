//
//  main.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include <vector>
#include <iostream>
#include <assert.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

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
    dRFromAxisAndAngle(R, 0, 1, 0, -theta);
}

class Track {
public:
    TrackKinematicModel m;
    dReal density;
    dBodyID trackBody, wheel1Body, wheel2Body;
    dMass trackMass, wheel1Mass, wheel2Mass;
    dGeomID wheel1Geom, wheel2Geom;
    dJointID wheel1Joint, wheel2Joint, guideJoint;
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
  density(1.0),
  grouserBody(numGrousers_), grouserGeom(numGrousers_),
  grouserJoint(numGrousers_), grouserMass(numGrousers_)
{
}

void Track::create(dWorldID world, dSpaceID space, dReal xOffset, dReal yOffset, dReal zOffset) {
    trackBody = dBodyCreate(world);
    dMassSetBox(&trackMass, density, m.distance, m.radius2, m.trackDepth);
    dBodySetMass(trackBody, &trackMass);
    
    wheel1Geom = dCreateCylinder(space, m.radius1, m.trackDepth);
    dGeomSetCategoryBits(wheel1Geom, 0x1);
    dGeomSetCollideBits(wheel1Geom, 0x2);
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
    
    wheel2Geom = dCreateCylinder(space, m.radius2, m.trackDepth);
    dGeomSetCategoryBits(wheel2Geom, 0x1);
    dGeomSetCollideBits(wheel2Geom, 0x2);
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

    dJointSetHingeParam(wheel2Joint, dParamFMax, 10);
    
    // grouser shrink/grow factor
    const dReal f = 0.8;
    
    for(size_t i = 0; i < m.numGrousers; i++) {
        grouserGeom[i] = dCreateBox(space, m.grouserHeight, m.trackDepth, f * m.grouserWidth);
        dGeomSetCategoryBits(grouserGeom[i], 0x2);
        dGeomSetCollideBits(grouserGeom[i], 0x1 | 0x4);
        dMassSetBox(&grouserMass[i], 10 * density, m.grouserHeight, m.trackDepth, f * m.grouserWidth);
        grouserBody[i] = dBodyCreate(world);
        dBodySetMass(grouserBody[i], &grouserMass[i]);
        dGeomSetBody(grouserGeom[i], grouserBody[i]);
        dVector3 pos; dMatrix3 R;
        m.computeGrouserTransform3D(i, pos, R);
        dBodySetPosition(grouserBody[i], xOffset + pos[0], yOffset + pos[1], zOffset + pos[2]);
        dBodySetRotation(grouserBody[i], R);

        // Disregard for now.
        // if (i == 0) {
        //     guideJoint = dJointCreateDHinge(world, 0);
        //     dJointAttach(guideJoint, wheel1Body, grouserBody[i]);
        //     dJointSetDHingeAxis (guideJoint, 0, 1, 0);
        //     dJointSetDHingeAnchor1 (guideJoint, xOffset, yOffset, zOffset);
        //     dJointSetDHingeAnchor2 (guideJoint, xOffset + pos[0], yOffset + pos[1], zOffset + pos[2]);
        // }
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
    dGeomSetCategoryBits(vehicleGeom, 0x0);
    dGeomSetCollideBits(vehicleGeom, 0x0);
    dBodySetMass(vehicleBody, &vehicleMass);
    dBodySetPosition(vehicleBody, xOffset, yOffset, zOffset);
    dGeomSetBody(vehicleGeom, vehicleBody);
    //dGeomSetPosition(vehicleGeom, leftTrack.m.distance, width, leftTrack.m.radius1);
    dReal w = width + 2 * leftTrack.m.trackDepth;
    leftTrack.create(world, space, xOffset, yOffset + 0.5 * w, zOffset);
    rightTrack.create(world, space, xOffset, yOffset - 0.5 * w, zOffset);
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

void nearCallback(void *data, dGeomID o1, dGeomID o2) {
    int i;
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    dContact contact[MAX_CONTACTS];
    for(i = 0; i < MAX_CONTACTS; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactMu2 | dContactFDir1;
        const dReal *v;
        dVector3 f;

        assert(dGeomGetClass(o1) == dBoxClass ||
               dGeomGetClass(o2) == dBoxClass);

        if (dGeomGetClass(o1) == dBoxClass)
            v = dBodyGetLinearVel(b1);
        else
            v = dBodyGetLinearVel(b2);

        dCalcVectorCross3(contact[i].fdir1, contact[i].geom.normal, v);
        dSafeNormalize3(contact[i].fdir1);

        if (dGeomGetClass(o1) == dPlaneClass ||
            dGeomGetClass(o2) == dPlaneClass) {
            contact[i].surface.mu = 2.0;
            contact[i].surface.mu2 = 0.5;
        } else if (dGeomGetClass(o1) == dCylinderClass ||
                   dGeomGetClass(o2) == dCylinderClass) {
            contact[i].surface.mu = contact[i].surface.mu2 = dInfinity;
        } else {
            printf ("%d, %d\n", dGeomGetClass(o1), dGeomGetClass(o2));
            assert(0);
        }

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
        for(int i = 0; i < 10; i++) {
            // find collisions and add contact joints
            dSpaceCollide(space, 0, &nearCallback);
            // step the simulation
            dWorldQuickStep(world, 0.001);
            // remove all contact joints
            dJointGroupEmpty(contactGroup);
        }
    }
    v.draw();
}

void stop() {
}

void command(int cmd) {
    const dReal V = 5;
    switch(cmd) {
        case 'a':
            dJointSetHingeParam(v.rightTrack.wheel2Joint, dParamVel, V);
            dJointSetHingeParam(v.leftTrack.wheel2Joint, dParamVel, -V);
            break;
        case 'd':
            dJointSetHingeParam(v.rightTrack.wheel2Joint, dParamVel, -V);
            dJointSetHingeParam(v.leftTrack.wheel2Joint, dParamVel, V);
            break;
        case 'w':
            dJointSetHingeParam(v.rightTrack.wheel2Joint, dParamVel, V);
            dJointSetHingeParam(v.leftTrack.wheel2Joint, dParamVel, V);
            break;
        case 's':
            dJointSetHingeParam(v.rightTrack.wheel2Joint, dParamVel, -V);
            dJointSetHingeParam(v.leftTrack.wheel2Joint, dParamVel, -V);
            break;
        case ' ':
            dJointSetHingeParam(v.rightTrack.wheel2Joint, dParamVel, 0);
            dJointSetHingeParam(v.leftTrack.wheel2Joint, dParamVel, 0);
            break;
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
    //dWorldSetERP(world, 0.2);
    //dWorldSetCFM(world, 1e-5);
    //dWorldSetContactMaxCorrectingVel(world, 0.9);
    //dWorldSetContactSurfaceLayer(world, 0.001);
    dWorldSetAutoDisableFlag(world, 1);
    
    planeGeom = dCreatePlane(space, 0, 0, 1, 0); // (a, b, c)' (x, y, z) = d
    dGeomSetCategoryBits(planeGeom, 0x4);
    dGeomSetCollideBits(planeGeom, 0x2);
    
    v.create(world, space, 0, 0, 0.301);

    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &step;
    fn.stop = &stop;
    fn.command = &command;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    dsSimulationLoop(argc, argv, 800, 600, &fn);
    
    dJointGroupDestroy(contactGroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    
    return 0;
}

