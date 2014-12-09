//
//  main.c
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include <assert.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

typedef struct {
    dReal radius1;
    dReal radius2;
    dReal distance;
    size_t numGrousers;
    dReal grouserWidth;
    dReal grouserHeight;
    dReal trackDepth;
    dReal radiusDiff;
    dReal pDistance;
    dReal theta;
    dReal p1x;
    dReal p1y;
    dReal p2x;
    dReal p2y;
    dReal arc1Length;
    dReal arc2Length;
    dReal totalLength;
    dReal dlimits[4];
} TrackKinematicModel;
    
TrackKinematicModel* track_kinematic_model_create(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserHeight_, dReal trackDepth_) {
    TrackKinematicModel *m = (TrackKinematicModel *)malloc(sizeof(TrackKinematicModel));
    m->radius1 = radius1_;
    m->radius2 = radius2_;
    m->distance = distance_;
    m->numGrousers = numGrousers_;
    m->grouserHeight = grouserHeight_;
    m->trackDepth = trackDepth_;
    m->radiusDiff = m->radius1 - m->radius2;
    m->pDistance = sqrt(pow(m->distance, 2) - pow(m->radiusDiff, 2));
    m->theta = atan2(m->pDistance, m->radiusDiff);
    m->p1x = m->radius1 * cos(m->theta);
    m->p1y = m->radius1 * sin(m->theta);
    m->p2x = m->distance + m->radius2 * cos(m->theta);
    m->p2y = m->radius2 * sin(m->theta);
    m->arc1Length = m->radius1 * 2 * (M_PI - m->theta);
    m->arc2Length = m->radius2 * 2 * m->theta;
    m->totalLength = 0.0;
    m->dlimits[0] = m->arc1Length;
    m->dlimits[1] = m->pDistance;
    m->dlimits[2] = m->arc2Length;
    m->dlimits[3] = m->pDistance;
    for(size_t i = 0; i < 4; i++)
        m->totalLength += m->dlimits[i];
    m->grouserWidth = m->totalLength / (double)m->numGrousers;
    return m;
}

void track_kinematic_model_destroy(TrackKinematicModel *m) {
    free(m);
}

void track_kinematic_model_get_point_on_path(TrackKinematicModel *m, dReal u, dReal *x_, dReal *y_, dReal *theta_) {
    // compute which piece of the path u touches
    // and compute local parameter v
    size_t section = -1;
    dReal v = NAN;
    dReal u1 = (u - floor(u)) * m->totalLength;
    for(size_t i = 0; i < 4; i++) {
        if(u1 < m->dlimits[i]) {
            v = u1 / m->dlimits[i];
            section = i;
            break;
        }
        u1 -= m->dlimits[i];
    }
    // compute point and angle on path
    switch(section) {
        case 0:
            *theta_ = (1 - v) * (2 * M_PI - m->theta) + v * m->theta;
            *x_ = m->radius1 * cos(*theta_);
            *y_ = m->radius1 * sin(*theta_);
            break;
        case 1:
            *theta_ = m->theta;
            *x_ = (1 - v) * m->p1x + v * m->p2x;
            *y_ = (1 - v) * m->p1y + v * m->p2y;
            break;
        case 2:
            *theta_ = (1 - v) * m->theta + v * -m->theta;
            *x_ = m->distance + m->radius2 * cos(*theta_);
            *y_ = m->radius2 * sin(*theta_);
            break;
        case 3:
            *theta_ = -m->theta;
            *x_ = (1 - v) * m->p2x + v * m->p1x;
            *y_ = (1 - v) * -m->p2y + v * -m->p1y;
            break;
    }
}

void track_kinematic_model_compute_grouser_transform_3D(TrackKinematicModel *m, size_t i, dReal *pos, dReal *R) {
    // 2d track is on XZ plane, with roller axes in Y direction
    pos[1] = 0;
    dReal theta;
    track_kinematic_model_get_point_on_path(m, i / (dReal)m->numGrousers, &pos[0], &pos[2], &theta);
    dRFromAxisAndAngle(R, 0, 1, 0, -theta);
}

typedef struct {
    TrackKinematicModel *m;
    dReal density;
    dBodyID trackBody;
    dBodyID wheel1Body;
    dBodyID wheel2Body;
    dMass trackMass;
    dMass wheel1Mass;
    dMass wheel2Mass;
    dGeomID wheel1Geom;
    dGeomID wheel2Geom;
    dJointID wheel1Joint;
    dJointID wheel2Joint;
    //dJointID guideJoint;
    dBodyID* grouserBody;
    dGeomID* grouserGeom;
    dJointID* grouserJoint;
    dMass* grouserMass;
} Track;
    
Track* track_create(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserHeight_, dReal trackDepth_, dWorldID world, dSpaceID space, dReal xOffset, dReal yOffset, dReal zOffset) {
    Track* t = (Track *)malloc(sizeof(Track));
    t->m = track_kinematic_model_create(radius1_, radius2_, distance_, numGrousers_, grouserHeight_, trackDepth_);
    t->density = 1.0;
    t->grouserBody = (dBodyID *)malloc(numGrousers_ * sizeof(dBodyID));
    t->grouserGeom = (dGeomID *)malloc(numGrousers_ * sizeof(dGeomID));
    t->grouserJoint = (dJointID *)malloc(numGrousers_ * sizeof(dJointID));
    t->grouserMass = (dMass *)malloc(numGrousers_ * sizeof(dMass));

    t->trackBody = dBodyCreate(world);
    dMassSetBox(&t->trackMass, t->density, t->m->distance, t->m->radius2, t->m->trackDepth);
    dBodySetMass(t->trackBody, &t->trackMass);
    
    t->wheel1Geom = dCreateCylinder(space, t->m->radius1, t->m->trackDepth);
    dGeomSetCategoryBits(t->wheel1Geom, 0x1);
    dGeomSetCollideBits(t->wheel1Geom, 0x2);
    dMassSetCylinder(&t->wheel1Mass, t->density, 3, t->m->radius1, t->m->trackDepth);
    t->wheel1Body = dBodyCreate(world);
    dBodySetMass(t->wheel1Body, &t->wheel1Mass);
    dGeomSetBody(t->wheel1Geom, t->wheel1Body);
    dBodySetPosition(t->wheel1Body, xOffset, yOffset, zOffset);
    dMatrix3 wheel1R;
    dRFromZAxis(wheel1R, 0, 1, 0);
    dBodySetRotation(t->wheel1Body, wheel1R);
    t->wheel1Joint = dJointCreateHinge(world, 0);
    dJointAttach(t->wheel1Joint, t->trackBody, t->wheel1Body);
    dJointSetHingeAnchor(t->wheel1Joint, xOffset, yOffset, zOffset);
    dJointSetHingeAxis(t->wheel1Joint, 0, 1, 0);
    
    t->wheel2Geom = dCreateCylinder(space, t->m->radius2, t->m->trackDepth);
    dGeomSetCategoryBits(t->wheel2Geom, 0x1);
    dGeomSetCollideBits(t->wheel2Geom, 0x2);
    dMassSetCylinder(&t->wheel2Mass, t->density, 3, t->m->radius2, t->m->trackDepth);
    t->wheel2Body = dBodyCreate(world);
    dBodySetMass(t->wheel2Body, &t->wheel2Mass);
    dGeomSetBody(t->wheel2Geom, t->wheel2Body);
    dBodySetPosition(t->wheel2Body, xOffset + t->m->distance, yOffset, zOffset);
    dMatrix3 wheel2R;
    dRFromZAxis(wheel2R, 0, 1, 0);
    dBodySetRotation(t->wheel2Body, wheel2R);
    t->wheel2Joint = dJointCreateHinge(world, 0);
    dJointAttach(t->wheel2Joint, t->trackBody, t->wheel2Body);
    dJointSetHingeAnchor(t->wheel2Joint, xOffset + t->m->distance, yOffset, zOffset);
    dJointSetHingeAxis(t->wheel2Joint, 0, 1, 0);

    dJointSetHingeParam(t->wheel2Joint, dParamFMax, 10);
    
    // grouser shrink/grow factor
    const dReal f = 0.8;
    
    for(size_t i = 0; i < t->m->numGrousers; i++) {
        t->grouserGeom[i] = dCreateBox(space, t->m->grouserHeight, t->m->trackDepth, f * t->m->grouserWidth);
        dGeomSetCategoryBits(t->grouserGeom[i], 0x2);
        dGeomSetCollideBits(t->grouserGeom[i], 0x1 | 0x4);
        dMassSetBox(&t->grouserMass[i], 10 * t->density, t->m->grouserHeight, t->m->trackDepth, f * t->m->grouserWidth);
        t->grouserBody[i] = dBodyCreate(world);
        dBodySetMass(t->grouserBody[i], &t->grouserMass[i]);
        dGeomSetBody(t->grouserGeom[i], t->grouserBody[i]);
        dVector3 pos; dMatrix3 R;
        track_kinematic_model_compute_grouser_transform_3D(t->m, i, pos, R);
        dBodySetPosition(t->grouserBody[i], xOffset + pos[0], yOffset + pos[1], zOffset + pos[2]);
        dBodySetRotation(t->grouserBody[i], R);

        // Disregard for now.
        // if (i == 0) {
        //     t->guideJoint = dJointCreateDHinge(world, 0);
        //     dJointAttach(t->guideJoint, t->wheel1Body, t->grouserBody[i]);
        //     dJointSetDHingeAxis (t->guideJoint, 0, 1, 0);
        //     dJointSetDHingeAnchor1 (t->guideJoint, xOffset, yOffset, zOffset);
        //     dJointSetDHingeAnchor2 (t->guideJoint, xOffset + pos[0], yOffset + pos[1], zOffset + pos[2]);
        // }
    }
    
    for(size_t i = 0; i < t->m->numGrousers; i++) {
        size_t j = (i + 1) % t->m->numGrousers;
        dReal px, pz, qx, qz, a, dx, dz;
        track_kinematic_model_get_point_on_path(t->m, i / (dReal)t->m->numGrousers, &px, &pz, &a);
        dx = cos(a - M_PI_2); dz = sin(a - M_PI_2);
        qx = px - t->m->grouserWidth * f * 0.5 * dx; qz = pz - t->m->grouserWidth * f * 0.5 * dz;
        px = px + t->m->grouserWidth * f * 0.5 * dx; pz = pz + t->m->grouserWidth * f * 0.5 * dz;
        t->grouserJoint[i] = dJointCreateHinge(world, 0);
        dJointAttach(t->grouserJoint[i], t->grouserBody[i], t->grouserBody[j]);
        dJointSetHingeAnchor(t->grouserJoint[i], xOffset + px, yOffset, zOffset + pz);
        dJointSetHingeAxis(t->grouserJoint[i], 0, 1, 0);
    }

    return t;
}

void track_destroy(Track* t) {
    free(t->grouserBody);
    free(t->grouserGeom);
    free(t->grouserJoint);
    free(t->grouserMass);
    free(t);
}

void track_draw(Track* t) {
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
    
    for(int i = 0; i < t->m->numGrousers; i++) {
        const dReal *pos = dGeomGetPosition(t->grouserGeom[i]);
        const dReal *R = dGeomGetRotation(t->grouserGeom[i]);
        dReal sides[3];
        dGeomBoxGetLengths(t->grouserGeom[i], sides);
        dsDrawBoxD(pos, R, sides);
    }
}

typedef struct {
    Track *leftTrack;
    Track *rightTrack;
    dReal density;
    dBodyID vehicleBody;
    dMass vehicleMass;
    dGeomID vehicleGeom;
    dJointID leftTrackJoint;
    dJointID rightTrackJoint;
    dReal width;
} TrackedVehicle;

TrackedVehicle* tracked_vehicle_create(dReal wheelRadius_, dReal wheelBase_, dReal trackWidth_, dReal vehicleWidth_, dWorldID world, dSpaceID space, dReal xOffset, dReal yOffset, dReal zOffset) {
    TrackedVehicle* v = (TrackedVehicle *)malloc(sizeof(TrackedVehicle));
    v->density = 1.0;
    v->width = vehicleWidth_;
    const size_t numGrousers = 30;
    const dReal grouserHeight = 0.01;
    dReal w = v->width + 2 * trackWidth_;
    v->leftTrack = track_create(wheelRadius_, wheelRadius_, wheelBase_, numGrousers, grouserHeight, trackWidth_, world, space, xOffset, yOffset - 0.5 * w, zOffset);
    v->rightTrack = track_create(wheelRadius_, wheelRadius_, wheelBase_, numGrousers, grouserHeight, trackWidth_, world, space, xOffset, yOffset + 0.5 * w, zOffset);
    v->vehicleBody = dBodyCreate(world);
    v->vehicleGeom = dCreateBox(space, v->leftTrack->m->distance, v->width, v->leftTrack->m->radius1);
    dMassSetBox(&v->vehicleMass, v->density, v->leftTrack->m->distance, v->width, v->leftTrack->m->radius1);
    dGeomSetCategoryBits(v->vehicleGeom, 0x0);
    dGeomSetCollideBits(v->vehicleGeom, 0x0);
    dBodySetMass(v->vehicleBody, &v->vehicleMass);
    dBodySetPosition(v->vehicleBody, xOffset, yOffset, zOffset);
    dGeomSetBody(v->vehicleGeom, v->vehicleBody);
    v->leftTrackJoint = dJointCreateFixed(world, 0);
    v->rightTrackJoint = dJointCreateFixed(world, 0);
    dJointAttach(v->leftTrackJoint, v->vehicleBody, v->leftTrack->trackBody);
    dJointAttach(v->rightTrackJoint, v->vehicleBody, v->rightTrack->trackBody);
    dJointSetFixed(v->leftTrackJoint);
    dJointSetFixed(v->rightTrackJoint);
    return v;
}

void tracked_vehicle_draw(TrackedVehicle *v) {
    {
        const dReal *pos = dGeomGetPosition(v->vehicleGeom);
        const dReal *R = dGeomGetRotation(v->vehicleGeom);
        dReal sides[3];
        dGeomBoxGetLengths(v->vehicleGeom, sides);
        dsDrawBoxD(pos, R, sides);
    }
    
    track_draw(v->leftTrack);
    track_draw(v->rightTrack);
}

#define MAX_CONTACTS 10

dWorldID world;
dSpaceID space;
dJointGroupID contactGroup;

dGeomID planeGeom;

TrackedVehicle *v;

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
    int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
    for(i = 0; i < numc; i++) {
        dJointID c = dJointCreateContact(world, contactGroup, contact + i);
        dJointAttach(c, b1, b2);
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
    tracked_vehicle_draw(v);
}

void stop() {
}

void command(int cmd) {
    const dReal V = 5;
    switch(cmd) {
        case 'a':
            dJointSetHingeParam(v->rightTrack->wheel2Joint, dParamVel, V);
            dJointSetHingeParam(v->leftTrack->wheel2Joint, dParamVel, -V);
            break;
        case 'd':
            dJointSetHingeParam(v->rightTrack->wheel2Joint, dParamVel, -V);
            dJointSetHingeParam(v->leftTrack->wheel2Joint, dParamVel, V);
            break;
        case 'w':
            dJointSetHingeParam(v->rightTrack->wheel2Joint, dParamVel, V);
            dJointSetHingeParam(v->leftTrack->wheel2Joint, dParamVel, V);
            break;
        case 's':
            dJointSetHingeParam(v->rightTrack->wheel2Joint, dParamVel, -V);
            dJointSetHingeParam(v->leftTrack->wheel2Joint, dParamVel, -V);
            break;
        case ' ':
            dJointSetHingeParam(v->rightTrack->wheel2Joint, dParamVel, 0);
            dJointSetHingeParam(v->leftTrack->wheel2Joint, dParamVel, 0);
            break;
        default:
            printf("cmd=%d\n", cmd);
            break;
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
    
    v = tracked_vehicle_create(0.3, 0.8, 0.2, 0.5, world, space, 0, 0, 0.301);

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

