//
//  track.c
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "track.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <drawstuff/drawstuff.h>

Track * track_init(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserHeight_, dReal trackDepth_, dReal xOffset, dReal yOffset, dReal zOffset) {
    Track *t = (Track *)malloc(sizeof(Track));
    t->m = track_kinematic_model_init(radius1_, radius2_, distance_, numGrousers_, grouserHeight_, trackDepth_);
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

void track_create(Track *t, dWorldID world, dSpaceID space) {
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
    dBodySetPosition(t->wheel1Body, t->xOffset, t->yOffset, t->zOffset);
    dMatrix3 wheel1R;
    dRFromZAxis(wheel1R, 0, 1, 0);
    dBodySetRotation(t->wheel1Body, wheel1R);
    t->wheel1Joint = dJointCreateHinge(world, 0);
    dJointAttach(t->wheel1Joint, t->trackBody, t->wheel1Body);
    dJointSetHingeAnchor(t->wheel1Joint, t->xOffset, t->yOffset, t->zOffset);
    dJointSetHingeAxis(t->wheel1Joint, 0, 1, 0);

    t->wheel2Geom = dCreateCylinder(space, t->m->radius2, t->m->trackDepth);
    dGeomSetCategoryBits(t->wheel2Geom, 0x1);
    dGeomSetCollideBits(t->wheel2Geom, 0x2);
    dMassSetCylinder(&t->wheel2Mass, t->density, 3, t->m->radius2, t->m->trackDepth);
    t->wheel2Body = dBodyCreate(world);
    dBodySetMass(t->wheel2Body, &t->wheel2Mass);
    dGeomSetBody(t->wheel2Geom, t->wheel2Body);
    dBodySetPosition(t->wheel2Body, t->xOffset + t->m->distance, t->yOffset, t->zOffset);
    dMatrix3 wheel2R;
    dRFromZAxis(wheel2R, 0, 1, 0);
    dBodySetRotation(t->wheel2Body, wheel2R);
    t->wheel2Joint = dJointCreateHinge(world, 0);
    dJointAttach(t->wheel2Joint, t->trackBody, t->wheel2Body);
    dJointSetHingeAnchor(t->wheel2Joint, t->xOffset + t->m->distance, t->yOffset, t->zOffset);
    dJointSetHingeAxis(t->wheel2Joint, 0, 1, 0);

    dJointSetHingeParam(t->wheel2Joint, dParamFMax, 10);

    // grouser shrink/grow factor
    const dReal f = 1.03;
    size_t i;

    for(i = 0; i < t->m->numGrousers; i++) {
        t->grouserGeom[i] = dCreateBox(space, t->m->grouserHeight, t->m->trackDepth, f * t->m->grouserWidth);
        dGeomSetCategoryBits(t->grouserGeom[i], 0x2);
        dGeomSetCollideBits(t->grouserGeom[i], 0x1 | 0x4);
        dMassSetBox(&t->grouserMass[i], 10 * t->density, t->m->grouserHeight, t->m->trackDepth, f * t->m->grouserWidth);
        t->grouserBody[i] = dBodyCreate(world);
        dBodySetMass(t->grouserBody[i], &t->grouserMass[i]);
        dGeomSetBody(t->grouserGeom[i], t->grouserBody[i]);
        dVector3 pos; dMatrix3 R;
        track_kinematic_model_compute_grouser_transform_3D(t->m, i, pos, R);
        dBodySetPosition(t->grouserBody[i], t->xOffset + pos[0], t->yOffset + pos[1], t->zOffset + pos[2]);
        dBodySetRotation(t->grouserBody[i], R);

        // Disregard for now.
        // if(i == 0) {
        //     t->guideJoint = dJointCreateDHinge(world, 0);
        //     dJointAttach(t->guideJoint, t->wheel1Body, t->grouserBody[i]);
        //     dJointSetDHingeAxis(t->guideJoint, 0, 1, 0);
        //     dJointSetDHingeAnchor1(t->guideJoint, xOffset, yOffset, zOffset);
        //     dJointSetDHingeAnchor2(t->guideJoint, xOffset + pos[0], yOffset + pos[1], zOffset + pos[2]);
        // }
    }

    for(i = 0; i < t->m->numGrousers; i++) {
        size_t j = (i + 1) % t->m->numGrousers;
        dReal px, pz, qx, qz, a, dx, dz;
        track_kinematic_model_get_point_on_path(t->m, i / (dReal)t->m->numGrousers, &px, &pz, &a);
        dx = cos(a - M_PI_2);
        dz = sin(a - M_PI_2);
        qx = px - t->m->grouserWidth * f * 0.5 * dx;
        qz = pz - t->m->grouserWidth * f * 0.5 * dz;
        px = px + t->m->grouserWidth * f * 0.5 * dx;
        pz = pz + t->m->grouserWidth * f * 0.5 * dz;
        t->grouserJoint[i] = dJointCreateHinge(world, 0);
        dJointAttach(t->grouserJoint[i], t->grouserBody[i], t->grouserBody[j]);
        dJointSetHingeAnchor(t->grouserJoint[i], t->xOffset + px, t->yOffset, t->zOffset + pz);
        dJointSetHingeAxis(t->grouserJoint[i], 0, 1, 0);
    }
}

void track_destroy(Track *t) {
}

void track_deinit(Track *t) {
    track_kinematic_model_deinit(t->m);
    free(t->grouserBody);
    free(t->grouserGeom);
    free(t->grouserJoint);
    free(t->grouserMass);
    free(t);
}

void track_draw(Track *t) {
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
}

