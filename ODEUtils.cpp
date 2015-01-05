//
//  ODEUtils.cpp
//  tvs
//
//  Created by Federico Ferri on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "ODEUtils.h"

void dRigidBodyArraySetPosition(dBodyID *bodyArray, size_t arraySize, dBodyID center, dReal x, dReal y, dReal z) {
    const dReal *p0 = dBodyGetPosition(center);
    dVector3 ps = {x, y, z};
    dVector3 ps_p0;
    dOP(ps_p0, -, ps, p0);
    for(size_t i = 0; i < arraySize; i++) {
        const dReal *pi = dBodyGetPosition(bodyArray[i]);
        dVector3 p1;
        dOP(p1, +, ps_p0, pi);
        dBodySetPosition(bodyArray[i], p1[0], p1[1], p1[2]);
    }
}

void dRigidBodyArraySetRotationRelative(dBodyID *bodyArray, size_t arraySize, dBodyID center, const dReal *Rs) {
    const dReal *p0 = dBodyGetPosition(center);
    const dReal *R0 = dBodyGetRotation(center);
    dMatrix3 R0Rs;
    dMULTIPLY0_333(R0Rs, R0, Rs);
    dMatrix3 R0RsR0t;
    dMULTIPLY2_333(R0RsR0t, R0Rs, R0);
    for(size_t i = 0; i < arraySize; i++) {
        const dReal *pi = dBodyGetPosition(bodyArray[i]);
        const dReal *Ri = dBodyGetRotation(bodyArray[i]);
        dMatrix3 R1;
        dMULTIPLY0_333(R1, R0RsR0t, Ri);
        dVector3 p1, pi_p0, R0RsR0t__pi_p0;
        dOP(pi_p0, -, pi, p0);
        dMULTIPLY0_331(R0RsR0t__pi_p0, R0RsR0t, pi_p0);
        dOP(p1, +, R0RsR0t__pi_p0, p0);
        dBodySetPosition(bodyArray[i], p1[0], p1[1], p1[2]);
        dBodySetRotation(bodyArray[i], R1);
    }
}

void dRigidBodyArraySetRotation(dBodyID *bodyArray, size_t arraySize, dBodyID center, const dReal *Rs) {
    const dReal *p0 = dBodyGetPosition(center);
    const dReal *R0 = dBodyGetRotation(center);
    dMatrix3 RsR0t;
    dMULTIPLY2_333(RsR0t, Rs, R0);
    for(size_t i = 0; i < arraySize; i++) {
        const dReal *pi = dBodyGetPosition(bodyArray[i]);
        const dReal *Ri = dBodyGetRotation(bodyArray[i]);
        dMatrix3 R1;
        dMULTIPLY0_333(R1, RsR0t, Ri);
        dVector3 p1, pi_p0, R0RsR0t__pi_p0;
        dOP(pi_p0, -, pi, p0);
        dMULTIPLY0_331(R0RsR0t__pi_p0, RsR0t, pi_p0);
        dOP(p1, +, R0RsR0t__pi_p0, p0);
        dBodySetPosition(bodyArray[i], p1[0], p1[1], p1[2]);
        dBodySetRotation(bodyArray[i], R1);
    }
}

void dRigidBodyArraySetQuaternion(dBodyID *bodyArray, size_t arraySize, dBodyID center, const dReal *q) {
    dMatrix3 R;
    dRfromQ(R, q);
    dRigidBodyArraySetRotation(bodyArray, arraySize, center, R);
}

void dRigidBodyArraySetLinearVel(dBodyID *bodyArray, size_t arraySize, dBodyID center, dReal lx, dReal ly, dReal lz) {
    for(size_t i = 0; i < arraySize; i++) {
        dBodySetLinearVel(bodyArray[i], lx, ly, lz);
    }
}

void dRigidBodyArraySetAngularVel(dBodyID *bodyArray, size_t arraySize, dBodyID center, dReal ax, dReal ay, dReal az) {
    const dReal *p0 = dBodyGetPosition(center);
    dVector3 omega = {ax, ay, az};
    for(size_t i = 0; i < arraySize; i++) {
        const dReal *p = dBodyGetPosition(bodyArray[i]);
        dVector3 pdot, r;
        dOP(r, -, p, p0);
        dCalcVectorCross3(pdot, omega, r);
        dBodySetLinearVel(bodyArray[i], pdot[0], pdot[1], pdot[2]);
        dBodySetAngularVel(bodyArray[i], ax, ay, az);
    }
}

void dRigidBodyArrayAddLinearVel(dBodyID *bodyArray, size_t arraySize, dBodyID center, dReal lx, dReal ly, dReal lz) {
    for(size_t i = 0; i < arraySize; i++) {
        const dReal *v = dBodyGetLinearVel(bodyArray[i]);
        dBodySetLinearVel(bodyArray[i], v[0] + lx, v[1] + ly, v[2] + lz);
    }
}

void dRigidBodyArraySetVel(dBodyID *bodyArray, size_t arraySize, dBodyID center, dReal lx, dReal ly, dReal lz, dReal ax, dReal ay, dReal az) {
    dRigidBodyArraySetAngularVel(bodyArray, arraySize, center, ax, ay, az);
    dRigidBodyArrayAddLinearVel(bodyArray, arraySize, center, lx, ly, lz);
}

const char * dClassGetName(int c) {
    switch(c) {
        case dSphereClass: return "sphere";
        case dBoxClass: return "box";
        case dCapsuleClass: return "capsule";
        case dCylinderClass: return "cylinder";
        case dPlaneClass: return "plane";
        case dRayClass: return "ray";
        case dConvexClass: return "convex";
        case dGeomTransformClass: return "geom-transform";
        case dTriMeshClass: return "tri-mesh";
        case dHeightfieldClass: return "heightfield";
    };
    static char buf[20];
    snprintf(buf, 20, "class#%d", c);
    return buf;
}
