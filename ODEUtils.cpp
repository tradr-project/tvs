//
//  ODEUtils.cpp
//  tvs
//
//  Created by Federico Ferri on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "ODEUtils.h"

dRigidBodyArrayID dRigidBodyArrayCreate(dBodyID centerBody) {
    return new dxRigidBodyArray(centerBody);
}

void dRigidBodyArrayAdd(dRigidBodyArrayID bodyArray, dBodyID body) {
    bodyArray->bodies.push_back(body);
}

void dRigidBodyArrayDestroy(dRigidBodyArrayID bodyArray) {
    delete bodyArray;
}

size_t dRigidBodyArraySize(dRigidBodyArrayID bodyArray) {
    return bodyArray->bodies.size() + 1;
}

dBodyID dRigidBodyArrayGet(dRigidBodyArrayID bodyArray, size_t i) {
    if(i == 0) return bodyArray->center;
    else return bodyArray->bodies[i - 1];
}

void dRigidBodyArraySetPosition(dRigidBodyArrayID bodyArray, dReal x, dReal y, dReal z) {
    dBodyID center = bodyArray->body;
    const dReal *p0 = dBodyGetPosition(center);
    dVector3 ps = {x, y, z};
    dVector3 ps_p0;
    dOP(ps_p0, -, ps, p0);
    for(size_t i = 0; i < dRigidBodyArraySize(bodyArray); i++) {
        dBodyID body = dRigidBodyArrayGet(bodyArray, i);
        const dReal *pi = dBodyGetPosition(body);
        dVector3 p1;
        dOP(p1, +, ps_p0, pi);
        dBodySetPosition(body, p1[0], p1[1], p1[2]);
    }
}

void dRigidBodyArraySetRotationRelative(dRigidBodyArrayID bodyArray, const dReal *Rs) {
    dBodyID center = bodyArray->body;
    const dReal *p0 = dBodyGetPosition(center);
    const dReal *R0 = dBodyGetRotation(center);
    dMatrix3 R0Rs;
    dMULTIPLY0_333(R0Rs, R0, Rs);
    dMatrix3 R0RsR0t;
    dMULTIPLY2_333(R0RsR0t, R0Rs, R0);
    for(size_t i = 0; i < dRigidBodyArraySize(bodyArray); i++) {
        dBodyID body = dRigidBodyArrayGet(bodyArray, i);
        const dReal *pi = dBodyGetPosition(body);
        const dReal *Ri = dBodyGetRotation(body);
        dMatrix3 R1;
        dMULTIPLY0_333(R1, R0RsR0t, Ri);
        dVector3 p1, pi_p0, R0RsR0t__pi_p0;
        dOP(pi_p0, -, pi, p0);
        dMULTIPLY0_331(R0RsR0t__pi_p0, R0RsR0t, pi_p0);
        dOP(p1, +, R0RsR0t__pi_p0, p0);
        dBodySetPosition(body, p1[0], p1[1], p1[2]);
        dBodySetRotation(body, R1);
    }
}

void dRigidBodyArraySetRotation(dRigidBodyArrayID bodyArray, const dReal *Rs) {
    dBodyID center = bodyArray->body;
    const dReal *p0 = dBodyGetPosition(center);
    const dReal *R0 = dBodyGetRotation(center);
    dMatrix3 RsR0t;
    dMULTIPLY2_333(RsR0t, Rs, R0);
    for(size_t i = 0; i < dRigidBodyArraySize(bodyArray); i++) {
        dBodyID body = dRigidBodyArrayGet(bodyArray, i);
        const dReal *pi = dBodyGetPosition(body);
        const dReal *Ri = dBodyGetRotation(body);
        dMatrix3 R1;
        dMULTIPLY0_333(R1, RsR0t, Ri);
        dVector3 p1, pi_p0, R0RsR0t__pi_p0;
        dOP(pi_p0, -, pi, p0);
        dMULTIPLY0_331(R0RsR0t__pi_p0, RsR0t, pi_p0);
        dOP(p1, +, R0RsR0t__pi_p0, p0);
        dBodySetPosition(body, p1[0], p1[1], p1[2]);
        dBodySetRotation(body, R1);
    }
}

void dRigidBodyArraySetQuaternion(dRigidBodyArrayID bodyArray, const dReal *q) {
    dMatrix3 R;
    dRfromQ(R, q);
    dRigidBodyArraySetRotation(bodyArray, R);
}

void dRigidBodyArraySetLinearVel(dRigidBodyArrayID bodyArray, dReal lx, dReal ly, dReal lz) {
    for(size_t i = 0; i < dRigidBodyArraySize(bodyArray); i++) {
        dBodyID body = dRigidBodyArrayGet(bodyArray, i);
        dBodySetLinearVel(body, lx, ly, lz);
    }
}

void dRigidBodyArraySetAngularVel(dRigidBodyArrayID bodyArray, dReal ax, dReal ay, dReal az) {
    dBodyID center = bodyArray->body;
    const dReal *p0 = dBodyGetPosition(center);
    dVector3 omega = {ax, ay, az};
    for(size_t i = 0; i < dRigidBodyArraySize(bodyArray); i++) {
        dBodyID body = dRigidBodyArrayGet(bodyArray, i);
        const dReal *p = dBodyGetPosition(body);
        dVector3 pdot, r;
        dOP(r, -, p, p0);
        dCalcVectorCross3(pdot, omega, r);
        dBodySetLinearVel(body, pdot[0], pdot[1], pdot[2]);
        dBodySetAngularVel(body, ax, ay, az);
    }
}

void dRigidBodyArrayAddLinearVel(dRigidBodyArrayID bodyArray, dReal lx, dReal ly, dReal lz) {
    for(size_t i = 0; i < dRigidBodyArraySize(bodyArray); i++) {
        dBodyID body = dRigidBodyArrayGet(bodyArray, i);
        const dReal *v = dBodyGetLinearVel(body);
        dBodySetLinearVel(body, v[0] + lx, v[1] + ly, v[2] + lz);
    }
}

void dRigidBodyArraySetVel(dRigidBodyArrayID bodyArray, dReal lx, dReal ly, dReal lz, dReal ax, dReal ay, dReal az) {
    dRigidBodyArraySetAngularVel(bodyArray, ax, ay, az);
    dRigidBodyArrayAddLinearVel(bodyArray, lx, ly, lz);
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
