//
//  ODEUtils.cpp
//  tvs
//
//  Created by Main User on 18/12/2014.
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

void dRigidBodyArraySetRotation(dBodyID *bodyArray, size_t arraySize, dBodyID center, const dReal *Rs) {
    const dReal *p0 = dBodyGetPosition(center);
    const dReal *R0 = dBodyGetRotation(center);
    dMatrix3 R0Rs;
    dMULTIPLY0_333(R0Rs, R0, Rs);
    dMatrix3 R0RsR0t;
    dMULTIPLY2_333(R0RsR0t, R0Rs, R0);
    dVector3 R0Rsp0;
    dMULTIPLY0_331(R0Rsp0, R0Rs, p0);
    for(size_t i = 0; i < arraySize; i++) {
        const dReal *pi = dBodyGetPosition(bodyArray[i]);
        const dReal *Ri = dBodyGetRotation(bodyArray[i]);
        dMatrix3 R1;
        dMULTIPLY0_333(R1, R0RsR0t, Ri);
        dVector3 p1, R0RsR0tpi, R0RsR0tpi_R0Rsp0;
        dMULTIPLY0_331(R0RsR0tpi, R0RsR0t, pi);
        dOP(R0RsR0tpi_R0Rsp0, -, R0RsR0tpi, R0Rsp0);
        dOP(p1, +, R0RsR0tpi_R0Rsp0, p0);
        dBodySetPosition(bodyArray[i], p1[0], p1[1], p1[2]);
        dBodySetRotation(bodyArray[i], R1);
    }
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
