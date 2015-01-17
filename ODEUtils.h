//
//  ODEUtils.h
//  tvs
//
//  Created by Federico Ferri on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef __tvs__ODEUtils__
#define __tvs__ODEUtils__

#include <ode/ode.h>

struct dxRigidBodyArray;
typedef struct dxRigidBodyArray *dRigidBodyArrayID;

struct dxRigidBodyArray {
    dBodyID body;
    dRigidBodyArrayID next;
    dxRigidBodyArray() : body(0L), next(0L) {}
    dxRigidBodyArray(dBodyID body_) : body(body_), next(0L) {}
    dxRigidBodyArray(dBodyID body_, dRigidBodyArrayID next_) : body(body_), next(next_) {}
};

dRigidBodyArrayID dRigidBodyArrayCreate(dBodyID centerBody);
void dRigidBodyArrayAdd(dRigidBodyArrayID bodyArray, dBodyID body);
void dRigidBodyArrayAdd(dRigidBodyArrayID bodyArray, dRigidBodyArrayID bodyArray2);
void dRigidBodyArrayDestroy(dRigidBodyArrayID bodyArray);
void dRigidBodyArraySetPosition(dRigidBodyArrayID bodyArray, dReal x, dReal y, dReal z);
void dRigidBodyArraySetRotationRelative(dRigidBodyArrayID bodyArray, const dReal *Rs);
void dRigidBodyArraySetRotation(dRigidBodyArrayID bodyArray, const dReal *Rs);
void dRigidBodyArraySetQuaternion(dRigidBodyArrayID bodyArray, const dReal *q);
void dRigidBodyArraySetLinearVel(dRigidBodyArrayID bodyArray, dReal lx, dReal ly, dReal lz);
void dRigidBodyArraySetAngularVel(dRigidBodyArrayID bodyArray, dReal ax, dReal ay, dReal az);
void dRigidBodyArrayAddLinearVel(dRigidBodyArrayID bodyArray, dReal lx, dReal ly, dReal lz);
void dRigidBodyArraySetVel(dRigidBodyArrayID bodyArray, dReal lx, dReal ly, dReal lz, dReal ax, dReal ay, dReal az);
const char * dClassGetName(int c);

struct dLine {
    dVector3 a, b;
};

#endif /* defined(__tvs__ODEUtils__) */
