//
//  ODEUtils.h
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef __tvs__ODEUtils__
#define __tvs__ODEUtils__

#include <ode/ode.h>

void dRigidBodyArraySetPosition(dBodyID *bodyArray, size_t arraySize, dBodyID center, dReal x, dReal y, dReal z);
void dRigidBodyArraySetRotation(dBodyID *bodyArray, size_t arraySize, dBodyID center, const dReal *Rs);
void dRigidBodyArraySetLinearVel(dBodyID *bodyArray, size_t arraySize, dBodyID center, dReal lx, dReal ly, dReal lz);
void dRigidBodyArraySetAngularVel(dBodyID *bodyArray, size_t arraySize, dBodyID center, dReal ax, dReal ay, dReal az);
void dRigidBodyArrayAddLinearVel(dBodyID *bodyArray, size_t arraySize, dBodyID center, dReal lx, dReal ly, dReal lz);
void dRigidBodyArraySetVel(dBodyID *bodyArray, size_t arraySize, dBodyID center, dReal lx, dReal ly, dReal lz, dReal ax, dReal ay, dReal az);
const char * dClassGetName(int c);

#endif /* defined(__tvs__ODEUtils__) */
