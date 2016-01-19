/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#ifndef _ODE_PLANAR_JOINT_H_
#define _ODE_PLANAR_JOINT_H_

#include <stdint.h>
typedef uint8_t uint8;

#include "../../ode/ode/src/joints/joint.h"


// 2d joint

struct dxPlanarJoint : public dxJoint
{
public:
    dVector3 anchor;
    dVector3 planeNormal;


    dxPlanarJoint( dxWorld *w );
    virtual void getSureMaxInfo( SureMaxInfo* info );
    virtual void getInfo1( Info1* info );
    virtual void getInfo2( dReal worldFPS, dReal worldERP, const Info2Descr* info );
    virtual dJointType type() const;
    virtual size_t size() const;

    virtual void updatePlane();

public:
    dVector3 axis1, axis2;
    dVector3 anchor1, anchor2;

};

ODE_API dJointID dJointCreatePlanar (dWorldID w, dJointGroupID group); // body 2 (or world) moves the plane, whereas body 1 restricts its movement to the plane
ODE_API void dJointPlanarSetAnchor(dJointID joint, dVector3 anchor);
ODE_API void dJointPlanarSetPlaneNormal(dJointID joint, dVector3 planeNormal);

#endif