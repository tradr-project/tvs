#include <ode/matrix.h>
#include <ode/odemath_legacy.h>
#include <ode/odemath.h>
#include <ode/objects.h>
#include <ostream>
#include <iostream>
#include "PlanarJoint.h"

#define checktype(j,t) dUASSERT(j->type() == dJointType##t, \
    "joint type is not " #t)


//****************************************************************************
// PlanarJoint
//****************************************************************************

dJointID dJointCreatePlanar(dWorldID w, dJointGroupID group) {
    dAASSERT (w);

    dxJoint *j;
    if (group) {
        j = group->alloc<dxPlanarJoint>(w);
    } else {
        j = new dxPlanarJoint(w);
    }
    return j;
}


dxPlanarJoint::dxPlanarJoint( dxWorld *w ) :
        dxJoint( w )
{
    dSetZero(anchor, 4);

    // default to plane z=0
    dSetZero(planeNormal, 4);
    planeNormal[2] = 1;

    dSetZero(axis1, 4);
    dSetZero(axis2, 4);
    dSetZero(anchor1, 4);
    dSetZero(anchor2, 4);
}


void
dxPlanarJoint::getSureMaxInfo( SureMaxInfo* info )
{
    info->max_m = 3;
}


void
dxPlanarJoint::getInfo1( dxJoint::Info1 *info )
{
    info->nub = 3;
    info->m = 3;
}

void
dxPlanarJoint::getInfo2( dReal worldFPS, dReal worldERP, const Info2Descr *info )
{
    int r0 = 0;
    int r1 = info->rowskip;
    int r2 = 2 * r1;
    dReal eps = worldFPS * worldERP;

    // ======================================================================
    // Work on the angular part (i.e. row 0, 1)
    // Set the two orientation rows. The rotoide axis should be the only
    // unconstrained rotational axis, the angular velocity of the two bodies
    // perpendicular to the rotoide axis should be equal.
    // Thus the constraint equations are:
    //    p*w1 - p*w2 = 0
    //    q*w1 - q*w2 = 0
    // where p and q are unit vectors normal to the rotoide axis, and w1 and w2
    // are the angular velocity vectors of the two bodies.
    // Since the rotoide axis is the same as the prismatic axis.
    //
    //
    // Also, compute the right hand side (RHS) of the rotation constraint equation set.
    // The first 2 element will result in the relative angular velocity of the two
    // bodies along axis p and q. This is set to bring the rotoide back into alignment.
    // if `theta' is the angle between ax1 and ax2, we need an angular velocity
    // along u to cover angle erp*theta in one step :
    //   |angular_velocity| = angle/time = erp*theta / stepsize
    //                      = (erp*fps) * theta
    //    angular_velocity  = |angular_velocity| * u
    //                      = (erp*fps) * theta * u
    // where rotation along unit length axis u by theta brings body 2's frame
    //
    // if theta is smallish, sin(theta) ~= theta and cos(theta) ~= 1
    // where the quaternion of the relative rotation between the two bodies is
    //    quat = [cos(theta/2) sin(theta/2)*u]
    //    quat = [1 theta/2*u]
    //         => q[0] ~= 1
    //            2 * q[1+i] = theta * u[i]
    //
    // Since there is no constraint along the rotoide axis
    // only along p and q that we want the same angular velocity and need to reduce
    // the error

    dVector3 ax1;
    if (node[1].body) {
        dBodyVectorToWorld(node[1].body, axis2[0], axis2[1], axis2[2], ax1);
    } else {
        dCopyVector3(ax1, axis2);
    }

    // Find the 2 direction vectors of the plane.
    dVector3 p, q;
    dPlaneSpace ( ax1, p, q );

    // LHS 2
    dCopyVector3 ( ( info->J2a ) + r0, p );
    dCopyVector3 ( ( info->J2a ) + r1, q );

    // LHS 1
    dCopyNegatedVector3 ( ( info->J1a ) + r0, p );
    dCopyNegatedVector3 ( ( info->J1a ) + r1, q );


    // Some math for the RHS
    dVector3 ax2;
    dBodyVectorToWorld(node[0].body, axis1[0], axis1[1], axis1[2], ax2);

    dVector3 b;
    dCalcVectorCross3( b, ax1, ax2 );

    // RHS
    info->c[0] = eps * dCalcVectorDot3 ( p, b );
    info->c[1] = eps * dCalcVectorDot3 ( q, b );

    // linear velocity constraint (removes 1 translational DOF along the plane normal)

    // node[0] je vazany na rovinu, rovina je vazana na node[1] (pripadne svet)

    dVector3 globalAxis;
    if (node[1].body) {
        dBodyVectorToWorld(node[1].body, axis2[0], axis2[1], axis2[2], globalAxis);
    } else {
        dCopyVector3(globalAxis, axis2);
    }

    dCopyNegatedVector3(info->J1l + r2, globalAxis);
    dCopyVector3(info->J2l + r2, globalAxis);


    dVector3 pos1;
    dCopyVector3(pos1, node[0].body->posr.pos);

    dVector3 pos2 = {0, 0, 0};
    if (node[1].body) {
        dCopyVector3(pos2, node[1].body->posr.pos);
    }

    dVector3 pos2_pos1;
    dSubtractVectors3(pos2_pos1, pos2, pos1);

    dVector3 pos2_pos1LocalToBody2;
    if (node[1].body) {
        dBodyVectorFromWorld(node[1].body, pos2_pos1[0], pos2_pos1[1], pos2_pos1[2], pos2_pos1LocalToBody2);
    } else {
        dCopyVector3(pos2_pos1LocalToBody2, pos2_pos1);
    }

    dCalcVectorCross3(info->J1a + r2, axis1, pos2_pos1LocalToBody2);

    // RHS

    dReal t1, t2, t3;
    t1 = dCalcVectorDot3(globalAxis, pos1);
    t2 = dCalcVectorDot3(axis2, anchor2);
    t3 = dCalcVectorDot3(globalAxis, pos2);

    info->c[2] = eps * (t1 - t2 - t3);

    if (t1 - t2 - t3 > 1) {
        std::cout << (t1 - t2 - t3) << std::endl;
    }
}


dJointType
dxPlanarJoint::type() const
{
    return dJointTypePlane2D;
}


size_t
dxPlanarJoint::size() const
{
    return sizeof( *this );
}

void dJointPlanarSetAnchor(dJointID joint, dVector3 anchor) {
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, Plane2D );
    dxPlanarJoint* planarJoint = ( dxPlanarJoint* )( joint );

    planarJoint->anchor[0] = anchor[0];
    planarJoint->anchor[1] = anchor[1];
    planarJoint->anchor[2] = anchor[2];

    planarJoint->updatePlane();
}

void dJointPlanarSetPlaneNormal(dJointID joint, dVector3 planeNormal) {
    dUASSERT( joint, "bad joint argument" );
    dUASSERT(dLENGTHSQUARED(planeNormal) > 0.000001, "plane normal cannot have zero length");

    checktype( joint, Plane2D );
    dxPlanarJoint* planarJoint = ( dxPlanarJoint* )( joint );

    planarJoint->planeNormal[0] = planeNormal[0];
    planarJoint->planeNormal[1] = planeNormal[1];
    planarJoint->planeNormal[2] = planeNormal[2];

    dNormalize3(planarJoint->planeNormal);

    planarJoint->updatePlane();
}


void dxPlanarJoint::updatePlane() {

    dBodyID b0 = this->node[0].body;
    if ( b0 )
    {

        dBodyVectorFromWorld(b0, planeNormal[0], planeNormal[1], planeNormal[2], axis1);
        dBodyGetPosRelPoint(b0, anchor[0], anchor[1], anchor[2], anchor1);

        dBodyID b1 = this->node[1].body;
        if ( b1 )
        {
            dBodyVectorFromWorld(b1, planeNormal[0], planeNormal[1], planeNormal[2], axis2);
            dBodyGetPosRelPoint(b1, anchor[0], anchor[1], anchor[2], anchor2);
        }
        else
        {
            dCopyVector3(axis2, planeNormal);
            dCopyVector3(anchor2, anchor);
        }
    }
}
