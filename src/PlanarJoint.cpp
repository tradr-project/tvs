#include <ode/matrix.h>
#include <ode/odemath_legacy.h>
#include <ode/odemath.h>
#include <ode/objects.h>
#include <ostream>
#include <iostream>
#include "PlanarJoint.h"

#define checktype(j,t) dUASSERT(j->type() == dJointType##t, \
    "joint type is not " #t)

dJointID dJointCreatePlanar(dWorldID world, dJointGroupID group) {
    dAASSERT (world);

    dxJoint *joint;
    if (group) {
        joint = group->alloc<dxPlanarJoint>(world);
    } else {
        joint = new dxPlanarJoint(world);
    }
    return joint;
}


dxPlanarJoint::dxPlanarJoint(dxWorld *world) :
        dxJoint(world)
{
    dSetZero(anchor, 3);
    dCopyVector3(anchor1, anchor);
    dCopyVector3(anchor2, anchor);

    // default to plane z=0
    dSetZero(planeNormal, 3);
    planeNormal[2] = 1;
    dCopyVector3(axis1, planeNormal);
    dCopyVector3(axis2, planeNormal);
}


void dxPlanarJoint::getSureMaxInfo(SureMaxInfo* info)
{
    info->max_m = 3;
}


void dxPlanarJoint::getInfo1(dxJoint::Info1* info)
{
    info->nub = 3;
    info->m = 3;
}

void dxPlanarJoint::getInfo2(dReal worldFPS, dReal worldERP, const Info2Descr* info)
{
    /* ====================================================================
     * This joint restricts 2 rotational and 1 translational DOF.
     *
     * The 2 rotational constraints are essentially the same as in dxJointPiston, so the implementation here is mostly
     * just a copy of them.
     *
     * The translational 1DOF constraint has not yet been implemented for any joint in ODE, so we'll talk more about it.
     * It is similar to the constraints in a slider joint, but in this case we only want body2 to move the plane, and
     * body1 anchor has no use in this case, too.
     *
     * We basically want to express the plane in body2 local coordinates (because it should move along with body2), and
     * check, if body1 origin lies in the plane.
     *
     * Let's say n' is the plane normal in body2 coordinates and a' is the anchor point in body2 coordinates
     * (with n, a, being the corresponding global variables).
     * Further, let's call the global position of body1 p1, and similarly p2 for body2.
     * Last, let's call body2 rotation matrix R2 (so that n = R2*n' and a = R2*a' + p2).
     *
     * The plane can then be expressed in global coordinates as:
     * (n^ . x) - (n^ . a) = 0,
     * where x is in global coordinates and ^ denotes vector/matrix transpose.
     *
     * Rewriting the equation using body2 local coordinates and setting x=p1, we get the constraint:
     * (R2*n')^ * p1 - (R2*n')^ * (R2*a' + p2) = 0
     * ...
     * (n'^ * R2^ * p1) - (n'^ * R2^ * p2) - (n'^ * a') = 0
     * (n^ * p1) - (n^ * p2) - (n'^ * a') = 0
     *
     * The part left to "=" will be the "c" on the RHS of the Jacobian equation (because it expresses
     * the distance of p1 from the plane).
     *
     * Next, we need to take time derivative of the constraint to get the J1 and J2 parts.
     * v1, v2, w1 and w2 denote the linear and angular velocities od body1 and body2.
     * [a]x denotes the skew-symmetric cross-product matrix of vector a.
     *
     * d/dt [(n'^ * R2^ * p1) - (n'^ * R2^ * p2) - (n'^ * a') = 0]        (n', a' are constant in time)
     * n'^ * ((d/dt[R2^] * p1) + (R2^ * v1)) - n'^ * ((d/dt[R2^] * p2) + (R2^ * v2)) = 0
     * n'^ * ((([w2]x R2)^ * p1) + (R2^ * v1)) - n'^ * ((([w2]x R2)^ * p2) + (R2^ * v2)) = 0
     * ...
     * n^*v1 - n^*v2 + [n]x (p1 - p2) . w2 = 0
     * -n^*v1 + n^*v2 + [n]x (p2 - p1) . w2 = 0
     *
     * Thus we see that
     * J1l = -n
     * J2l = n
     * J1a = 0
     * J2a = [n]x (p2 - p1)
     * c = eps * ( (n^ * p1) - (n^ * a) )
     *
     */

    const int row0Index = 0;
    const int row1Index = info->rowskip;
    const int row2Index = 2 * row1Index;
    const dReal eps = worldFPS * worldERP;

    dVector3 globalAxis1;
    dBodyVectorToWorld(node[0].body, axis1[0], axis1[1], axis1[2], globalAxis1);

    dVector3 globalAxis2;
    dVector3 globalAnchor;

    if (node[1].body) {
        dBodyVectorToWorld(node[1].body, axis2[0], axis2[1], axis2[2], globalAxis2);
        dBodyGetRelPointPos(node[1].body, anchor2[0], anchor2[1], anchor2[2], globalAnchor);
    } else {
        dCopyVector3(globalAxis2, axis2);
        dCopyVector3(globalAnchor, anchor2);
    }

    ///////////////////////////////////////////////////////
    // Angular velocity constraints (2 rows)
    //
    // Refer to the piston joint source code for details.
    ///////////////////////////////////////////////////////

    // Find the 2 direction vectors of the plane.
    dVector3 p, q;
    dPlaneSpace ( globalAxis2, p, q );

    // LHS 0
    dCopyNegatedVector3 (info->J1a + row0Index, p );
    dCopyNegatedVector3 (info->J1a + row1Index, q );

    // LHS 1
    dCopyVector3 (info->J2a + row0Index, p );
    dCopyVector3 (info->J2a + row1Index, q );

    // RHS 0 & 1 (absolute errors of the axis direction computed by dot product of the desired and actual axis)
    dVector3 b;
    dCalcVectorCross3( b, globalAxis2, globalAxis1 );
    info->c[0] = eps * dCalcVectorDot3 ( p, b );
    info->c[1] = eps * dCalcVectorDot3 ( q, b );

    //////////////////////////////////////////////////////////////////////////////////////////
    // Linear velocity constraint (1 row, removes 1 translational DOF along the plane normal)
    //
    // Only body1 should be restricted by the plane, which is moved along with body2.
    //////////////////////////////////////////////////////////////////////////////////////////

    dVector3 body1Center;
    dCopyVector3(body1Center, node[0].body->posr.pos);

    dVector3 body2Center = {0, 0, 0};
    if (node[1].body) {
        dCopyVector3(body2Center, node[1].body->posr.pos);
    }

    // LHS 2
    dCopyNegatedVector3(info->J1l + row2Index, globalAxis2);
    dCopyVector3(       info->J2l + row2Index, globalAxis2);

    dVector3 body2_body1;
    dSubtractVectors3(body2_body1, body2Center, body1Center);
    dCalcVectorCross3(info->J2a + row2Index, globalAxis2, body2_body1);

    // RHS 2 (distance of body1 center from the plane)
    info->c[2] = eps * (dCalcVectorDot3(globalAxis2, body1Center) - dCalcVectorDot3(globalAxis2, globalAnchor));
}


// HACK since we cannot effectively create a new joint type, we misuse the Plane2D joint type.
dJointType dxPlanarJoint::type() const
{
    return dJointTypePlane2D;
}


size_t dxPlanarJoint::size() const
{
    return sizeof( *this );
}

void dJointPlanarSetAnchor(dJointID joint, dVector3 anchor) {
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, Plane2D );
    dxPlanarJoint* planarJoint = (dxPlanarJoint*) joint;

    dCopyVector3(planarJoint->anchor, anchor);

    planarJoint->updatePlane();
}

void dJointPlanarSetPlaneNormal(dJointID joint, dVector3 planeNormal) {
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, Plane2D );
    dxPlanarJoint* planarJoint = (dxPlanarJoint*) joint;

    dUASSERT(dLENGTHSQUARED(planeNormal) > 0.000001, "plane normal cannot have zero length");

    dCopyVector3(planarJoint->planeNormal, planeNormal);
    dNormalize3(planarJoint->planeNormal);

    planarJoint->updatePlane();
}


void dxPlanarJoint::updatePlane() {

    dBodyID body1 = this->node[0].body;
    if (body1)
    {
        // compute plane normal and anchor coordinates in the local coordinates of the first body
        dBodyVectorFromWorld(body1, planeNormal[0], planeNormal[1], planeNormal[2], axis1);
        dBodyGetPosRelPoint(body1, anchor[0], anchor[1], anchor[2], anchor1);

        dBodyID body2 = this->node[1].body;
        if (body2) // second body given, attach the plane to it
        {
            // compute plane normal and anchor coordinates in the local coordinates of the second body
            dBodyVectorFromWorld(body2, planeNormal[0], planeNormal[1], planeNormal[2], axis2);
            dBodyGetPosRelPoint(body2, anchor[0], anchor[1], anchor[2], anchor2);
        }
        else // second body not given, attach the plane to the world
        {
            // plane normal and anchor coordinates in the world world frame are the same as global coordinates
            dCopyVector3(axis2, planeNormal);
            dCopyVector3(anchor2, anchor);
        }
    }
}
