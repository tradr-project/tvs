#ifndef _ODE_PLANAR_JOINT_H_
#define _ODE_PLANAR_JOINT_H_

// HACK start
// There is no standard way to develop new joints without actually forking the whole ODE.
// As this seems like too much overhead to me, instead I just place ODE sources to ../ode, and use the non-public API
// needed for developing new joints.

// For whatever reason, the uint8 type needs to be declared manually.
#include <stdint.h>
typedef uint8_t uint8;

#include "joints/joint.h" // this has to be directed to ode sources directory!
// HACK end


/**
 * @brief A joint that restricts movement of a body to a given (arbitrary) 2D plane.
 *
 * The restricting plane may be either static (connected to the world) or may move along with another body.
 */
struct dxPlanarJoint : public dxJoint
{
public:
    /** The anchor point in global coordinates at the time it has been set. */
    dVector3 anchor;
    /** The plane normal in global coordinates at the time it has been set. */
    dVector3 planeNormal;

    /**
     * Create the planar joint with default restriction to the @c z=0 plane.
     *
     * @param world The world to create the joint in.
     */
    dxPlanarJoint( dxWorld *world);

    ///////////////////////////////
    // dxJoint API implementation
    ///////////////////////////////

    /** @copydoc dxJoint::getSureMaxInfo() */
    virtual void getSureMaxInfo( SureMaxInfo* info );

    /** @copydoc dxJoint::getInfo1() */
    virtual void getInfo1( Info1* info );

    /** @copydoc dxJoint::getInfo2() */
    virtual void getInfo2( dReal worldFPS, dReal worldERP, const Info2Descr* info );

    /** @copydoc dxJoint::type() */
    virtual dJointType type() const;

    /** @copydoc dxJoint::size() */
    virtual size_t size() const;

    ///////////////////
    // Custom methods
    ///////////////////

    /**
     * @brief Update the body1 and body2 local restricting plane equations based on @c anchor, @c planeNormal and
     *        current poses of the bodies.
     *
     * Called automatically after dJointPlanarSetPlaneNormal() and dJointPlanarSetAnchor().
     */
    virtual void updatePlane();

protected:
    dVector3    axis1, /**< The restricting plane normal in body1 local coordinates. */
                axis2, /**< The restricting plane normal in body2 local coordinates. */
                anchor1, /**< The restricting plane anchor point in body1 local coordinates. */
                anchor2; /**< The restricting plane anchor point in body2 local coordinates. */

};

/**
 * @brief Create a joint that restricts movement of a body to a given (arbitrary) 2D plane (defined by an anchor
 * ("origin") and plane normal).
 *
 * The restricting plane may be either static (connected to the world) or may move along with another body.
 *
 * When attaching, body2 (or world) moves the plane, whereas body1's movement is restricted by the plane.
 *
 * After bodies are attached to the joint, call dJointPlanarSetPlaneNormal() and dJointPlanarSetAnchor() to set the
 * restricting plane. It will remember the relative pose to both bodies at the moment it was called, and this relative
 * pose will be used to constrain body1 motion. By default, body1 is restricted to move along the @c z=0 plane.
 *
 * @ingroup joints
 *
 * @param world The world to create the joint in.
 * @param group Set to 0 to allocate the joint normally.
 *              If it is nonzero the joint is allocated in the given joint group.
 *
 * @return The joint.
 */
ODE_API dJointID dJointCreatePlanar (dWorldID world, dJointGroupID group);

/**
 * @brief Set the anchor point of the plane restricting a planar joint.
 *
 * This is one of the points lying in the pla. Together with the plane normal, it uniquely defines the plane.
 *
 * @ingroup joints
 *
 * @param joint The joint for which to set the anchor.
 * @param anchor The anchor point in global coordinates.
 */
ODE_API void dJointPlanarSetAnchor(dJointID joint, dVector3 anchor);

/**
 * @brief Set the normal of the plane restricting a planar joint.
 *
 * Together with the anchor point, it uniquely defines the plane.
 *
 * @ingroup joints
 *
 * @param joint The joint for which to set the plane normal.
 * @param anchor The plane normal in global coordinates (not necessarily unit-length).
 */
ODE_API void dJointPlanarSetPlaneNormal(dJointID joint, dVector3 planeNormal);

#endif