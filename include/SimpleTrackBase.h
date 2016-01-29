#ifndef TRACKED_MOTION_SIMPLE_TRACKBASE_H
#define TRACKED_MOTION_SIMPLE_TRACKBASE_H


#include <string>
#include <ode/ode.h>
#include "utils.h"
#include "ODEUtils.h"
#include "Environment.h"

class SimpleTrackBase {
public:
    dRigidBodyArrayID bodyArray;

    SimpleTrackBase(const std::string &name_, dReal rearRadius, dReal frontRadius,
                        dReal betweenWheelsDistance, dReal trackDepth,
                        unsigned long additionalCategory);
    virtual ~SimpleTrackBase();
    virtual void create(Environment *environment);
    virtual void destroy();
    virtual void draw();
    virtual void step(dReal stepSize) = 0;
    virtual void setVelocity(dReal velocity) = 0;
    virtual dReal getVelocity() = 0;

    dReal betweenWheelsDistance;
    dBodyID trackBody;
    dReal rearRadius;
    dReal frontRadius;
    dReal trackDepth;
protected:
    virtual unsigned long getGrouserCategory() = 0;

    std::string name;
    dReal density;
    dGeomID trackGeom[2];
    dGeomID wheelGeom[2];
    dMass trackMass;
    unsigned long additionalCategory;
};


#endif //TRACKED_MOTION_SIMPLE_TRACKBASE_H
