//
//  OMPLEnvironment.h
//  tvs
//
//  Created by Main User on 18/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef __tvs__OMPLEnvironment__
#define __tvs__OMPLEnvironment__

#include <ompl/extensions/opende/OpenDESimpleSetup.h>
#include <ompl/util/ClassForward.h>
#include <cstdio>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

OMPL_CLASS_FORWARD(OMPLEnvironment);

class OMPLEnvironment : public oc::OpenDEEnvironment {
public:
    OMPLEnvironment();
    virtual ~OMPLEnvironment();
    
    /* Functions needed by planning: */
    virtual unsigned int getControlDimension(void) const;
    virtual void getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const;
    virtual void applyControl(const double *control) const;
    virtual bool isValidCollision(dGeomID g1, dGeomID g2, const dContact& contact) const;
    virtual void setupContact(dGeomID g1, dGeomID g2, dContact &contact) const;
    /**************************************************/
    
    // OMPL does not require this function here; we implement it here
    // for convenience. This function is only OpenDE code to create a
    // simulation environment. At the end of the function, there is a
    // call to setPlanningParameters(), which configures members of
    // the base class needed by planners.
    void createWorld();
    
    // Clear all OpenDE objects
    void destroyWorld();
    
    // Set parameters needed by the base class (such as the bodies
    // that make up to state of the system we are planning for)
    void setPlanningParameters();
    
    // the simulation world
    dWorldID bodyWorld;
    
    // the space for all objects
    dSpaceID space;
    
    // the car mass
    dMass    m;
    
    // the body geom
    dGeomID  boxGeom;
    
    // the body
    dBodyID  boxBody;
};

#endif /* defined(__tvs__OMPLEnvironment__) */
