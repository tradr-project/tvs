//
//  Environment.h
//  tvs
//
//  Created by Federico Ferri on 17/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef WORLD_H_INCLUDED
#define WORLD_H_INCLUDED

#include <ode/ode.h>
#include <string>
#include <map>
#include <boost/lexical_cast.hpp>
#include "TrackedVehicle.h"
#include "PointCloud.h"

class Environment {
public:
    dWorldID world;
    dSpaceID space;
    dJointGroupID contactGroup;

    dGeomID planeGeom;

    TrackedVehicle *v;
    PointCloud *pcl;
    
    bool badCollision;
    
    std::map<dGeomID, std::string> geomNames;
    
    Environment();
    virtual ~Environment();
    void create();
    void destroy();
    std::string getGeomName(dGeomID geom) const;
    void setGeomName(dGeomID geom, const std::string &name);
    int getMaxContacts(dGeomID o1, dGeomID o2);
    bool isCatPair(unsigned long cat1, unsigned long cat2, dGeomID *o1, dGeomID *o2);
    bool isValidCollision(dGeomID o1, dGeomID o2, const dContact& contact);
    void nearCallback(dGeomID o1, dGeomID o2);
    void nearCallbackWheelGrouser(dGeomID o1, dGeomID o2);
    void nearCallbackGrouserTerrain(dGeomID o1, dGeomID o2);
    void nearCallbackDefault(dGeomID o1, dGeomID o2);
    bool step(dReal stepSize, int simulationStepsPerFrame);
    void evaluateCollisionNearCallback(dGeomID o1, dGeomID o2);
    bool evaluateCollision();
    void draw();
};

enum Category {
    TERRAIN  = 1 << 0,
    GROUSER  = 1 << 1,
    WHEEL    = 1 << 2,
    OBSTACLE = 1 << 3
};

#endif // WORLD_H_INCLUDED
