//
//  world.h
//  tvs
//
//  Created by Federico Ferri on 17/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef WORLD_H_INCLUDED
#define WORLD_H_INCLUDED

#include <ode/ode.h>
#include "tracked_vehicle.h"
#include "point_cloud.h"

class World {
public:
    dWorldID world;
    dSpaceID space;
    dJointGroupID contactGroup;

    dGeomID planeGeom;

    TrackedVehicle *v;
    PointCloud *pcl;
    
    World();
    virtual ~World();
    void create();
    void destroy();
    void nearCallback(dGeomID o1, dGeomID o2);
    void step(dReal stepSize, int simulationStepsPerFrame);
    void draw();
};

#endif // WORLD_H_INCLUDED
