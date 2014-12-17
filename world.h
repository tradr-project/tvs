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

typedef struct {
    dWorldID world;
    dSpaceID space;
    dJointGroupID contactGroup;

    dGeomID planeGeom;

    TrackedVehicle *v;
    PointCloud *pcl;
} World;

World * world_init();

void world_create(World *w);

void world_step(World *w, dReal stepSize, int simulationStepsPerFrame);

void world_destroy(World *w);

void world_deinit(World *w);

void world_draw(World *w);

#endif // WORLD_H_INCLUDED

