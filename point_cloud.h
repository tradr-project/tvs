//
//  pointcloud.h
//  tvs
//
//  Created by Federico Ferri on 11/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include <ode/ode.h>

typedef struct {
    dReal *data;
    size_t size;
    dReal point_radius;
    dGeomID *geom;
} PointCloud;

PointCloud * point_cloud_read(const char *filename);

void point_cloud_destroy(PointCloud *p);

PointCloud * point_cloud_filter_far(PointCloud *p, const dReal *center, dReal distance);

void point_cloud_create_geom(PointCloud *p, dWorldID world, dSpaceID space);

void point_cloud_draw(PointCloud *p);

