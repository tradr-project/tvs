//
//  PointCloud.h
//  tvs
//
//  Created by Federico Ferri on 11/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef POINT_CLOUD_H_INCLUDED
#define POINT_CLOUD_H_INCLUDED

#include <ode/ode.h>

class Environment;

class PointCloud {
public:
    dReal *data;
    size_t size;
    dReal point_radius;
    dGeomID *geom;
    
    PointCloud();
    PointCloud(const char *filename);
    virtual ~PointCloud();
    void create(Environment *environment);
    void destroy();
    void draw();
    void filterFar(const dReal *center, dReal distance);
};

#endif // POINT_CLOUD_H_INCLUDED
