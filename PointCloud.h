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
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

class Environment;

struct Points {
    dReal *data;
    size_t size;
};

class PointCloud {
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_flat;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree;
    dReal point_radius;
    dGeomID *geom;
    
    Points all;
    Points near;
    
    PointCloud(const char *filename, dReal downsampling_radius);
    virtual ~PointCloud();
    void create(Environment *environment, const dReal *pos, dReal radius);
    void destroy();
    void draw();
};

#endif // POINT_CLOUD_H_INCLUDED
