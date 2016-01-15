//
//  PointCloud.cpp
//  tvs
//
//  Created by Federico Ferri on 11/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Environment.h"
#include "PointCloud.h"
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <drawstuff/drawstuff.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

PointCloud::PointCloud(const char *filename, dReal downsampling_radius)
:
    cloud(new pcl::PointCloud<pcl::PointXYZ>()),
    kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>())
{
    this->point_radius = downsampling_radius;
    
    char filename2[256];
    snprintf(filename2, 256, "%s/%s", POINTCLOUDS_PATH, filename);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(filename2, *cloud) == -1) {
        std::cerr << "PointCloud: load error: " << filename << std::endl;
        exit(1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> f;
    f.setInputCloud(cloud);
    f.setLeafSize(this->point_radius, this->point_radius, this->point_radius);
    f.filter(*cloud_ds);
    cloud.swap(cloud_ds);

    //cloud_flat = cloud->makeShared();
    //for(size_t i = 0; i < cloud_flat->points.size(); i++) {
    //    cloud_flat->points[i].z /= 10000.0;
    //}
    
    kdtree->setInputCloud(cloud);

    this->all.size = this->cloud->points.size();
    this->all.data = new dReal[this->all.size * 3];
    size_t h = 0;
    for(size_t i = 0; i < this->cloud->points.size(); i++) {
        this->all.data[h++] = this->cloud->points[i].x;
        this->all.data[h++] = this->cloud->points[i].y;
        this->all.data[h++] = this->cloud->points[i].z;
    }

    this->geom = 0L;
    this->geom = new dGeomID[this->cloud->points.size()];
    for(size_t i = 0; i < this->cloud->points.size(); i++) {
        this->geom[i] = 0L;
    }
}

PointCloud::~PointCloud() {
    if(this->geom) delete [] this->geom;
    if(this->all.data) delete [] this->all.data;
    if(this->near.data) delete [] this->near.data;
}

void PointCloud::create(Environment *environment, const dReal *pos, dReal radius) {
    std::vector<int> nearIndices;
    std::vector<float> distances;

    pcl::PointXYZ p;
    p.x = pos[0];
    p.y = pos[1];
    p.z = pos[2];

    this->kdtree->radiusSearch(p, radius, nearIndices, distances);
    bool near1[this->cloud->points.size()];
    this->near.size = nearIndices.size();
    if(this->near.data) delete [] this->near.data;
    this->near.data = new dReal[3 * this->near.size];
    for(size_t i = 0; i < this->cloud->points.size(); i++)
        near1[i] = false;
    for(size_t j = 0; j < nearIndices.size(); j++) {
        size_t i = nearIndices[j];
        near1[i] = true;
        this->near.data[3 * j + 0] = this->cloud->points[i].x;
        this->near.data[3 * j + 1] = this->cloud->points[i].y;
        this->near.data[3 * j + 2] = this->cloud->points[i].z;
    }

    for(size_t i = 0; i < this->cloud->points.size(); i++) {
        if(this->geom[i] && !near1[i]) {
            dGeomDestroy(this->geom[i]);
            this->geom[i] = 0L;
            continue;
        }
        if(this->geom[i] && near1[i]) {
            continue;
        }
        if(!this->geom[i] && !near1[i]) {
            continue;
        }
        this->geom[i] = dCreateSphere(environment->space, this->point_radius);
        dGeomSetPosition(this->geom[i], this->cloud->points[i].x, this->cloud->points[i].y, this->cloud->points[i].z);
        dMatrix3 R; dRSetIdentity(R);
        dGeomSetRotation(this->geom[i], R);
        dGeomSetCategoryBits(this->geom[i], Category::TERRAIN);
        dGeomSetCollideBits(this->geom[i], Category::TRACK_GROUSER | Category::OBSTACLE);
    }
}

void PointCloud::destroy() {
    if(this->geom) {
        for(size_t i = 0; i < this->cloud->points.size(); i++) {
            if(this->geom[i])
                dGeomDestroy(this->geom[i]);
        }
    }
}

void PointCloud::draw() {
    dsSetColor(0,1,0);
    dsDrawPCLD(this->near.data, this->near.size, this->point_radius);
    dsSetColorAlpha(1,1,1,0.3);
    dsDrawPCLD(this->all.data, this->all.size, this->point_radius);
}

