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

PointCloud::PointCloud() {
}

PointCloud::PointCloud(const char *filename) {
    this->point_radius = 0.1;
    
    char filename2[256];
    snprintf(filename2, 256, "%s/%s", POINTCLOUDS_PATH, filename);
    
    FILE *f = fopen(filename2, "r");
    int m = 0;
    fscanf(f, "%d", &m);
    this->data = new dReal[m * 3];
    this->size = 0;
    this->geom = 0L;
    dReal *q = this->data;
    while(this->size++ < m && fscanf(f, "%lf %lf %lf", q, q + 1, q + 2) != EOF)
        q += 3;
    fclose(f);
}

PointCloud::~PointCloud() {
    if(this->geom) delete [] this->geom;
    if(this->data) delete [] this->data;
}

void PointCloud::create(Environment *environment) {
    this->geom = new dGeomID[this->size];
    for(size_t i = 0; i < this->size; i++) {
        this->geom[i] = dCreateSphere(environment->space, this->point_radius);
        dGeomSetPosition(this->geom[i], this->data[i * 3], this->data[i * 3 + 1], this->data[i * 3 + 2]);
        dMatrix3 R; dRSetIdentity(R);
        dGeomSetRotation(this->geom[i], R);
        dGeomSetCategoryBits(this->geom[i], 0x4);
        dGeomSetCollideBits(this->geom[i], 0x2);
    }
}

void PointCloud::destroy() {
}

void PointCloud::draw() {
    dsDrawPCLD(this->data, this->size, this->point_radius);
}

static dReal dist(const dReal *a, const dReal *b) {
    dReal sd = 0.0;
    int i;
    for(i = 0; i < 3; i++) sd += pow(a[i] - b[i], 2);
    return sqrt(sd);
}

void PointCloud::filterFar(const dReal *center, dReal distance) {
    size_t newSize = 0;
    // count near points:
    for(size_t i = 0; i < this->size; i++)
        if(dist(&this->data[3 * i], center) < distance)
            newSize++;
    // allocate storage for near points:
    dReal *newData = new dReal[newSize * 3];
    size_t j = 0;
    for(size_t i = 0; i < this->size; i++)
        if(dist(&this->data[3 * i], center) < distance)
            memcpy(&newData[3 * j++], &this->data[3 * i], 3 * sizeof(dReal));
    // replace data:
    if(this->geom) delete [] this->geom;
    if(this->data) delete [] this->data;
    this->data = newData;
    this->size = newSize;
}


