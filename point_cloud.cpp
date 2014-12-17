//
//  pointcloud.cpp
//  tvs
//
//  Created by Federico Ferri on 11/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "point_cloud.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <drawstuff/drawstuff.h>

PointCloud * point_cloud_read(const char *filename) {
    PointCloud *p = (PointCloud *)malloc(sizeof(PointCloud));
    p->point_radius = 0.1;
    char filename2[256];
    snprintf(filename2, 256, "%s/%s", POINTCLOUDS_PATH, filename);
    FILE *f = fopen(filename2, "r");
    int m = 0;
    fscanf(f, "%d", &m);
    p->data = (dReal *)malloc(sizeof(dReal) * m * 3);
    p->size = 0;
    p->geom = NULL;
    dReal *q = p->data;
    while(p->size++ < m && fscanf(f, "%lf %lf %lf", q, q + 1, q + 2) != EOF)
        q += 3;
    fclose(f);
    return p;
}

void point_cloud_destroy(PointCloud *p) {
}

void point_cloud_deinit(PointCloud *p) {
    if(p->geom) free(p->geom);
    free(p->data);
    free(p);
}

static dReal dist(const dReal *a, const dReal *b) {
    dReal sd = 0.0;
    int i;
    for(i = 0; i < 3; i++) sd += pow(a[i] - b[i], 2);
    return sqrt(sd);
}

PointCloud * point_cloud_filter_far(PointCloud *p, const dReal *center, dReal distance) {
    size_t i;
    PointCloud *pnear = (PointCloud *)malloc(sizeof(PointCloud));
    pnear->size = 0;
    // count near points:
    for(i = 0; i < p->size; i++)
        if(dist(&p->data[3 * i], center) < distance)
            pnear->size++;
    // allocate storage for near points:
    pnear->data = (dReal *)malloc(sizeof(dReal) * 3 * pnear->size);
    pnear->size = 0;
    for(i = 0; i < p->size; i++)
        if(dist(&p->data[3 * i], center) < distance)
            memcpy(&pnear->data[3 * pnear->size++], &p->data[3 * i], 3 * sizeof(dReal));
    return pnear;
}

void point_cloud_create_geom(PointCloud *p, dWorldID world, dSpaceID space) {
    size_t i;
    p->geom = (dGeomID *)malloc(sizeof(dGeomID) * p->size);
    for(i = 0; i < p->size; i++) {
        p->geom[i] = dCreateSphere(space, p->point_radius);
        dGeomSetPosition(p->geom[i], p->data[i * 3], p->data[i * 3 + 1], p->data[i * 3 + 2]);
        dMatrix3 R; dRSetIdentity(R);
        dGeomSetRotation(p->geom[i], R);
        dGeomSetCategoryBits(p->geom[i], 0x4);
        dGeomSetCollideBits(p->geom[i], 0x2);
    }
}

void point_cloud_draw(PointCloud *p) {
    dsDrawPCLD(p->data, p->size, p->point_radius);
}

