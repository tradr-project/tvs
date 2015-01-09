//
//  TriMesh.cpp
//  tvs
//
//  Created by Federico Ferri on 8/1/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Environment.h"
#include "TriMesh.h"
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <vector>
#include <fstream>
#include <drawstuff/drawstuff.h>

TriMesh::TriMesh() {
}

TriMesh::~TriMesh() {
}

void TriMesh::create(Environment *environment, const char *inputfile) {
    std::ifstream f(inputfile, std::ios::in | std::ios::binary);
    char header[80] = "";
    char buf4[4];
    unsigned long ntri;

    if(!f) {
        std::cout << "STL read error" << std::endl;
        exit(1);
    }

    f.read(header, 80);
    header[79] = '\0';

    if(!f) {
        std::cout << "STL read error (1)" << std::endl;
        exit(1);
    }

    f.read(buf4, 4);
    ntri = *((unsigned int *)buf4);

    if(!f) {
        std::cout << "STL read error (2)" << std::endl;
        exit(1);
    }

    this->vertex_count = 3 * ntri;
    this->triangle_count = ntri;
    this->vertices = new Vertex[this->vertex_count * 4];
    this->triangles = new Triangle[this->triangle_count];
    size_t h = 0, k = 0;
    for(size_t i = 0; i < ntri; i++) {
        char buf50[50];
        f.read(buf50, 50);
        if(!f) {
            std::cout << "STL read error (3:" << i << ")" << std::endl;
            exit(1);
        }

        unsigned short attr = *((unsigned short *)(buf50 + 48));
        this->vertices[h].v[0] = ((float *)buf50)[3];
        this->vertices[h].v[1] = ((float *)buf50)[4];
        this->vertices[h].v[2] = ((float *)buf50)[5];
        this->triangles[k].i[2] = h++;
        this->vertices[h].v[0] = ((float *)buf50)[6];
        this->vertices[h].v[1] = ((float *)buf50)[7];
        this->vertices[h].v[2] = ((float *)buf50)[8];
        this->triangles[k].i[1] = h++;
        this->vertices[h].v[0] = ((float *)buf50)[9];
        this->vertices[h].v[1] = ((float *)buf50)[10];
        this->vertices[h].v[2] = ((float *)buf50)[11];
        this->triangles[k].i[0] = h++;
        k++;
    }

    this->data = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildSingle(this->data, this->vertices, sizeof(Vertex), this->vertex_count, this->triangles, this->triangle_count, sizeof(Triangle));
    this->geom = dCreateTriMesh(environment->space, this->data, 0, 0, 0);
    dGeomSetCategoryBits(this->geom, Category::TERRAIN);
    dGeomSetCollideBits(this->geom, Category::GROUSER | Category::OBSTACLE);
}

void TriMesh::destroy() {
    dGeomTriMeshDataDestroy(this->data);
}

void TriMesh::draw() {
    const dReal* pos = dGeomGetPosition(this->geom);
    const dReal* R = dGeomGetRotation(this->geom);
#ifdef HAVE_DSDRAWTRIANGLES_FUNCTION
    dsDrawTrianglesD(pos, R, v, n, 1);
#else
    for(size_t i = 0; i < this->triangle_count; i++) {
        const dReal *p1 = this->vertices[this->triangles[i].i[2]].v;
        const dReal *p2 = this->vertices[this->triangles[i].i[1]].v;
        const dReal *p3 = this->vertices[this->triangles[i].i[0]].v;
        dsDrawTriangleD(pos, R, p1, p2, p3, 1);
    }
#endif // HAVE_DSDRAWTRIANGLES_FUNCTION
}

