//
//  Heightfield.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Environment.h"
#include "Heightfield.h"
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <drawstuff/drawstuff.h>

Heightfield::Heightfield(dReal width, dReal depth, int wstep, int dstep, dReal scale) {
    this->width = width;
    this->depth = depth;
    this->wstep = wstep;
    this->dstep = dstep;
    this->scale = scale;
    this->wsamp = this->width / (this->wstep - 1);
    this->dsamp = this->depth / (this->dstep - 1);
    this->data = dGeomHeightfieldDataCreate();
    this->bWrap = 1;
}

Heightfield::~Heightfield() {
}

void Heightfield::create(Environment *environment) {
    dGeomHeightfieldDataBuildCallback(this->data, this, &Heightfield::getCallback,
        this->width, this->depth, this->wstep, this->dstep, 1.0, 0.0, 0.0, this->bWrap);
    this->geom = dCreateHeightfield(environment->space, this->data, 1);
    dMatrix3 R;
    dRFromAxisAndAngle(R, 1, 0, 0, M_PI_2);
    dGeomSetRotation(this->geom, R);
    dGeomSetCategoryBits(this->geom, Category::TERRAIN);
    dGeomSetCollideBits(this->geom, Category::GROUSER | Category::OBSTACLE);
}

void Heightfield::destroy() {
    dGeomHeightfieldDataDestroy(this->data);
}

dReal Heightfield::get(int x, int y) {
    dReal fx = (((dReal)x) - (this->wstep - 1) / 2) / (dReal)(this->wstep - 1);
    dReal fy = (((dReal)y) - (this->dstep - 1) / 2) / (dReal)(this->dstep - 1);
    return this->scale * (1 + sin(2 * fx * M_PI) * cos(2 * fy * M_PI));
}

dReal Heightfield::getCallback(void *h, int x, int y) {
    return reinterpret_cast<Heightfield *>(h)->get(x, y);
}

void Heightfield::drawOne(int xOffset, int yOffset) {
    dsSetColorAlpha(0.5, 0.9, 0.5, 1.0);

    int ox = (int)(-this->width/2) + xOffset;
    int oz = (int)(-this->depth/2) + yOffset;

    int i, j, k = 0;

    int n = this->wstep * this->dstep * 2;
    dReal *v = (dReal *)malloc(sizeof(dReal) * n * 9);

    for(i = 0; i < this->wstep - 1; i++) {
        for(j = 0; j < this->dstep - 1; j++) {
            v[3*(k+0)+0]                = ox + i * this->wsamp;
            v[3*(k+0)+1]                = this->get(i, j);
            v[3*(k+0)+2]                = oz + j * this->dsamp;

            v[3*(k+2)+0] = v[3*(k+3)+0] = ox + (i + 1) * this->wsamp;
            v[3*(k+2)+1] = v[3*(k+3)+1] = this->get(i + 1, j);
            v[3*(k+2)+2] = v[3*(k+3)+2] = oz + j * this->dsamp;

            v[3*(k+1)+0] = v[3*(k+4)+0] = ox + i * this->wsamp;
            v[3*(k+1)+1] = v[3*(k+4)+1] = this->get(i, j + 1);
            v[3*(k+1)+2] = v[3*(k+4)+2] = oz + (j + 1) * this->dsamp;

            v[3*(k+5)+0]                = ox + (i + 1) * this->wsamp;
            v[3*(k+5)+1]                = this->get(i + 1, j + 1);
            v[3*(k+5)+2]                = oz + (j + 1) * this->dsamp;

            k += 6;
        }
    }

#ifdef HAVE_DSDRAWTRIANGLES_FUNCTION
    const dReal* pos = dGeomGetPosition(h->geom);
    const dReal* R = dGeomGetRotation(h->geom);
    dsDrawTrianglesD(pos, R, v, n, 1);
#endif // HAVE_DSDRAWTRIANGLES_FUNCTION

    free(v);
}

void Heightfield::draw() {
    const int d = 3 * this->bWrap;
    int ox, oy;
    for(ox = -d; ox <= d; ox++) {
        for(oy = -d; oy <= d; oy++) {
            this->drawOne(ox * this->wstep / 4, oy * this->dstep / 4);
        }
    }
}

