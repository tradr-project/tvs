//
//  heightfield.c
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "heightfield.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <drawstuff/drawstuff.h>

Heightfield * heightfield_create(dWorldID world, dSpaceID space, dReal width, dReal depth, int wstep, int dstep, dReal scale) {
    Heightfield *h = (Heightfield *)malloc(sizeof(Heightfield));
    h->width = width;
    h->depth = depth;
    h->wstep = wstep;
    h->dstep = dstep;
    h->scale = scale;
    h->wsamp = h->width / (h->wstep - 1);
    h->dsamp = h->depth / (h->dstep - 1);
    h->data = dGeomHeightfieldDataCreate();
    h->bWrap = 1;
    dGeomHeightfieldDataBuildCallback(h->data, h, &heightfield_get_callback,
        h->width, h->depth, h->wstep, h->dstep, 1.0, 0.0, 0.0, h->bWrap);
    h->geom = dCreateHeightfield(space, h->data, 1);
    dMatrix3 R;
    dRFromAxisAndAngle(R, 1, 0, 0, M_PI_2);
    dGeomSetRotation(h->geom, R);
    dGeomSetCategoryBits(h->geom, 0x4);
    dGeomSetCollideBits(h->geom, 0x2);
    return h;
}

void heightfield_destroy(Heightfield *h) {
    dGeomHeightfieldDataDestroy(h->data);
    free(h);
}

dReal heightfield_get(Heightfield *h, int x, int y) {
    dReal fx = (((dReal)x) - (h->wstep - 1) / 2) / (dReal)(h->wstep - 1);
    dReal fy = (((dReal)y) - (h->dstep - 1) / 2) / (dReal)(h->dstep - 1);
    //return h->scale * (1.0 - 16.0 * (pow(fx, 3) + pow(fy, 3)));
    return h->scale * (1 + sin(2 * fx * M_PI) * cos(2 * fy * M_PI));
}

dReal heightfield_get_callback(void *h, int x, int y) {
    return heightfield_get((Heightfield *)h, x, y);
}

void heightfield_draw_one(Heightfield *h, int xOffset, int yOffset) {
    dsSetColorAlpha(0.5, 0.9, 0.5, 1.0);

    int ox = (int)(-h->width/2) + xOffset;
    int oz = (int)(-h->depth/2) + yOffset;

    int i, j, k = 0;

    int n = h->wstep * h->dstep * 2;
    dReal *v = (dReal *)malloc(sizeof(dReal) * n * 9);

    for(i = 0; i < h->wstep - 1; i++) {
        for(j = 0; j < h->dstep - 1; j++) {
            v[3*(k+0)+0]                = ox + i * h->wsamp;
            v[3*(k+0)+1]                = heightfield_get(h, i, j);
            v[3*(k+0)+2]                = oz + j * h->dsamp;

            v[3*(k+2)+0] = v[3*(k+3)+0] = ox + (i + 1) * h->wsamp;
            v[3*(k+2)+1] = v[3*(k+3)+1] = heightfield_get(h, i + 1, j);
            v[3*(k+2)+2] = v[3*(k+3)+2] = oz + j * h->dsamp;

            v[3*(k+1)+0] = v[3*(k+4)+0] = ox + i * h->wsamp;
            v[3*(k+1)+1] = v[3*(k+4)+1] = heightfield_get(h, i, j + 1);
            v[3*(k+1)+2] = v[3*(k+4)+2] = oz + (j + 1) * h->dsamp;

            v[3*(k+5)+0]                = ox + (i + 1) * h->wsamp;
            v[3*(k+5)+1]                = heightfield_get(h, i + 1, j + 1);
            v[3*(k+5)+2]                = oz + (j + 1) * h->dsamp;

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

void heightfield_draw(Heightfield *h) {
    const int d = 3 * h->bWrap;
    int ox, oy;
    for(ox = -d; ox <= d; ox++) {
        for(oy = -d; oy <= d; oy++) {
            heightfield_draw_one(h, ox * h->wstep / 4, oy * h->dstep / 4);
        }
    }
}

