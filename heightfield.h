//
//  heightfield.h
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef HEIGHTFIELD_H_INCLUDED
#define HEIGHTFIELD_H_INCLUDED

#include <ode/ode.h>

typedef struct {
    dReal width;
    dReal depth;
    int wstep;
    int dstep;
    dReal wsamp;
    dReal dsamp;
    dReal scale;
    dGeomID geom;
    dHeightfieldDataID data;
    int bWrap;
} Heightfield;

Heightfield * heightfield_create(dWorldID world, dSpaceID space, dReal width, dReal depth, int wstep, int dstep, dReal scale);

void heightfield_destroy(Heightfield *h);

dReal heightfield_get(Heightfield *h, int x, int y);

dReal heightfield_get_callback(void *h, int x, int y);

void heightfield_draw_one(Heightfield *h, int xOffset, int yOffset);

void heightfield_draw(Heightfield *h);

#endif // HEIGHTFIELD_H_INCLUDED
