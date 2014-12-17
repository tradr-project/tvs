//
//  Heightfield.h
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef HEIGHTFIELD_H_INCLUDED
#define HEIGHTFIELD_H_INCLUDED

#include <ode/ode.h>

class World;

class Heightfield {
public:
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
    
    Heightfield(dReal width, dReal depth, int wstep, int dstep, dReal scale);
    virtual ~Heightfield();
    void create(World *world);
    void destroy();
    dReal get(int x, int y);
    static dReal getCallback(void *h, int x, int y);
    void drawOne(int xOffset, int yOffset);
    void draw();
};

#endif // HEIGHTFIELD_H_INCLUDED
