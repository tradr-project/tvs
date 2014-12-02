//
//  TrackModel.h
//  tvs
//
//  Created by Federico Ferri on 01/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef __tvs__TrackModel__
#define __tvs__TrackModel__

#include <vector>

#include <ode/ode.h>

class View;

struct Grouser {
    size_t i;
    dGeomID gTrans, gBox;
    dBodyID body;
    dReal x, y, z, theta, ox, oy, oz;
    dMass mass;
    dMatrix3 R;
};

class TrackModel {
private:
    dReal radius1;
    dReal radius2;
    dReal distance;
    dReal pDistance;
    dReal radiusDiff;
    dReal theta;
    dReal p1x, p1y, p2x, p2y;
    dReal arc1Length;
    dReal arc2Length;
    dReal totalLength;
    static const size_t sections = 4;
    dReal dlimits[sections];
    size_t numGrousers;
    dReal grouserWidth;
    dReal grouserHeight;
    dReal trackDepth;
    std::vector<Grouser> grousers;
    dBodyID body;
    dMass mass;
    dReal density;
    dReal u;
public:
    TrackModel(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserWidth_, dReal grouserHeight_, dReal trackDepth_);
    dReal scale(dReal v, dReal vmin, dReal vmax);
    size_t section(dReal u, dReal *v);
    void getPointOnPath(dReal u, dReal *x_, dReal *y_, dReal *theta_);
    void computeGrouserTransform3D(Grouser *g, size_t i);
    void createAllGrousers(dWorldID world, dSpaceID space);
    void create(dWorldID world, dSpaceID space);
    void draw(View *view);
    void rotate(dReal du);
    void test();
};

#endif /* defined(__tvs__TrackModel__) */
