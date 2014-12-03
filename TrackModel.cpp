//
//  TrackModel.cpp
//  tvs
//
//  Created by Federico Ferri on 01/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "TrackModel.h"
#include "View.h"

/*
 *    ----- p1
 *   /     *---
 *  /     / \  -------   p2
 *  |    /theta       -----*\
 *  |  O    |              \/
 *  \       /
 *   \     /
 *    -----
 */
TrackModel::TrackModel(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserWidth_, dReal grouserHeight_, dReal trackDepth_) : grousers(numGrousers_) {
    density = 10.0;
    radius1 = fmax(radius1_, radius2_);
    radius2 = fmin(radius1_, radius2_);
    distance = fabs(distance_);
    numGrousers = numGrousers_;
    grouserWidth = grouserWidth_;
    grouserHeight = grouserHeight_;
    trackDepth = trackDepth_;
    radiusDiff = radius1 - radius2;
    pDistance = sqrt(pow(distance, 2) - pow(radiusDiff, 2));
    theta = atan2(pDistance, radiusDiff);
    p1x = radius1 * cos(theta);
    p1y = radius1 * sin(theta);
    p2x = distance + radius2 * cos(theta);
    p2y = radius2 * sin(theta);
    arc1Length = radius1 * 2 * (M_PI - theta);
    arc2Length = radius2 * 2 * theta;
    dlimits[0] = arc1Length;
    dlimits[1] = pDistance;
    dlimits[2] = arc2Length;
    dlimits[3] = pDistance;
    totalLength = 0.0;
    for(size_t i = 0; i < sections; i++) totalLength += dlimits[i];
}

dReal TrackModel::scale(dReal v, dReal vmin, dReal vmax) {
    return v * (vmax - vmin) + vmin;
}

size_t TrackModel::section(dReal u, dReal *v) {
    dReal u1 = (u - floor(u)) * totalLength;
    for(size_t i = 0; i < sections; i++) {
        if(u1 < dlimits[i]) {
            *v = u1 / dlimits[i];
            return i;
        } else {
            u1 -= dlimits[i];
        }
    }
    return -1;
}

void TrackModel::getPointOnPath(dReal u, dReal *x_, dReal *y_, dReal *theta_) {
    dReal v;
    switch(section(u, &v)) {
        case 0:
            *theta_ = scale(v, 2 * M_PI - theta, theta);
            *x_ = radius1 * cos(*theta_);
            *y_ = radius1 * sin(*theta_);
            break;
        case 1:
            *theta_ = theta;
            *x_ = scale(v, p1x, p2x);
            *y_ = scale(v, p1y, p2y);
            break;
        case 2:
            *theta_ = scale(v, theta, -theta);
            *x_ = distance + radius2 * cos(*theta_);
            *y_ = radius2 * sin(*theta_);
            break;
        case 3:
            *theta_ = -theta;
            *x_ = scale(v, p2x, p1x);
            *y_ = scale(v, -p2y, -p1y);
            break;
    }
}

void TrackModel::computeGrouserTransform3D(Grouser *g, size_t i) {
    g->i = i;
    g->z = 0;
    getPointOnPath(i / (dReal)numGrousers + u, &g->x, &g->y, &g->theta);
    dRFromAxisAndAngle(g->R, 0, 0, 1, g->theta);
}

void TrackModel::createAllGrousers(dWorldID world, dSpaceID space) {
    // mass of the composite object:
    dMassSetZero(&mass);
    
    // size of the boxes:
    dReal sides[] = {grouserHeight, grouserWidth, trackDepth};
    
    for(size_t i = 0; i < numGrousers; i++) {
        Grouser& g = grousers[i];
        computeGrouserTransform3D(&grousers[i], i);
        
        // sub object will be added to this transform:
        g.gTrans = dCreateGeomTransform(space);
        dGeomTransformSetCleanup(g.gTrans, 1);
        
        g.gBox = dCreateBox(0, sides[0], sides[1], sides[2]);
        
        dMassSetBox(&g.mass, density, sides[0], sides[1], sides[2]);
        
        dGeomTransformSetGeom(g.gTrans, g.gBox);
        
        dGeomSetPosition(g.gBox, g.x, g.y, g.z);
        dMassTranslate(&g.mass, g.x, g.y, g.z);
        
        dGeomSetRotation(g.gBox, g.R);
        dMassRotate(&g.mass, g.R);
        
        dMassAdd(&mass, &g.mass);
    }
    
    // translate so that center of mass is at origin and ode won't complain
    for(int i = 0; i < numGrousers; i++) {
        Grouser& g = grousers[i];
        g.ox = -mass.c[0];
        g.oy = -mass.c[1];
        g.oz = -mass.c[2];
        dGeomSetPosition(g.gBox, g.x + g.ox, g.y + g.oy, g.z + g.oz);
    }
    dMassTranslate(&mass, -mass.c[0], -mass.c[1], -mass.c[2]);
    
    body = dBodyCreate(world);
    
    for(int i = 0; i < numGrousers; i++) {
        Grouser& g = grousers[i];
        dGeomSetBody(g.gTrans, body);
    }
    
    dBodySetMass(body, &mass);
    dBodySetPosition(body, 0.0, 0.4, 1.0);
}

void TrackModel::create(dWorldID world, dSpaceID space) {
    createAllGrousers(world, space);
}

void TrackModel::draw(View *view) {
    const dReal *pos = dBodyGetPosition(body);
    const dReal *R = dBodyGetRotation(body);
    
    view->drawAxes(pos, R);
    
    for(std::vector<Grouser>::iterator it = grousers.begin(); it != grousers.end(); it++) {
        const dReal *posL = dGeomGetPosition(it->gBox);
        const dReal *RL = dGeomGetRotation(it->gBox);
        dVector3 posW;
        dMatrix3 RW;
        dMULTIPLY0_331(posW, R, posL);
        posW[0] += pos[0];
        posW[1] += pos[1];
        posW[2] += pos[2];
        dMULTIPLY0_333(RW, R, RL);
        
        dReal sides[3];
        dGeomBoxGetLengths(it->gBox, sides);
        view->drawBox(sides, posW, RW);
    }
}

void TrackModel::rotate(dReal du) {
    u += du;
    
    for(size_t i = 0; i < numGrousers; i++) {
        Grouser& g = grousers[i];
        
        // get position of object on curve:
        computeGrouserTransform3D(&grousers[i], i);

        dGeomSetPosition(g.gBox, g.x + g.ox, g.y + g.oy, g.z + g.oz);
        dGeomSetRotation(g.gBox, g.R);        
    }
}

#include "Physics.h"
extern Physics physics;

void TrackModel::test() {
    //dBodyAddTorque(body, 0.0, 0.1, 0.0);
    //dBodyAddForce(body, 0, 0.01 * (1+sin(physics.stepNum * 0.01)), 0);
}
