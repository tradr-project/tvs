//
//  Physics.h
//  tvs
//
//  Created by Federico Ferri on 01/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef __tvs__Physics__
#define __tvs__Physics__

#include "TrackModel.h"

#include <ode/ode.h>

#define MAX_CONTACTS 10

class Physics {
private:
    dWorldID world;
    dSpaceID space;
    dJointGroupID contactGroup;
    
public:
    size_t stepNum;

    TrackModel track;
    dGeomID planeGeom;
    
    std::vector<dContactGeom> contactsCache;
public:
    Physics();
    void init();
    void destroy();
    static void nearCallbackWrapper(void *data, dGeomID o1, dGeomID o2);
    void nearCallback(dGeomID o1, dGeomID o2);
    void step();
};

#endif /* defined(__tvs__Physics__) */
