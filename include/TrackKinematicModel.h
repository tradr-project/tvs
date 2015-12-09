//
//  TrackKinematicModel.h
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef TRACK_KINEMATIC_MODEL_H_INCLUDED
#define TRACK_KINEMATIC_MODEL_H_INCLUDED

#include <ode/ode.h>

class TrackKinematicModel {
public:
    dReal radius[2];
    dReal distance;
    size_t numGrousers;
    dReal grouserWidth;
    dReal grouserHeight;
    dReal trackDepth;
    dReal radiusDiff;
    dReal pDistance;
    dReal theta;
    dReal px[2];
    dReal py[2];
    dReal arc1Length;
    dReal arc2Length;
    dReal totalLength;
    dReal dlimits[4];

    TrackKinematicModel(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserHeight_, dReal trackDepth_);
    virtual ~TrackKinematicModel();
    void getPointOnPath(dReal u, dReal *x_, dReal *y_, dReal *theta_);
    void computeGrouserTransform3D(size_t i, dReal *pos, dReal *R);
};

#endif // TRACK_KINEMATIC_MODEL_H_INCLUDED
