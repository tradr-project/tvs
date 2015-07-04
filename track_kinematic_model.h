//
//  track_kinematic_model.h
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef TRACK_KINEMATIC_MODEL_H_INCLUDED
#define TRACK_KINEMATIC_MODEL_H_INCLUDED

#include <ode/ode.h>

typedef struct {
    dReal radius1;
    dReal radius2;
    dReal distance;
    size_t numGrousers;
    dReal grouserWidth;
    dReal grouserHeight;
    dReal trackDepth;
    dReal flipDepth;
    dReal radiusDiff;
    dReal pDistance;
    dReal theta;
    dReal p1x;
    dReal p1y;
    dReal p2x;
    dReal p2y;
    dReal arc1Length;
    dReal arc2Length;
    dReal totalLength;
    dReal dlimits[4];
} TrackKinematicModel;

TrackKinematicModel * track_kinematic_model_init(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserHeight_, dReal trackDepth_, dReal flipDepth_);

void track_kinematic_model_deinit(TrackKinematicModel *m);

void track_kinematic_model_get_point_on_path(TrackKinematicModel *m, dReal u, dReal *x_, dReal *y_, dReal *theta_);

void track_kinematic_model_compute_grouser_transform_3D(TrackKinematicModel *m, size_t i, dReal *pos, dReal *R);

#endif // TRACK_KINEMATIC_MODEL_H_INCLUDED
