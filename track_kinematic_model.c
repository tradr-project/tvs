//
//  track_kinematic_model.c
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "track_kinematic_model.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <drawstuff/drawstuff.h>

TrackKinematicModel * track_kinematic_model_create(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserHeight_, dReal trackDepth_) {
    size_t i;
    TrackKinematicModel *m = (TrackKinematicModel *)malloc(sizeof(TrackKinematicModel));
    m->radius1 = radius1_;
    m->radius2 = radius2_;
    m->distance = distance_;
    m->numGrousers = numGrousers_;
    m->grouserHeight = grouserHeight_;
    m->trackDepth = trackDepth_;
    m->radiusDiff = m->radius1 - m->radius2;
    m->pDistance = sqrt(pow(m->distance, 2) - pow(m->radiusDiff, 2));
    m->theta = atan2(m->pDistance, m->radiusDiff);
    m->p1x = m->radius1 * cos(m->theta);
    m->p1y = m->radius1 * sin(m->theta);
    m->p2x = m->distance + m->radius2 * cos(m->theta);
    m->p2y = m->radius2 * sin(m->theta);
    m->arc1Length = m->radius1 * 2 * (M_PI - m->theta);
    m->arc2Length = m->radius2 * 2 * m->theta;
    m->totalLength = 0.0;
    m->dlimits[0] = m->arc1Length;
    m->dlimits[1] = m->pDistance;
    m->dlimits[2] = m->arc2Length;
    m->dlimits[3] = m->pDistance;
    for(i = 0; i < 4; i++)
        m->totalLength += m->dlimits[i];
    m->grouserWidth = m->totalLength / (double)m->numGrousers;
    return m;
}

void track_kinematic_model_destroy(TrackKinematicModel *m) {
    free(m);
}

void track_kinematic_model_get_point_on_path(TrackKinematicModel *m, dReal u, dReal *x_, dReal *y_, dReal *theta_) {
    // compute which piece of the path u touches
    // and compute local parameter v
    size_t i, section = -1;
    dReal v = NAN;
    dReal u1 = (u - floor(u)) * m->totalLength;
    for(i = 0; i < 4; i++) {
        if(u1 < m->dlimits[i]) {
            v = u1 / m->dlimits[i];
            section = i;
            break;
        }
        u1 -= m->dlimits[i];
    }
    // compute point and angle on path
    switch(section) {
        case 0:
            *theta_ = (1 - v) * (2 * M_PI - m->theta) + v * m->theta;
            *x_ = m->radius1 * cos(*theta_);
            *y_ = m->radius1 * sin(*theta_);
            break;
        case 1:
            *theta_ = m->theta;
            *x_ = (1 - v) * m->p1x + v * m->p2x;
            *y_ = (1 - v) * m->p1y + v * m->p2y;
            break;
        case 2:
            *theta_ = (1 - v) * m->theta + v * -m->theta;
            *x_ = m->distance + m->radius2 * cos(*theta_);
            *y_ = m->radius2 * sin(*theta_);
            break;
        case 3:
            *theta_ = -m->theta;
            *x_ = (1 - v) * m->p2x + v * m->p1x;
            *y_ = (1 - v) * -m->p2y + v * -m->p1y;
            break;
    }
}

void track_kinematic_model_compute_grouser_transform_3D(TrackKinematicModel *m, size_t i, dReal *pos, dReal *R) {
    // 2d track is on XZ plane, with roller axes in Y direction
    pos[1] = 0;
    dReal theta;
    track_kinematic_model_get_point_on_path(m, i / (dReal)m->numGrousers, &pos[0], &pos[2], &theta);
    dRFromAxisAndAngle(R, 0, 1, 0, -theta);
}

