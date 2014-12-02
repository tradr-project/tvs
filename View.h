//
//  View.h
//  tvs
//
//  Created by Federico Ferri on 01/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef __tvs__View__
#define __tvs__View__

#include <GL/glut.h>
#include <ode/ode.h>

void drawGeom(dGeomID g);
void drawBox(const dReal sides[3], const dReal pos[3], const dReal R[12]);
void drawPlane(const dReal params[4]);
void drawCylinder(dReal r, dReal h, const dReal pos[3], const dReal R[12]);
void drawCone(dReal r, dReal h, const dReal pos[3], const dReal R[12]);
void drawSphere(dReal r, const dReal pos[3], const dReal R[12]);
void drawAxes(const dReal pos[3], const dReal R[12]);
void drawAxes(dReal w, dReal l, dReal aw, dReal al, const dReal pos[3], const dReal R[12]);
void setColor(GLfloat r, GLfloat g, GLfloat b);
void glMultMatrixODE(const dReal* p, const dReal* R);
void initOpenGL();

#endif /* defined(__tvs__View__) */
