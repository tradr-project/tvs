//
//  View.cpp
//  tvs
//
//  Created by Federico Ferri on 01/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "View.h"
#include "Physics.h"

#include <iostream>

GLfloat cameraTheta = 14.3;
GLfloat cameraHeight = 0.5;
GLfloat cameraDist = 3.1;
dReal cameraCenter[] = {0.0, 0.0, 0.0};
static int lastx = 0, lasty = 0, mcenter = 0, mrot = 0, mdist = 0;

dMatrix3 I;
dVector3 O;

extern Physics physics;

void glMultMatrixODE(const dReal* p, const dReal* R) {
    GLfloat M[16];
    M[0]  = (GLfloat)R[0]; M[1]  = (GLfloat)R[4]; M[2]  = (GLfloat)R[8];  M[3]  = (GLfloat)0;
    M[4]  = (GLfloat)R[1]; M[5]  = (GLfloat)R[5]; M[6]  = (GLfloat)R[9];  M[7]  = (GLfloat)0;
    M[8]  = (GLfloat)R[2]; M[9]  = (GLfloat)R[6]; M[10] = (GLfloat)R[10]; M[11] = (GLfloat)0;
    M[12] = (GLfloat)p[0]; M[13] = (GLfloat)p[1]; M[14] = (GLfloat)p[2];  M[15] = (GLfloat)1;
    glMultMatrixf(&M[0]);
}

void drawBox(const dReal sides[3], const dReal pos[3], const dReal R[12]) {
    static GLfloat n[6][3] = {  /* Normals for the 6 faces of a cube. */
        {-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0},
        {0.0, -1.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 0.0, 1.0} };
    static GLint faces[6][4] = {  /* Vertex indices for the 6 faces of a cube. */
        {0, 1, 2, 3}, {3, 2, 6, 7}, {7, 6, 5, 4},
        {4, 5, 1, 0}, {5, 6, 2, 1}, {7, 4, 0, 3} };
    
    GLfloat v[8][3]; /* Cube vertex data. */
    v[0][0] = v[1][0] = v[2][0] = v[3][0] = -0.5*sides[0];
    v[4][0] = v[5][0] = v[6][0] = v[7][0] = 0.5*sides[0];
    v[0][1] = v[1][1] = v[4][1] = v[5][1] = -0.5*sides[1];
    v[2][1] = v[3][1] = v[6][1] = v[7][1] = 0.5*sides[1];
    v[0][2] = v[3][2] = v[4][2] = v[7][2] = 0.5*sides[2];
    v[1][2] = v[2][2] = v[5][2] = v[6][2] = -0.5*sides[2];
    
    glPushMatrix();
    glMultMatrixODE(pos, R);
    for(int i = 0; i < 6; i++) {
        glBegin(GL_QUADS);
        glNormal3fv(&n[i][0]);
        glVertex3fv(&v[faces[i][0]][0]);
        glNormal3fv(&n[i][0]);
        glVertex3fv(&v[faces[i][1]][0]);
        glNormal3fv(&n[i][0]);
        glVertex3fv(&v[faces[i][2]][0]);
        glNormal3fv(&n[i][0]);
        glVertex3fv(&v[faces[i][3]][0]);
        glEnd();
    }
    glPopMatrix();
}

void drawPlane(const dReal params[4]) {
    double a = params[0], b = params[1], c = params[2], d = params[3];
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDisable(GL_LIGHTING);
    glBegin(GL_QUADS);
    const dReal incr = 0.2, ext = 10.0;
    for(dReal i = -ext; i < ext; i += incr) {
        for(dReal j = -ext; j < ext; j += incr) {
            glVertex3d(i, -(a*i+c*j-d)/b, j);
            glVertex3d(i, -(a*i+c*(j+1)-d)/b, j+1);
            glVertex3d(i+incr, -(a*(i+1)+c*(j+1)-d)/b, j+1);
            glVertex3d(i+incr, -(a*(i+1)+c*j-d)/b, j);
        }
    }
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_LIGHTING);
}

void drawCylinder(dReal r, dReal h, const dReal pos[3], const dReal R[12]) {
    glPushMatrix();
    glMultMatrixODE(pos, R);
    GLUquadric *q = gluNewQuadric();
    gluCylinder(q, r, r, h, 12, 2);
    gluDeleteQuadric(q);
    glPopMatrix();
}

void drawCone(dReal r, dReal h, const dReal pos[3], const dReal R[12]) {
    glPushMatrix();
    glMultMatrixODE(pos, R);
    GLUquadric *q = gluNewQuadric();
    gluCylinder(q, r, 0, h, 12, 2);
    gluDeleteQuadric(q);
    glPopMatrix();
}

void drawSphere(dReal r, const dReal pos[3], const dReal R[12]) {
    glPushMatrix();
    glMultMatrixODE(pos, R);
    GLUquadric *q = gluNewQuadric();
    gluSphere(q, r, 12, 6);
    gluDeleteQuadric(q);
    glPopMatrix();
}

// B = A'
void t(dReal B[12], const dReal A[12]) {
    B[0]  = A[0]; B[4]  = A[1]; B[8]  = A[2];
    B[1]  = A[4]; B[5]  = A[5]; B[9]  = A[6];
    B[2]  = A[8]; B[6]  = A[9]; B[10] = A[10];
}

void drawAxes(const dReal pos[3], const dReal R[12]) {
    drawAxes(0.01, 0.1, 0.02, 0.04, pos, R);
}

void drawAxes(dReal w, dReal l, dReal aw, dReal al, const dReal pos[3], const dReal R[12]) {
    dVector3 x, y, z;
    x[0] = y[1] = z[2] = l;
    x[1] = x[2] = y[0] = y[2] = z[0] = z[1] = 0;
    dVector3 posX, posY, posZ;
    dMatrix3 Rt; t(Rt,R);
    dMultiply1_331(posX, Rt, x);
    dMultiply1_331(posY, Rt, y);
    dMultiply1_331(posZ, Rt, z);
    dMatrix3 Rx_, Ry_, Rz_, Rx, Ry, Rz;
    dRFromAxisAndAngle(Rx_, 0, 1, 0, M_PI_2);
    dRFromAxisAndAngle(Ry_, 1, 0, 0, -M_PI_2);
    dRFromAxisAndAngle(Rz_, 1, 0, 0, 0);
    dMULTIPLY1_333(Rx, Rt, Rx_);
    dMULTIPLY1_333(Ry, Rt, Ry_);
    dMULTIPLY1_333(Rz, Rt, Rz_);
    posX[0] += pos[0]; posX[1] += pos[1]; posX[2] += pos[2];
    posY[0] += pos[0]; posY[1] += pos[1]; posY[2] += pos[2];
    posZ[0] += pos[0]; posZ[1] += pos[1]; posZ[2] += pos[2];
    
    setColor(1.0, 0.0, 0.0);
    drawCylinder(w, l, pos, Rx);
    drawCone(aw, al, posX, Rx);
    
    setColor(0.0, 1.0, 0.0);
    drawCylinder(w, l, pos, Ry);
    drawCone(aw, al, posY, Ry);
    
    setColor(0.0, 0.0, 1.0);
    drawCylinder(w, l, pos, Rz);
    drawCone(aw, al, posZ, Rz);
    
    setColor(1.0, 1.0, 1.0);
    drawSphere(w * 2.5, pos, R);
}

void drawGeom(dGeomID g) {
    if(!g) return;
    
    int type = dGeomGetClass(g);
    
    glPushMatrix();
    
    if(type == dPlaneClass) {
        dReal params[4];
        dGeomPlaneGetParams(g, params);
        drawPlane(params);
    } else {
        const dReal *pos = dGeomGetPosition(g);
        const dReal *R = dGeomGetRotation(g);
        
        if(type == dBoxClass) {
            dReal sides[3];
            dGeomBoxGetLengths(g, sides);
            drawBox(sides, pos, R);
        } else if(type == dSphereClass) {
            dReal radius = dGeomSphereGetRadius(g);
            drawSphere(radius, pos, R);
        } else if(type == dCylinderClass) {
            dReal radius, length;
            dGeomCylinderGetParams(g, &radius, &length);
            drawCylinder(radius, length, pos, R);
        }
    }
    
    glPopMatrix();
}

void setColor(GLfloat r, GLfloat g, GLfloat b) {
    GLfloat setColor_material[] = {r, g, b, 1.0};
    glMaterialfv(GL_FRONT, GL_DIFFUSE, setColor_material);
    glColor3f(r, g, b);
}

void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    GLfloat cx = cameraCenter[0], cy = cameraCenter[1], cz = cameraCenter[2];
    GLfloat x = cx + cameraDist * cos(cameraTheta);
    GLfloat z = cy + cameraDist * sin(cameraTheta);
    GLfloat y = cz + cameraHeight;
    gluLookAt(x, y, z,         /* eye */
              cx, cy, cz,      /* center */
              0.0, 1.0, 0.0);  /* up direction vector */
    
    //static GLfloat mat_ambient[] = { 1.0, 0.0, 0.0, 1.0 };
    //glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    
    //static GLfloat mat_diffuse[] = { 1.0, 0.0, 0.0, 1.0 };
    //glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
    
    drawAxes(O, I);
    
    setColor(0.35, 0.35, 0.35);
    drawGeom(physics.planeGeom);
    
    setColor(1.0, 1.0, 1.0);
    physics.track.draw();
    
    if(mcenter) {
        setColor(1.0, 0.0, 0.0);
        drawSphere(0.02, cameraCenter, I);
    }
    
#if 1
    setColor(1.0, 0.0, 1.0);
    for(int i = 0; i < physics.contactsCache.size(); i++) {
        drawSphere(0.02 + 0.001*physics.contactsCache[i].depth, physics.contactsCache[i].pos, I);
        //dMatrix3 R;
        //dRFrom2Axes(R, 1.0, 0.0, 0.0, physics.contactsCache[i].normal[0], physics.contactsCache[i].normal[1], physics.contactsCache[i].normal[2]);
        //drawCylinder(0.01, 0.09, physics.contactsCache[i].pos, R);
    }
#endif
    
    glutSwapBuffers();
}

void mouseButton(int button, int state, int x, int y) {
    if(button == GLUT_LEFT_BUTTON) {
        mcenter = mrot = mdist = 0;
        if(state == GLUT_DOWN) {
            lastx = x;
            lasty = y;
            int m = glutGetModifiers();
            //std::cout << "________________________ m = " << m << std::endl;
            switch(m) {
                case 0:
                    mrot = 1;
                    break;
                case GLUT_ACTIVE_SHIFT:
                    mcenter = 1;
                    break;
                case GLUT_ACTIVE_CTRL:
                    mdist = 1;
                    break;
            }
        }
    }
}

void mouseMotion(int x, int y) {
    int dx = x - lastx;
    int dy = y - lasty;
    lastx = x;
    lasty = y;
    if(mrot) {
        cameraTheta += 0.01 * dx;
        cameraHeight += 0.01 * dy;
    } else if(mdist) {
        cameraTheta += 0.01 * dx;
        cameraDist += 0.01 * dy;
    } else if(mcenter) {
        cameraCenter[0] += 0.005 * dx;
        cameraCenter[2] += 0.005 * dy;
    }
    cameraHeight = fmaxf(0.1, fminf(4.0, cameraHeight));
    cameraDist = fmaxf(0.1, fminf(10.0, cameraDist));
    std::cout
    << "cameraTheta = " << cameraTheta << std::endl
    << "cameraHeight = " << cameraHeight << std::endl
    << "cameraDist = " << cameraDist << std::endl;
}

void initOpenGL() {
    dRSetIdentity(I);
    O[0] = O[1] = O[2] = 0.0;
    
    int argc = 0; char *argv[1]; argv[0] = "blah";
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE);
    glutInitWindowSize(800, 600);
    glutCreateWindow("tvs");
    glutDisplayFunc(display);
    glutMouseFunc(mouseButton);
    glutMotionFunc(mouseMotion);
    
    /* Enable a single OpenGL light. */
    GLfloat light_diffuse[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat light_position[] = {1.0, 1.0, 10.0, 0.0};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    
    /* Use depth buffering for hidden surface elimination. */
    glEnable(GL_DEPTH_TEST);
    
    /* enable antialiasing */
    glEnable(GL_MULTISAMPLE_ARB);
    
    /* Setup the view of the cube. */
    glMatrixMode(GL_PROJECTION);
    gluPerspective(/* field of view in degree */ 40.0,
                   /* aspect ratio */ 1.333,
                   /* Z near */ 1.0, /* Z far */ 100.0);
    
    glClearColor(0.19f, 0.2f, 0.21f, 1.0f);
}

