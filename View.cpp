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

dMatrix3 I;
dVector3 O;

extern Physics physics;

#define checkError2()     checkError(__FILE__, __LINE__, __FUNCTION__)

void View::init() {
    cx = 0.0; cy = 0.0; cz = 0.0;
    ex = 0.0; ey = 1.0; ez = -5.0;
    ux = 0.0; uy = 1.0; uz = 0.0;
    cameraTheta = 14.3;
    cameraHeight = 0.5;
    cameraDist = 3.1;
    setCamera(cameraTheta, cameraHeight, cameraDist);

    lastx = 0.0;
    lasty = 0.0;
    mcenter = 0;
    mrot = 0;
    mdist = 0;
    
    dRSetIdentity(I);
    O[0] = O[1] = O[2] = 0.0;
    
    initGLFW();
    initGL();
}

void View::initGLFW() {
    if(!glfwInit()) {
        std::cerr << "error: failed to initialize GLFW" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    glfwSetErrorCallback(errorCallback);
    
    glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
    
    // Open a window and create its OpenGL context
    window = glfwCreateWindow(800, 600, "tvs", NULL, NULL);
    if(window == NULL) {
        std::cerr << "error: failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try with OpenGL 2.1." << std::endl;
        destroy();
        exit(1);
    }
    
    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, keyCallbackWrapper);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallbackWrapper);
    glfwSetMouseButtonCallback(window, mouseButtonCallbackWrapper);
    glfwSetCursorPosCallback(window, mousePositionCallbackWrapper);
    glfwMakeContextCurrent(window);

    glCheckError();
}

void View::initGL() {
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    double ratio = width / (double) height;
    glViewport(0, 0, width, height);
    
    /* Enable a single OpenGL light. */
    GLfloat light_diffuse[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat light_position[] = {1.0, 1.0, 10.0, 0.0};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);

    /* Use depth buffering for hidden surface elimination. */
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);
    
    /* enable antialiasing */
    glEnable(GL_MULTISAMPLE_ARB);
    
    /* Setup the view of the cube. */
    glMatrixMode(GL_PROJECTION);
    gluPerspective(/* field of view in degree */ 40.0,
                   /* aspect ratio */ ratio,
                   /* Z near */ 1.0, /* Z far */ 100.0);
    
    glMatrixMode(GL_MODELVIEW);
    
    glClearColor(0.19f, 0.2f, 0.21f, 1.0f);
    
    glCheckError();
}

void View::display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glLoadIdentity();
    gluLookAt(ex, ey, ez,  /* eye */
              cx, cy, cz,  /* center */
              ux, uy, uz); /* up direction vector */
    
    draw();
    
    glfwSwapBuffers(window);
    
    glCheckError();
}

void View::draw() {
    drawAxes(O, I);
    
    setColor(0.35, 0.35, 0.35);
    drawGeom(physics.planeGeom);
    
    setColor(1.0, 1.0, 1.0);
    physics.track.draw(this);
    
    if(mcenter) {
        setColor(1.0, 0.0, 0.0);
        dReal p[] = {cx, cy, cz};
        drawSphere(0.02, p, I);
    }
    
    setColor(1.0, 0.0, 1.0);
    for(int i = 0; i < physics.contactsCache.size(); i++) {
        drawSphere(0.02 + 0.001*physics.contactsCache[i].depth, physics.contactsCache[i].pos, I);
        //dMatrix3 R;
        //dRFrom2Axes(R, 1.0, 0.0, 0.0, physics.contactsCache[i].normal[0], physics.contactsCache[i].normal[1], physics.contactsCache[i].normal[2]);
        //drawCylinder(0.01, 0.09, physics.contactsCache[i].pos, R);
    }
}

void View::destroy() {
    glfwDestroyWindow(window);
    glfwTerminate();
}

void View::setCamera(GLfloat theta, GLfloat height, GLfloat dist) {
    ex = cx + cameraDist * cos(cameraTheta);
    ez = cy + cameraDist * sin(cameraTheta);
    ey = cz + cameraHeight;
}

void View::errorCallback(int error, const char *description) {
    std::cerr << "error: GLFW: " << description << std::endl;
}

void View::keyCallbackWrapper(GLFWwindow *window, int key, int scancode, int action, int mods) {
    ((View *)glfwGetWindowUserPointer(window))->keyCallback(key, scancode, action, mods);
}

void View::framebufferSizeCallbackWrapper(GLFWwindow *window, int width, int height) {
    ((View *)glfwGetWindowUserPointer(window))->framebufferSizeCallback(width, height);
}

void View::mouseButtonCallbackWrapper(GLFWwindow *window, int button, int action, int mods) {
    ((View *)glfwGetWindowUserPointer(window))->mouseButtonCallback(button, action, mods);
}

void View::mousePositionCallbackWrapper(GLFWwindow *window, double x, double y) {
    ((View *)glfwGetWindowUserPointer(window))->mousePositionCallback(x, y);
}

void View::keyCallback(int key, int scancode, int action, int mods) {
    if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
}

void View::framebufferSizeCallback(int width, int height) {
    glViewport(0, 0, width, height);
}

void View::mouseButtonCallback(int button, int action, int mods) {
    if(button == GLFW_MOUSE_BUTTON_LEFT) {
        mcenter = mrot = mdist = 0;
        if(action == GLFW_PRESS) {
            glfwGetCursorPos(window, &lastx, &lasty);
            if(mods & GLFW_MOD_SHIFT) mcenter = 1;
            else if(mods & GLFW_MOD_CONTROL) mdist = 1;
            else mrot = 1;
        }
    }
}

void View::mousePositionCallback(double x, double y) {
    double dx = x - lastx;
    double dy = y - lasty;
    lastx = x;
    lasty = y;

    if(mrot) {
        cameraTheta += 0.01 * dx;
        cameraHeight += 0.01 * dy;
    } else if(mdist) {
        cameraTheta += 0.01 * dx;
        cameraDist += 0.01 * dy;
    } else if(mcenter) {
        cx += 0.005 * dx;
        cz += 0.005 * dy;
    }
    cameraHeight = fmaxf(0.1, fminf(4.0, cameraHeight));
    cameraDist = fmaxf(0.1, fminf(10.0, cameraDist));
    if(mrot || mdist || mcenter) {
        std::cout
            << "cameraTheta = " << cameraTheta << std::endl
            << "cameraHeight = " << cameraHeight << std::endl
            << "cameraDist = " << cameraDist << std::endl;
        setCamera(cameraTheta, cameraHeight, cameraDist);
    }
}

bool View::shutdown() {
    return glfwWindowShouldClose(window);
}

void View::glMultMatrixODE(const dReal* p, const dReal* R) {
    GLfloat M[16];
    M[0]  = (GLfloat)R[0]; M[1]  = (GLfloat)R[4]; M[2]  = (GLfloat)R[8];  M[3]  = (GLfloat)0;
    M[4]  = (GLfloat)R[1]; M[5]  = (GLfloat)R[5]; M[6]  = (GLfloat)R[9];  M[7]  = (GLfloat)0;
    M[8]  = (GLfloat)R[2]; M[9]  = (GLfloat)R[6]; M[10] = (GLfloat)R[10]; M[11] = (GLfloat)0;
    M[12] = (GLfloat)p[0]; M[13] = (GLfloat)p[1]; M[14] = (GLfloat)p[2];  M[15] = (GLfloat)1;
    glMultMatrixf(&M[0]);
}

void View::drawBox(const dReal sides[3], const dReal pos[3], const dReal R[12]) {
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

void View::drawPlane(const dReal params[4]) {
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

void View::drawCylinder(dReal r, dReal h, const dReal pos[3], const dReal R[12]) {
    glPushMatrix();
    glMultMatrixODE(pos, R);
    GLUquadric *q = gluNewQuadric();
    gluCylinder(q, r, r, h, 12, 2);
    gluDeleteQuadric(q);
    glPopMatrix();
}

void View::drawCone(dReal r, dReal h, const dReal pos[3], const dReal R[12]) {
    glPushMatrix();
    glMultMatrixODE(pos, R);
    GLUquadric *q = gluNewQuadric();
    gluCylinder(q, r, 0, h, 12, 2);
    gluDeleteQuadric(q);
    glPopMatrix();
}

void View::drawSphere(dReal r, const dReal pos[3], const dReal R[12]) {
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

void View::drawAxes(const dReal pos[3], const dReal R[12]) {
    drawAxes(0.01, 0.1, 0.02, 0.04, pos, R);
}

void View::drawAxes(dReal w, dReal l, dReal aw, dReal al, const dReal pos[3], const dReal R[12]) {
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

void View::drawGeom(dGeomID g) {
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

void View::setColor(GLfloat r, GLfloat g, GLfloat b) {
    GLfloat setColor_material[] = {r, g, b, 1.0};
    glMaterialfv(GL_FRONT, GL_DIFFUSE, setColor_material);
    glColor3f(r, g, b);
}

void View::pollEvents() {
    glfwPollEvents();
}
