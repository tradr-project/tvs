//
//  View.h
//  tvs
//
//  Created by Federico Ferri on 01/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef __tvs__View__
#define __tvs__View__

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <ode/ode.h>

class View {
private:
    // camera params:
    GLfloat cx, cy, cz, ex, ey, ez, ux, uy, uz;
    GLfloat cameraTheta;
    GLfloat cameraHeight;
    GLfloat cameraDist;
    
    // mouse interaction:
    double lastx;
    double lasty;
    int mcenter;
    int mrot;
    int mdist;
    
    GLFWwindow *window;
public:
    void init();
    void initGLFW();
    void initGL();
    void display();
    void draw();
    void destroy();
    void setCamera(GLfloat theta, GLfloat height, GLfloat dist);
    
    static void errorCallback(int error, const char *description);
    static void keyCallbackWrapper(GLFWwindow *window, int key, int scancode, int action, int mods);
    static void framebufferSizeCallbackWrapper(GLFWwindow *window, int width, int height);
    static void mouseButtonCallbackWrapper(GLFWwindow *window, int button, int action, int mods);
    static void mousePositionCallbackWrapper(GLFWwindow *window, double x, double y);
    
    void keyCallback(int key, int scancode, int action, int mods);
    void framebufferSizeCallback(int width, int height);
    void mouseButtonCallback(int button, int action, int mods);
    void mousePositionCallback(double x, double y);
    
    bool shutdown();
    
    void glMultMatrixODE(const dReal* p, const dReal* R);
    void drawBox(const dReal sides[3], const dReal pos[3], const dReal R[12]);
    void drawPlane(const dReal params[4]);
    void drawCylinder(dReal r, dReal h, const dReal pos[3], const dReal R[12]);
    void drawCone(dReal r, dReal h, const dReal pos[3], const dReal R[12]);
    void drawSphere(dReal r, const dReal pos[3], const dReal R[12]);
    void drawAxes(const dReal pos[3], const dReal R[12]);
    void drawAxes(dReal w, dReal l, dReal aw, dReal al, const dReal pos[3], const dReal R[12]);
    void drawGeom(dGeomID g);
    void setColor(GLfloat r, GLfloat g, GLfloat b);
    void pollEvents();
};

#endif /* defined(__tvs__View__) */
