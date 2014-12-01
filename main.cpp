#include <ode/ode.h>

#include <GL/glut.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <cmath>

#define MAX_CONTACTS 10

#define NUM_X 10
#define NUM_Y 10
#define NUM_Z 10

void drawGeom(dGeomID g);

class Track;

struct Grouser {
    size_t i;
    dGeomID gTrans, gBox;
    dBodyID body;
    dReal x, y, z, theta;
    dMass mass;
};

class Track {
    friend class Grouser;
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
public:
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
    Track(dReal radius1_, dReal radius2_, dReal distance_, size_t numGrousers_, dReal grouserWidth_, dReal grouserHeight_, dReal trackDepth_) : grousers(numGrousers_) {
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

    inline dReal scale(dReal v, dReal vmin, dReal vmax) {
        return v * (vmax - vmin) + vmin;
    }

    inline size_t section(dReal u, dReal *v) {
        dReal u1 = (u - floor(u)) * totalLength;
        for(size_t i = 0; i < sections; i++) {
            if(u1 < dlimits[i]) {
                *v = u1 / dlimits[i];
                return i;
            } else {
                u1 -= dlimits[i];
            }
        }
        std::cerr << "WTF in Track::section(" << u << ")" << std::endl;
        std::cerr << "    u1 = " << u1 << std::endl;
        return -1;
    }

    void get(dReal u, dReal *p, dReal *a) {
        dReal v;
        switch(section(u, &v)) {
        case 0:
            *a = scale(v, 2 * M_PI - theta, theta);
            p[0] = radius1 * cos(*a);
            p[1] = radius1 * sin(*a);
            break;
        case 1:
            *a = theta;
            p[0] = scale(v, p1x, p2x);
            p[1] = scale(v, p1y, p2y);
            break;
        case 2:
            *a = scale(v, theta, -theta);
            p[0] = distance + radius2 * cos(*a);
            p[1] = radius2 * sin(*a);
            break;
        case 3:
            *a = -theta;
            p[0] = scale(v, p2x, p1x);
            p[1] = scale(v, -p2y, -p1y);
            break;
        default:
            std::cerr << "WTF in Track::get()" << std::endl;
        }
    }
    
    void get(size_t i, dReal u, dReal *p, dReal *a) {
        get(i/(dReal)numGrousers + u, p, a);
    }

    void createAll(dWorldID world, dSpaceID space) {
        // mass of the composite object:
        dMassSetZero(&mass);

        // size of the boxes:
        dReal sides[] = {grouserWidth, grouserHeight, trackDepth};

        for(size_t i = 0; i < numGrousers; i++) {
            grousers[i].i = i;
            
            // get positioon of object on curve:
            dReal p[2];
            get(grousers[i].i, 0.0, p, &grousers[i].theta);
            grousers[i].x = p[0];
            grousers[i].y = p[1];
            grousers[i].z = 0.0;
            dMatrix3 R;
            dRFromAxisAndAngle(R, 0.0, 0.0, 1.0, grousers[i].theta);
            
            // sub object will be added to this transform:
            grousers[i].gTrans = dCreateGeomTransform(space);
            dGeomTransformSetCleanup(grousers[i].gTrans, 1);
            
            grousers[i].gBox = dCreateBox(0, sides[0], sides[1], sides[2]);
            
            dMassSetBox(&grousers[i].mass, 1.0, sides[0], sides[1], sides[2]);
            
            dGeomTransformSetGeom(grousers[i].gTrans, grousers[i].gBox);
            
            dGeomSetPosition(grousers[i].gBox, grousers[i].x, grousers[i].y, grousers[i].z);
            dMassTranslate(&grousers[i].mass, grousers[i].x, grousers[i].y, grousers[i].z);
            
            dGeomSetRotation(grousers[i].gBox, R);
            dMassRotate(&grousers[i].mass, R);
            
            dMassAdd(&mass, &grousers[i].mass);
        }
        
        // translate so that center of mass is at origin and ode won't complain
        for(int i = 0; i < numGrousers; i++) {
            dGeomSetPosition(grousers[i].gBox,
                              grousers[i].x - mass.c[0],
                              grousers[i].y - mass.c[1],
                              grousers[i].z - mass.c[2]);
        }
        dMassTranslate(&mass,
                        -mass.c[0],
                        -mass.c[1],
                        -mass.c[2]);

        body = dBodyCreate(world);
        
        for(int i = 0; i < numGrousers; i++) {
            dGeomSetBody(grousers[i].gTrans, body);
        }
        
        dBodySetMass(body, &mass);
    }
    
    void test() {
        dBodyAddTorque(body, 0.3, 2.0, 4.0);
    }

    void draw() {
        for(std::vector<Grouser>::iterator it = grousers.begin(); it != grousers.end(); it++) {
            drawGeom(it->gBox);
        }
    }
};

dWorldID world;
dSpaceID space;
dJointGroupID contactGroup;

Track track(0.2, 0.15, 0.45, 20, 0.03, 0.06, 0.18);
dGeomID planeGeom;

void initODE() {
    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);

    world = dWorldCreate();
    space = dSimpleSpaceCreate(0);
    contactGroup = dJointGroupCreate(0);
    dWorldSetGravity(world, 0, -9.81 * 0.2, 0);
    dWorldSetERP(world, 0.2);
    dWorldSetCFM(world, 1e-5);
    dWorldSetContactMaxCorrectingVel(world, 0.9);
    dWorldSetContactSurfaceLayer(world, 0.001);
    dWorldSetAutoDisableFlag(world, 1);

    planeGeom = dCreatePlane(space, 0, 1, 0, -0.3); // (a, b, c)' (x, y, z) = d

    track.createAll(world, space);
}

static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
    int i;
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    dContact contact[MAX_CONTACTS];
    for(i = 0; i < MAX_CONTACTS; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = dInfinity;
        contact[i].surface.mu2 = 0;
        contact[i].surface.bounce = 0.5;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.001;
    }
    if(int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact))) {
        for(i = 0; i < numc; i++) {
            dJointID c = dJointCreateContact(world, contactGroup, contact + i);
            dJointAttach(c, b1, b2);
        }
    }
}

void simulationStep(int v) {
    static int stepNum = 0;
    std::cout << "about to perform simulation step " << stepNum << std::endl;

    dSpaceCollide(space, 0, &nearCallback);
    dWorldQuickStep(world, 0.05);
    dJointGroupEmpty(contactGroup);

    if(stepNum > 100) track.test();
    
    stepNum++;
}

void destroyODE() {
    dJointGroupDestroy(contactGroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
}

void glMultMatrixODE(const dReal* p, const dReal* R) {
    GLfloat M[16];
    M[0]  = (GLfloat)R[0]; M[1]  = (GLfloat)R[4]; M[2]  = (GLfloat)R[8];  M[3]  = (GLfloat)0;
    M[4]  = (GLfloat)R[1]; M[5]  = (GLfloat)R[5]; M[6]  = (GLfloat)R[9];  M[7]  = (GLfloat)0;
    M[8]  = (GLfloat)R[2]; M[9]  = (GLfloat)R[6]; M[10] = (GLfloat)R[10]; M[11] = (GLfloat)0;
    M[12] = (GLfloat)p[0]; M[13] = (GLfloat)p[1]; M[14] = (GLfloat)p[2];  M[15] = (GLfloat)1;
    glMultMatrixf(&M[0]);
}

GLfloat light_diffuse[] = {1.0, 0.0, 0.0, 1.0};
GLfloat light_position[] = {1.0, 1.0, 10.0, 0.0};

void drawBox(const dReal sides[3], const dReal pos[3], const dReal R[12]) {
    static GLfloat mat_ambient[] = { 0.8, 0.8, 0.8, 1.0 };
    static GLfloat mat_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
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

    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);

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

void drawGeom(dGeomID g) {
    if(!g) return;
    
    int type = dGeomGetClass(g);
    
    if(type == dPlaneClass) {
        dReal params[4];
        dGeomPlaneGetParams(g, params);

        return;
    }
    
    const dReal *pos = dGeomGetPosition(g);
    const dReal *R = dGeomGetRotation(g);

    if(type == dBoxClass) {
        dReal sides[3];
        dGeomBoxGetLengths(g, sides);
        drawBox(sides, pos, R);
        return;
    }
}

GLfloat cameraTheta = 14.9, cameraPhi = 0.5, cameraDist = 1.6;

void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    GLfloat x = cameraDist * cos(cameraTheta) * sin(cameraPhi);
    GLfloat y = cameraDist * sin(cameraTheta) * sin(cameraPhi);
    GLfloat z = cameraDist * cos(cameraPhi);
    gluLookAt(x, y, z,   /* eye */
        0.0, 0.0, 0.0,   /* center */
        0.0, 1.0, 0.0);  /* up direction vector */

    drawGeom(planeGeom);
    track.draw();

    glutSwapBuffers();
}

void initOpenGL(void) {
    /* Enable a single OpenGL light. */
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);

    /* Use depth buffering for hidden surface elimination. */
    glEnable(GL_DEPTH_TEST);

    /* Setup the view of the cube. */
    glMatrixMode(GL_PROJECTION);
    gluPerspective(/* field of view in degree */ 40.0,
        /* aspect ratio */ 1.333,
        /* Z near */ 1.0, /* Z far */ 100.0);
}

static int startx = 0, starty = 0, left = 0, right = 0;

void mouseButton(int button, int state, int x, int y) {
    left = 0;
    right = 0;
    if(button == GLUT_LEFT_BUTTON) {
        left = 1;
    } else if(button == GLUT_RIGHT_BUTTON) {
        right = 1;
    } else {
        return;
    }
    if(state == GLUT_DOWN) {
        startx = x;
        starty = y;
    }
}

void mouseMotion(int x, int y) {
    int dx = x - startx;
    int dy = y - starty;
    startx = x;
    starty = y;
    if(left) {
        cameraTheta += 0.1 * dx;
        cameraPhi += 0.1 * dy;
    }
    if(right) {
        cameraDist += 0.1 * dy;
    }
    cameraPhi = fmaxf(0.1, fminf(0.9, cameraPhi));
    cameraDist = fmaxf(0.1, fminf(10.0, cameraDist));
    std::cout
    << "cameraTheta = " << cameraTheta << std::endl
    << "cameraPhi = " << cameraPhi << std::endl
    << "cameraDist = " << cameraDist << std::endl;
}

void interval(int v) {
    simulationStep(v);
    glutPostRedisplay();
    glutTimerFunc(17, &interval, v);
}

int main(int argc, char **argv) {
#ifdef WITHOUT_OPENGL
    initODE();
    for(int i = 0; i < 200; i++) simulationStep(0);
    destroyODE();
#else
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("ODE");
    glutDisplayFunc(display);
    glutMouseFunc(mouseButton);
    glutMotionFunc(mouseMotion);
    glutTimerFunc(0, interval, 0);
    initOpenGL();
    initODE();
    glutMainLoop();
    destroyODE();
#endif
    return 0;
}
