#include <ode/ode.h>

#include <GL/glut.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>

#define MAX_CONTACTS 10

#define NUM_X 10
#define NUM_Y 10
#define NUM_Z 10

struct Object {
    dBodyID body;
    std::vector<dGeomID> geom;
};

dWorldID world;
dSpaceID space;
dJointGroupID contactGroup;

Object box1, box2;
dJointID joint;

std::vector<Object> objects;
std::vector<dGeomID> geoms;

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

    dGeomID planeGeom = dCreatePlane(space, 0, 1, 0, 0); // (a, b, c)' (x, y, z) = d
    geoms.push_back(planeGeom);

    {
        box1.body = dBodyCreate(world);
        dBodySetPosition(box1.body, 2.0, 8.0, 0.0);
        dMatrix3 R;
        dRFromAxisAndAngle(R,
            dRandReal() * 2.0 - 1.0,
            dRandReal() * 2.0 - 1.0,
            dRandReal() * 2.0 - 1.0,
            dRandReal() * 10.0 - 5.0);
        dBodySetRotation(box1.body, R);
        dBodySetLinearVel(box1.body,
            dRandReal() * 2.0 - 1.0,
            dRandReal() * 8.0 - 4.0,
            dRandReal() * 2.0 - 1.0);
        //dBodySetData(objects.body, (void*)0);
        dMass m;
        dReal sides[] = {0.66, 2.66, 0.66};
        dMassSetBox(&m, /* density: */ 1.0, sides[0], sides[1], sides[2]);
        dBodySetMass(box1.body, &m);
        dGeomID boxGeom = dCreateBox(space, sides[0], sides[1], sides[2]);
        geoms.push_back(boxGeom);
        box1.geom.push_back(boxGeom);
        dGeomSetBody(box1.geom[0], box1.body);
        objects.push_back(box1);
    }

    {
        box2.body = dBodyCreate(world);
        dBodySetPosition(box2.body, 0.0, 6.0, 0.0);
        dMatrix3 R;
        dRFromAxisAndAngle(R,
            dRandReal() * 2.0 - 1.0,
            dRandReal() * 2.0 - 1.0,
            dRandReal() * 2.0 - 1.0,
            dRandReal() * 10.0 - 5.0);
        dBodySetRotation(box2.body, R);
        dBodySetLinearVel(box2.body,
            dRandReal() * 2.0 - 1.0,
            dRandReal() * 2.0 - 1.0,
            dRandReal() * 2.0 - 1.0);
        //dBodySetData(objects.body, (void*)0);
        dMass m;
        dReal sides[] = {0.66, 0.66, 0.66};
        dMassSetBox(&m, /* density: */ 1.0, sides[0], sides[1], sides[2]);
        dBodySetMass(box2.body, &m);
        dGeomID boxGeom = dCreateBox(space, sides[0], sides[1], sides[2]);
        geoms.push_back(boxGeom);
        box2.geom.push_back(boxGeom);
        dGeomSetBody(box2.geom[0], box2.body);
        objects.push_back(box2);
    }

    joint = dJointCreateFixed(world, 0);
    dJointAttach(joint, objects[0].body, objects[1].body);
    dJointSetFixed(joint);
    //dJointSetHingeAnchor(joint, 0.0, 10.0, 0.0);
    //dJointSetHingeAxis(joint, 1, 0, 0);
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

    if(stepNum == 160) {
    }

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

void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 25.0, 25.0,  /* eye is at (0,5,5) */
        0.0, 0.0, 0.0,      /* center is at (0,0,0) */
        0.0, 1.0, 0.);      /* up is in positive Y direction */

    for(std::vector<dGeomID>::iterator it = geoms.begin(); it != geoms.end(); it++) {
        drawGeom(*it);
    }

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

void interval(int v) {
    simulationStep(v);
    glutPostRedisplay();
    glutTimerFunc(17, &interval, v);
}

int main(int argc, char **argv) {
#ifdef WITHOUT_OPENGL
    initODE();
    for(int i = 0; i < 100; i++) simulationStep(0);
    destroyODE();
#else
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("ODE");
    glutDisplayFunc(display);
    glutTimerFunc(0, &interval, 0);
    initOpenGL();
    initODE();
    glutMainLoop();
    destroyODE();
#endif
    return 0;
}
