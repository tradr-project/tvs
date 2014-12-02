//
//  main.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Physics.h"
#include "View.h"

Physics physics;

void interval(int v) {
    physics.step();
    glutPostRedisplay();
    glutTimerFunc(17, &interval, v);
}

int main(int argc, char **argv) {
    initOpenGL();
    physics.init();
    glutTimerFunc(0, interval, 0);
    glutMainLoop();
    physics.destroy();
    return 0;
}
