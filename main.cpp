//
//  main.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Physics.h"
#include "View.h"

View view;
Physics physics;

int main(int argc, char **argv) {
    view.init();
    physics.init();
    do {
        physics.step();
        view.display();
        view.pollEvents();
    }
    while(!view.shutdown());
    view.destroy();
    physics.destroy();
    return 0;
}
