//
//  Environment.h
//  tvs
//
//  Created by Federico Ferri on 17/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef WORLD_H_INCLUDED
#define WORLD_H_INCLUDED

#include <ode/ode.h>
#include <string>
#include <vector>
#include <map>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "TrackedVehicle.h"

struct ContactParams {
    int max_contacts;
    dReal bounce;
    dReal bounce_vel;
    dReal soft_cfm;
    dReal mu;
    dReal mu2;
};

struct StepParams {
    dReal step_size;
    int simulation_steps_per_frame;
};

struct WorldParams {
    dReal gravity_x, gravity_y, gravity_z;
};

struct Config {
    StepParams step;
    WorldParams world;
    ContactParams contact_wheel_grouser;
    ContactParams contact_grouser_terrain;
    ContactParams contact_grouser_guide;
    ContactParams contact_default;
};

class Environment {
public:
    Config config;

    dWorldID world;
    dSpaceID space;
    dJointGroupID contactGroup;
    std::map<dGeomID, std::string> geomNames;

    dGeomID planeGeom;
    TrackedVehicle *v;
    std::vector<dGeomID> boxes;
    
    bool badCollision;
    size_t stepNum;
    
    Environment();
    virtual ~Environment();
    void readConfig();
    void create();
    void destroy();
    std::string getGeomName(dGeomID geom) const;
    void setGeomName(dGeomID geom, const std::string &name);
    bool isCatPair(unsigned long cat1, unsigned long cat2, dGeomID *o1, dGeomID *o2);
    bool isValidCollision(dGeomID o1, dGeomID o2, const dContact& contact);
    void nearCallback(dGeomID o1, dGeomID o2);
    void nearCallbackWheelGrouser(dGeomID o1, dGeomID o2);
    void nearCallbackGrouserTerrain(dGeomID o1, dGeomID o2);
    void nearCallbackGrouserGuide(dGeomID o1, dGeomID o2);
    void nearCallbackDefault(dGeomID o1, dGeomID o2);
    bool step(dReal stepSize, int simulationStepsPerFrame);
    bool step();
    void evaluateCollisionNearCallback(dGeomID o1, dGeomID o2);
    bool evaluateCollision();
    void draw();
};

namespace Category { enum Category {
    TERRAIN  = 1 << 0,
    GROUSER  = 1 << 1,
    G_GUIDE  = 1 << 2,
    WHEEL    = 1 << 3,
    OBSTACLE = 1 << 4
}; };

#endif // WORLD_H_INCLUDED
