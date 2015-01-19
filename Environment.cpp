//
//  Environment.cpp
//  tvs
//
//  Created by Federico Ferri on 17/12/2014.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Environment.h"
#include "ODEUtils.h"
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <iostream>
#include <drawstuff/drawstuff.h>
#include <ompl/util/Console.h>
#include <boost/foreach.hpp>
#include "utils.h"
#include "TrackedVehicle.h"
#include "SkidSteeringVehicle.h"

Environment::Environment() {
    datetime = getDateTimeString();
    readConfig();
#if 1
    TrackedVehicle *tv = new TrackedVehicle("robot", 1, -2, 0.301);
    tv->leftTrack->velocity.setSlope(config.world.track_acceleration);
    tv->rightTrack->velocity.setSlope(config.world.track_acceleration);
    this->v = tv;
#else
    SkidSteeringVehicle *ssv = new SkidSteeringVehicle("robot", 1, -2, 0.301);
    ssv->velocityLeft.setSlope(config.world.track_acceleration);
    ssv->velocityRight.setSlope(config.world.track_acceleration);
    this->v = ssv;
#endif
}

Environment::~Environment() {
    dJointGroupDestroy(this->contactGroup);
    dSpaceDestroy(this->space);
    dWorldDestroy(this->world);
    if(this->v) delete this->v;
}

static void readContactParams(std::string section, ContactParams* p, const boost::property_tree::ptree& pt) {
    p->max_contacts = pt.get<int>(section + ".max_contacts");
    p->bounce = pt.get<dReal>(section + ".bounce");
    p->bounce_vel = pt.get<dReal>(section + ".bounce_vel");
    p->soft_cfm = pt.get<dReal>(section + ".soft_cfm");
    if(pt.get<std::string>(section + ".mu") == "infinity") p->mu = dInfinity; else p->mu = pt.get<dReal>(section + ".mu");
    if(pt.get<std::string>(section + ".mu2", "") == "infinity") p->mu2 = dInfinity; else p->mu2 = pt.get<dReal>(section + ".mu2", p->mu);
    p->debug = pt.get<bool>(section + ".debug", false);
}

void Environment::readConfig() {
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(CONFIG_PATH "/simulator.ini", pt);
    config.step.step_size = pt.get<float>("step.step_size", 0.01);
    config.step.simulation_steps_per_frame = pt.get<int>("step.simulation_steps_per_frame", 4);
    readContactParams("contact_wheel_grouser", &config.contact_wheel_grouser, pt);
    readContactParams("contact_grouser_terrain", &config.contact_grouser_terrain, pt);
    readContactParams("contact_grouser_guide", &config.contact_grouser_guide, pt);
    readContactParams("contact_default", &config.contact_default, pt);
    config.world.gravity_x = pt.get<dReal>("world.gravity_x", 0.0);
    config.world.gravity_y = pt.get<dReal>("world.gravity_y", 0.0);
    config.world.gravity_z = pt.get<dReal>("world.gravity_z", 0.0);
    config.world.max_track_speed = pt.get<dReal>("world.max_track_speed", 5.0);
    config.world.track_acceleration = pt.get<dReal>("world.track_acceleration", 200.0);
    config.joystick.enabled = pt.get<unsigned short>("joystick.enabled", 0);
    config.joystick.device = pt.get<unsigned short>("joystick.device", 0);
    config.joystick.gain = pt.get<dReal>("joystick.gain", 1.0);
    config.show_contact_points = false;
}

void createAABox(Environment *e, dReal x1, dReal y1, dReal z1, dReal x2, dReal y2, dReal z2, unsigned long cat = Category::OBSTACLE, dReal rx = 1.0, dReal ry = 0.0, dReal rz = 0.0, dReal rAngle = 0.0) {
    static int i = 0;
    dGeomID g = dCreateBox(e->space, x2 - x1, y2 - y1, z2 - z1);
    dGeomSetPosition(g, x1 + 0.5 * (x2 - x1), y1 + 0.5 * (y2 - y1), z1 + 0.5 * (z2 - z1));
    dMatrix3 R;
    dRFromAxisAndAngle(R, rx, ry, rz, rAngle);
    dGeomSetRotation(g, R);
    dGeomSetCategoryBits(g, Category::TERRAIN);
    dGeomSetCollideBits(g, Category::GROUSER);
    e->setGeomName(g, "panel" + boost::lexical_cast<std::string>(i++));
    e->boxes.push_back(g);
}

void makeStairCase(Environment *e, dReal x1, dReal y1, dReal z1, dReal x2, dReal y2, dReal z2, int axis, int steps) {
    dReal dx = x2 - x1, dy = y2 - y1, dz = z2 - z1;
    dReal riser = dz / dReal(steps);
    dReal tread = (axis == 0 ? dx : dy) / dReal(steps);
    unsigned long cat = Category::TERRAIN;
    for(int i = 0; i < steps; i++) {
        if(axis == 0)
            createAABox(e, x1 + i * tread, y1, z1 + i * riser, x1 + (i + 1) * tread, y2, z1 + (i + 1) * riser, cat);
        else if(axis == 1)
            createAABox(e, x1, y1 + i * tread, z1 + i * riser, x2, y1 + (i + 1) * tread, z1 + (i + 1) * riser, cat);
        else if(axis == 3)
            createAABox(e, x1, y2 - (i + 1) * tread, z1 + i * riser, x2, y2 - i * tread, z1 + (i + 1) * riser, cat);
    }
}

void Environment::create() {
    this->world = dWorldCreate();
#if 0
    this->space = dQuadTreeSpaceCreate(0, center, extents, 6);
#else
    this->space = dHashSpaceCreate(0);
#endif
    this->contactGroup = dJointGroupCreate(0);
    dWorldSetGravity(this->world, config.world.gravity_x, config.world.gravity_y, config.world.gravity_z);
    //dWorldSetERP(this->world, 0.7);
    //dWorldSetCFM(this->world, 1e-5);
    //dWorldSetContactMaxCorrectingVel(this->world, 0.9);
    //dWorldSetContactSurfaceLayer(this->world, 0.001);
    dWorldSetAutoDisableFlag(this->world, 1);

    this->planeGeom = dCreatePlane(this->space, 0, 0, 1, 0); // (a, b, c)' (x, y, z) = d
    setGeomName(this->planeGeom, "worldPlane");
    dGeomSetCategoryBits(this->planeGeom, Category::TERRAIN);
    dGeomSetCollideBits(this->planeGeom, Category::GROUSER | Category::OBSTACLE);

    if(this->v) this->v->create(this);
    
    const dReal h = 1.3; // wall height
    const dReal t = 0.1; // wall thickness
    const dReal T = 0.25; // arch thickness
    const dReal l = 4; // ramp length
    const dReal W = 9; // total width
    const dReal D = 8; // total depth
    const dReal O = 3; // arch width
    const dReal w = 2; // ramp width
    const dReal sl = 5; // staircase length
    dReal a = atan2(h,l), l2 = hypot(h,l);
    createAABox(this, 0,   0,   0,   W-O, t,   h);
    createAABox(this, 0,   D-t, 0,   W,   D,   h);
    createAABox(this, 0,   0,   0,   t,   D,   h);
    createAABox(this, W-t, 0,   0,   W,   D,   h);
    createAABox(this, W-O, 0,   0,   W-O+T, w, h);
    createAABox(this, W-T, 0,   0,   W,   w, h);
    createAABox(this, W-O, 0,   h-t, W,   w, h, Category::TERRAIN);
    createAABox(this, W-O-0.5*l-0.5*l2, 0, 0.5*h-0.5*t-0.5*t, W-O-0.5*l+0.5*l2, 1.5, 0.5*h+0.5*t-0.5*t, Category::TERRAIN, 0, 1, 0, -a);
    makeStairCase(this, W-w, w, 0, W, w+sl, h, 3, 10);
}

void Environment::destroy() {
    if(this->v) this->v->destroy();
}

std::string Environment::getGeomName(dGeomID geom) const {
    std::map<dGeomID, std::string>::const_iterator it = geomNames.find(geom);
    if (it == geomNames.end())
        return boost::lexical_cast<std::string>(reinterpret_cast<unsigned long>(geom));
    else
        return it->second;
}

void Environment::setGeomName(dGeomID geom, const std::string &name) {
    geomNames[geom] = name;
}

bool Environment::isCatPair(unsigned long cat1, unsigned long cat2, dGeomID *o1, dGeomID *o2) {
    unsigned long catBits1 = dGeomGetCategoryBits(*o1);
    unsigned long catBits2 = dGeomGetCategoryBits(*o2);

    if((catBits1 & cat1) && (catBits2 & cat2)) {
        return true;
    }
    if((catBits1 & cat2) && (catBits2 & cat1)) {
        // swap o1 and o2
        dGeomID tmp = *o1;
        *o1 = *o2;
        *o2 = tmp;
        return true;
    }
    return false;
}

bool Environment::isValidCollision(dGeomID o1, dGeomID o2, const dContact& contact) {
    if(isCatPair(Category::GROUSER, Category::TERRAIN, &o1, &o2))
        return true;
    if(isCatPair(Category::GROUSER, Category::G_GUIDE, &o1, &o2))
        return true;
    if(isCatPair(Category::WHEEL, Category::G_GUIDE, &o1, &o2)) // XXX: not needed really
        return true;
    if(isCatPair(Category::WHEEL, Category::GROUSER, &o1, &o2))
        return true;
    if(isCatPair(Category::WHEEL, Category::TERRAIN, &o1, &o2))
        return true;
    return false;
}

static void nearCallbackWrapper(void *data, dGeomID o1, dGeomID o2) {
    reinterpret_cast<Environment *>(data)->nearCallback(o1, o2);
}

void Environment::nearCallback(dGeomID o1, dGeomID o2) {
    if(isCatPair(Category::WHEEL, Category::GROUSER, &o1, &o2))
        nearCallbackWheelGrouser(o1, o2);
    else if(isCatPair(Category::GROUSER, Category::TERRAIN, &o1, &o2))
        nearCallbackGrouserTerrain(o1, o2);
    else if(isCatPair(Category::GROUSER, Category::G_GUIDE, &o1, &o2))
        nearCallbackGrouserGuide(o1, o2);
    else
        nearCallbackDefault(o1, o2);
}

void Environment::nearCallbackWheelGrouser(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
    int maxc = config.contact_wheel_grouser.max_contacts;
    dContact contact[maxc];
    int numc = dCollide(o1, o2, maxc, &contact[0].geom, sizeof(dContact));
    for(size_t i = 0; i < numc; i++) {
        const dReal *v = dBodyGetLinearVel(b2); // grouser vel
        dCalcVectorCross3(contact[i].fdir1, contact[i].geom.normal, v);
        dSafeNormalize3(contact[i].fdir1);
        contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactMu2 | dContactFDir1;
        contact[i].surface.bounce = config.contact_wheel_grouser.bounce;
        contact[i].surface.bounce_vel = config.contact_wheel_grouser.bounce_vel;
        contact[i].surface.soft_cfm = config.contact_wheel_grouser.soft_cfm;
        contact[i].surface.mu = config.contact_wheel_grouser.mu;
        contact[i].surface.mu2 = config.contact_wheel_grouser.mu2;
        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
        if(!isValidCollision(o1, o2, contact[i]))
            this->badCollision = true;
        if(config.contact_wheel_grouser.debug)
            this->contacts.push_back(contact[i].geom);
    }
}

void Environment::nearCallbackGrouserTerrain(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
    int maxc = config.contact_grouser_terrain.max_contacts;
    dContact contact[maxc];
    int numc = dCollide(o1, o2, maxc, &contact[0].geom, sizeof(dContact));
    for(size_t i = 0; i < numc; i++) {
        const dReal *v = dBodyGetLinearVel(b1); // grouser vel
        dCalcVectorCross3(contact[i].fdir1, contact[i].geom.normal, v);
        dSafeNormalize3(contact[i].fdir1);
        contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactMu2 | dContactFDir1;
        contact[i].surface.bounce = config.contact_grouser_terrain.bounce;
        contact[i].surface.bounce_vel = config.contact_grouser_terrain.bounce_vel;
        contact[i].surface.soft_cfm = config.contact_grouser_terrain.soft_cfm;
        contact[i].surface.mu = config.contact_grouser_terrain.mu;
        contact[i].surface.mu2 = config.contact_grouser_terrain.mu2;
        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
        if(!isValidCollision(o1, o2, contact[i]))
            this->badCollision = true;
        if(config.contact_grouser_terrain.debug)
            this->contacts.push_back(contact[i].geom);
    }
}

void Environment::nearCallbackGrouserGuide(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
    int maxc = config.contact_grouser_guide.max_contacts;
    dContact contact[maxc];
    int numc = dCollide(o1, o2, maxc, &contact[0].geom, sizeof(dContact));
    for(size_t i = 0; i < numc; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.bounce = config.contact_grouser_guide.bounce;
        contact[i].surface.bounce_vel = config.contact_grouser_guide.bounce_vel;
        contact[i].surface.soft_cfm = config.contact_grouser_guide.soft_cfm;
        contact[i].surface.mu = config.contact_grouser_guide.mu;
        contact[i].surface.mu2 = config.contact_grouser_guide.mu2;
        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
        if(!isValidCollision(o1, o2, contact[i]))
            this->badCollision = true;
        if(config.contact_grouser_guide.debug)
            this->contacts.push_back(contact[i].geom);
    }
}

void Environment::nearCallbackDefault(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
    int maxc = config.contact_default.max_contacts;
    dContact contact[maxc];
    int numc = dCollide(o1, o2, maxc, &contact[0].geom, sizeof(dContact));
    for(size_t i = 0; i < numc; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.bounce = config.contact_default.bounce;
        contact[i].surface.bounce_vel = config.contact_default.bounce_vel;
        contact[i].surface.soft_cfm = config.contact_default.soft_cfm;
        contact[i].surface.mu = config.contact_default.mu;
        contact[i].surface.mu2 = config.contact_default.mu2;
        dJointID c = dJointCreateContact(this->world, this->contactGroup, &contact[i]);
        dJointAttach(c, b1, b2);
        if(!isValidCollision(o1, o2, contact[i]))
            this->badCollision = true;
        if(config.contact_default.debug)
            this->contacts.push_back(contact[i].geom);
    }
}

bool Environment::step(dReal stepSize, int simulationStepsPerFrame) {
    stepNum++;

    if(this->v) this->v->step(stepSize);
    
    this->badCollision = false;
    this->contacts.clear();
    
    for(size_t i = 0; i < simulationStepsPerFrame; i++) {
        // find collisions and add contact joints
        dSpaceCollide(this->space, this, &nearCallbackWrapper);
        // step the simulation
        dWorldQuickStep(this->world, stepSize / (dReal)simulationStepsPerFrame);
        // remove all contact joints
        dJointGroupEmpty(this->contactGroup);
    }
    
    return this->badCollision;
}

bool Environment::step() {
    return step(config.step.step_size, config.step.simulation_steps_per_frame);
}

static void evaluateCollisionNearCallbackWrapper(void *data, dGeomID o1, dGeomID o2) {
    reinterpret_cast<Environment *>(data)->evaluateCollisionNearCallback(o1, o2);
}

void Environment::evaluateCollisionNearCallback(dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
    dContact contact[1];  // one contact is sufficient
    int numc = dCollide(o1, o2, 1, &contact[0].geom, sizeof(dContact));
    // flag collision if there is really a collision and is of those not allowed
    if(numc > 0 && !isValidCollision(o1, o2, contact[0])) {
        badCollision = true;
        std::cout << "collision between " << getGeomName(o1) << "[" << dClassGetName(dGeomGetClass(o1)) << "] and " << getGeomName(o2) << "[" << dClassGetName(dGeomGetClass(o2)) << "]" << std::endl;
    }
}

bool Environment::evaluateCollision() {
    this->badCollision = false;
    dSpaceCollide(this->space, this, &evaluateCollisionNearCallbackWrapper);
    return this->badCollision;
}

void Environment::draw() {
    if(this->v) this->v->draw();

    if(config.show_contact_points) {
        dsSetColor(1, 0, 1);
        dMatrix3 R; dRSetIdentity(R);
        BOOST_FOREACH(dContactGeom cg, this->contacts) {
            dsDrawSphereD(cg.pos, R, 0.05);
        }
    }
    
    for(std::vector<dGeomID>::iterator it = boxes.begin(); it != boxes.end(); it++) {
        dsSetColorAlpha(0.6, 0.6, 0.7, 0.8);
        const dReal *pos = dGeomGetPosition(*it);
        const dReal *R = dGeomGetRotation(*it);
        dReal sides[3];
        dGeomBoxGetLengths(*it, sides);
        dsDrawBoxD(pos, R, sides);
    }
}

