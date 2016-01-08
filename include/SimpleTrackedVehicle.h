//
// Created by peci1 on 6.1.16.
//

#ifndef TRACKED_MOTION_SIMPLETRACKEDVEHICLE_H
#define TRACKED_MOTION_SIMPLETRACKEDVEHICLE_H

#include "Vehicle.h"

class SimpleTrackedVehicle : public Vehicle {

public:
    SimpleTrackedVehicle(const std::string& name_, dReal xOffset, dReal yOffset, dReal zOffset);
    virtual ~SimpleTrackedVehicle();

    void create(Environment *environment);
    void destroy();
    void step(dReal stepSize);
    void draw();
    void setVelocities(dReal a, dReal b);
    void setWheelVelocities(dReal left, dReal right);
    const dReal * getPosition();
    const dReal * getLinearVel();
    const dReal * getAngularVel();
    const dReal * getQuaternion();
    const dReal * getRotation();
    void setPosition(const dReal *p);
    void setVel(const dReal *linear, const dReal *angular);
    void setQuaternion(const dReal *q);
    void setRotation(const dReal *R);
};

#endif //TRACKED_MOTION_SIMPLETRACKEDVEHICLE_H
