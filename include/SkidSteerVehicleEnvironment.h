//
// Created by peci1 on 6.1.16.
//

#ifndef TRACKED_MOTION_SKIDSTEERVEHICLEENVIRONMENT_H
#define TRACKED_MOTION_SKIDSTEERVEHICLEENVIRONMENT_H


#include <ode/ode.h>
#include "Environment.h"

class SkidSteerVehicleEnvironment : public Environment {

public:
    SkidSteerVehicleEnvironment();
    virtual ~SkidSteerVehicleEnvironment();

    bool isValidCollision(dGeomID o1, dGeomID o2, const dContact& contact);

    void nearCallback(dGeomID o1, dGeomID o2);

protected:
    void nearCallbackGrouserTerrain(dGeomID o1, dGeomID o2);

};


#endif //TRACKED_MOTION_SKIDSTEERVEHICLEENVIRONMENT_H
