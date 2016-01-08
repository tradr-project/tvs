//
// Created by peci1 on 6.1.16.
//

#ifndef TRACKED_MOTION_TRACKEDVEHICLEENVIRONMENT_H
#define TRACKED_MOTION_TRACKEDVEHICLEENVIRONMENT_H


#include <ode/ode.h>
#include "Environment.h"

class TrackedVehicleEnvironment : public Environment {

public:
    TrackedVehicleEnvironment();
    virtual ~TrackedVehicleEnvironment();

    bool isValidCollision(dGeomID o1, dGeomID o2, const dContact& contact);

    void nearCallback(dGeomID o1, dGeomID o2);

protected:

    void nearCallbackWheelGrouser(dGeomID o1, dGeomID o2);

    void nearCallbackGrouserTerrain(dGeomID o1, dGeomID o2);

    void nearCallbackGrouserGuide(dGeomID o1, dGeomID o2);
};


#endif //TRACKED_MOTION_TRACKEDVEHICLEENVIRONMENT_H
