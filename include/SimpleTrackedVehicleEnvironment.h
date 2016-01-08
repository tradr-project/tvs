//
// Created by peci1 on 6.1.16.
//

#ifndef TRACKED_MOTION_SIMPLETRACKEDVEHICLEENVIRONMENT_H
#define TRACKED_MOTION_SIMPLETRACKEDVEHICLEENVIRONMENT_H

#include "Environment.h"

class SimpleTrackedVehicleEnvironment : public Environment {

public:
    SimpleTrackedVehicleEnvironment();
    virtual ~SimpleTrackedVehicleEnvironment();

    bool isValidCollision(dGeomID o1, dGeomID o2, const dContact& contact);

    void nearCallback(dGeomID o1, dGeomID o2);

};

#endif //TRACKED_MOTION_SIMPLETRACKEDVEHICLEENVIRONMENT_H
