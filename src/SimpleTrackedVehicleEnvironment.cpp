//
// Created by peci1 on 6.1.16.
//

#include "SimpleTrackedVehicleEnvironment.h"
#include "SimpleTrackedVehicle.h"

SimpleTrackedVehicleEnvironment::SimpleTrackedVehicleEnvironment() {
    SimpleTrackedVehicle *ssv = new SimpleTrackedVehicle("robot", 1, -2, 0.301);
    //ssv->velocityLeft.setSlope(config.world.track_acceleration);
    //ssv->velocityRight.setSlope(config.world.track_acceleration);
    this->v = ssv;
}

SimpleTrackedVehicleEnvironment::~SimpleTrackedVehicleEnvironment() {

}

bool SimpleTrackedVehicleEnvironment::isValidCollision(dGeomID o1, dGeomID o2, const dContact& contact) {
    if(isCatPair(Category::TRACK_GROUSER, Category::TERRAIN, &o1, &o2))
        return true;
    return false;
}

void SimpleTrackedVehicleEnvironment::nearCallback(dGeomID o1, dGeomID o2){
//    if(isCatPair(Category::TRACK_GROUSER, Category::TERRAIN, &o1, &o2))
//        this->nearCallbackGrouserTerrain(o1, o2);
//    else
        nearCallbackDefault(o1, o2);
}