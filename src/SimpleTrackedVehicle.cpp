//
// Created by peci1 on 6.1.16.
//

#include "SimpleTrackedVehicle.h"

SimpleTrackedVehicle::SimpleTrackedVehicle(const std::string& name_, dReal xOffset, dReal yOffset, dReal zOffset) : Vehicle(name_) {

}

SimpleTrackedVehicle::~SimpleTrackedVehicle() {

}

void SimpleTrackedVehicle::create(Environment *environment) {

}

void SimpleTrackedVehicle::destroy() {

}

void SimpleTrackedVehicle::step(dReal stepSize) {
    Vehicle::step(stepSize);
}

void SimpleTrackedVehicle::draw() {

}

void SimpleTrackedVehicle::setVelocities(dReal a, dReal b) {

}

void SimpleTrackedVehicle::setWheelVelocities(dReal left, dReal right) {

}

const dReal *SimpleTrackedVehicle::getPosition() {
    return NULL;
}

const dReal *SimpleTrackedVehicle::getLinearVel() {
    return NULL;
}

const dReal *SimpleTrackedVehicle::getAngularVel() {
    return NULL;
}

const dReal *SimpleTrackedVehicle::getQuaternion() {
    return NULL;
}

const dReal *SimpleTrackedVehicle::getRotation() {
    return NULL;
}

void SimpleTrackedVehicle::setPosition(const dReal *p) {

}

void SimpleTrackedVehicle::setVel(const dReal *linear, const dReal *angular) {

}

void SimpleTrackedVehicle::setQuaternion(const dReal *q) {

}

void SimpleTrackedVehicle::setRotation(const dReal *R) {

}
