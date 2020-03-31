//
// Created by kirill on 30.03.2020.
//

#include "Stabilizer.h"
#include "Drone.h"

double Stabilizer::computePwm(Drone* drone, double currentAltitude, double dt) {
    double thrust = _hoverPID.calculate(dt, _targetAltitude, currentAltitude);
}

void Stabilizer::setTargetParameters(double targetAltitude) {
    _targetAltitude = targetAltitude;
}

Stabilizer::Stabilizer(PID hoverPID) : _hoverPID(hoverPID) {}
