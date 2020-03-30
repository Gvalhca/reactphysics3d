//
// Created by kirill on 30.03.2020.
//

#include "Stabilizer.h"

double Stabilizer::getThrottle(double currentAlt, double targetAlt, double dt) {
    _targetAlt = targetAlt;
    return hoverPID.calculate(dt, targetAlt,currentAlt);
}
