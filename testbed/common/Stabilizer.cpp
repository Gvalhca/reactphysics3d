
#include "Stabilizer.h"

namespace drone {

    double Drone::Stabilizer::computePwm(const Drone& drone, double dt) {
        double addThrust = _hoverPID.calculate(dt, _targetAltitude, drone.getAltitude());
        for (const auto& motor : drone.getMotors()) {
            double thrust = motor->getPwm() + addThrust;
            motor->setPwm(thrust);
        }
    }

}