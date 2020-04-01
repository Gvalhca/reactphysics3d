
#include "Stabilizer.h"

namespace drone {

    double Drone::Stabilizer::computePwm(const Drone& drone, double dt) {
        double addThrust = computeHoverMode(drone, dt);
        for (const auto& motor : drone.getMotors()) {
            motor->setPwm(addThrust);
        }
    }

    double Drone::Stabilizer::computeHoverMode(const Drone& drone, double dt) {
        double currentAltitude = drone.getAltitude();
        double thrust = _pids[HOVER_PID]->calculate(dt, _targetAltitude, currentAltitude);
        return thrust;
    }

    void Drone::Stabilizer::setTargetParameters(double targetAltitude) {
        _targetAltitude = targetAltitude;
    }

    void Drone::Stabilizer::reset() {
        for (const auto& pid : _pids) {
            pid->reset();
        }
        _targetAltitude = 0;
    }

    Drone::Stabilizer::Stabilizer(const PID& hoverPID, const PID& pitchPID, const PID& rollPID, const PID& yawPID) {
        _pids.push_back(new PID(hoverPID));
        _pids.push_back(new PID(pitchPID));
        _pids.push_back(new PID(rollPID));
        _pids.push_back(new PID(yawPID));
    }



}