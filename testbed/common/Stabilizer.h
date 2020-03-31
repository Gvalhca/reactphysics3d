//
// Created by kirill on 30.03.2020.
//

#ifndef REACTPHYSICS3D_STABILIZER_H
#define REACTPHYSICS3D_STABILIZER_H


#include "pid.h"
#include "PhysicsObject.h"
#include "Motor.h"
#include "Drone.h"

namespace drone {
    class Drone::Stabilizer {
    private:
        PID _hoverPID;
        double _targetAltitude;

    public:
        Stabilizer(const PID& hoverPID) : _hoverPID(hoverPID) {};

        double computePwm(const Drone& drone, double dt);

        inline void setTargetParameters(double targetAltitude) {
            _targetAltitude = targetAltitude;
        };

        ~Stabilizer() = default;
    };
}

#endif //REACTPHYSICS3D_STABILIZER_H
