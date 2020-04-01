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
        std::vector<PID*> _pids;
        double _targetAltitude;

        double computeHoverMode(const Drone& drone, double dt);
    public:
        typedef enum {
            HOVER_PID = 0,
            PITCH_PID = 1,
            ROLL_PID = 2,
            YAW_PID = 3
        } pid_names;

        Stabilizer(const PID& hoverPID, const PID& pitchPID, const PID& rollPID, const PID& yawPID);

        double computePwm(const Drone& drone, double dt);

        void setTargetParameters(double targetAltitude);

        void reset();

        ~Stabilizer() = default;
    };
}

#endif //REACTPHYSICS3D_STABILIZER_H
