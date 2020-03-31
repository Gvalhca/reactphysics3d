//
// Created by kirill on 30.03.2020.
//

#ifndef REACTPHYSICS3D_STABILIZER_H
#define REACTPHYSICS3D_STABILIZER_H


#include <pid.h>

class Stabilizer {
private:
    PID _hoverPID;
    double _targetAltitude;
public:
    Stabilizer(PID hoverPID);
    double computePwm(double currentAltitude, double dt);
    void setTargetParameters(double targetAltitude);
    ~Stabilizer();
};


#endif //REACTPHYSICS3D_STABILIZER_H
