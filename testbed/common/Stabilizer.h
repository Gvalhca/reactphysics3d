//
// Created by kirill on 30.03.2020.
//

#ifndef REACTPHYSICS3D_STABILIZER_H
#define REACTPHYSICS3D_STABILIZER_H


#include <pid.h>

class Stabilizer {
private:
    PID hoverPID{0.7, 0.35, 0.35};
    double _targetAlt;

public:
    Stabilizer();
    double getThrottle(double currentAlt, double targetAlt, double dt)
    ~Stabilizer();
};


#endif //REACTPHYSICS3D_STABILIZER_H
