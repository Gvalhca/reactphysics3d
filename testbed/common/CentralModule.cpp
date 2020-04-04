//
// Created by kirill on 03.04.2020.
//

#include "CentralModule.h"
#include "Barometer.h"
#include "Gyroscope.h"

namespace drone {
    CentralModule::CentralModule(double mass,
                                 const rp3d::Transform& defaultTransform,
                                 const QuadPIDs& quadPIDs,
                                 rp3d::DynamicsWorld* dynamicsWorld,
                                 const std::string& meshFolderPath) :
            DroneModule(mass, defaultTransform, dynamicsWorld, meshFolderPath),
            _stabilizer(new Stabilizer(quadPIDs, _physicsBody)) {}


    CentralModule::~CentralModule() {
        delete _stabilizer;
    }

}