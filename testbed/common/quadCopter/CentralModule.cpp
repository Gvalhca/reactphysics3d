//
// Created by kirill on 03.04.2020.
//

#include "CentralModule.h"
#include "Barometer.h"
#include "Gyroscope.h"

namespace quad {
    CentralModule::CentralModule(double mass, const openglframework::Vector3& size, const rp3d::Transform& defaultTransform,
                                 const QuadPIDs& quadPIDs, rp3d::DynamicsWorld* dynamicsWorld,
                                 const std::string& meshFolderPath) :
            DroneModule(mass, defaultTransform, dynamicsWorld, meshFolderPath, size),
            _stabilizer(new Stabilizer(quadPIDs, _physicsBody)) {}


    CentralModule::~CentralModule() = default;

}