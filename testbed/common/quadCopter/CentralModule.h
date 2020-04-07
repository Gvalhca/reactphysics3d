//
// Created by kirill on 03.04.2020.
//

#ifndef REACTPHYSICS3D_CENTRALMODULE_H
#define REACTPHYSICS3D_CENTRALMODULE_H

#include "quadCopter/DroneModule.h"
#include "Stabilizer.h"
#include "Barometer.h"
#include "Gyroscope.h"


namespace drone {

    class CentralModule : public DroneModule {
    public:

        CentralModule(double mass, const openglframework::Vector3& size, const rp3d::Transform& defaultTransform,
                      const QuadPIDs& quadPIDs, rp3d::DynamicsWorld* dynamicsWorld, const std::string& meshFolderPath);

        ~CentralModule();

        Stabilizer* _stabilizer;
    };
}

#endif //REACTPHYSICS3D_CENTRALMODULE_H
