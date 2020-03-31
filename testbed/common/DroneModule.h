
#ifndef REACTPHYSICS3D_DRONEMODULE_H
#define REACTPHYSICS3D_DRONEMODULE_H

#include "reactphysics3d.h"
#include "openglframework.h"
#include "Sphere.h"

namespace drone {

    class DroneModule {
    protected:
        Sphere* _physicsBody;
        rp3d::Transform defaultTransform;

    public:
        DroneModule(double mass, const openglframework::Color& color, const rp3d::Transform& defaultTransform,
                    rp3d::DynamicsWorld* dynamicsWorld,
                    const std::string& meshFolderPath);

        rp3d::Transform getDefaultTransform() const;

        ~DroneModule();

        Sphere* getPhysicsBody() const;
    };
}


#endif //REACTPHYSICS3D_DRONEMODULE_H
