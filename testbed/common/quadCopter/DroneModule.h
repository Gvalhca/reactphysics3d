
#ifndef REACTPHYSICS3D_DRONEMODULE_H
#define REACTPHYSICS3D_DRONEMODULE_H

#include "reactphysics3d.h"
#include "openglframework.h"
#include "Sphere.h"
#include "Box.h"

namespace quad {

    class DroneModule {
    protected:
        std::shared_ptr<PhysicsObject> _physicsBody;
        rp3d::Transform _defaultTransform;

    public:
        DroneModule(double mass,
                    const rp3d::Transform& defaultTransform, rp3d::DynamicsWorld* dynamicsWorld,
                    const std::string& meshFolderPath,
                    double radius = 0.01,
                    const openglframework::Color& color = openglframework::Color(0.9f, 0.88f, 0.145f, 1.0f));

        DroneModule(double mass,
                    const rp3d::Transform& defaultTransform, rp3d::DynamicsWorld* dynamicsWorld,
                    const std::string& meshFolderPath,
                    const openglframework::Vector3& size,
                    const openglframework::Color& color = openglframework::Color(0.9f, 0.88f, 0.145f, 1.0f));

        rp3d::Transform getDefaultTransform() const;

        virtual ~DroneModule();

        std::shared_ptr<PhysicsObject> getPhysicsBody() const;
    };
}


#endif //REACTPHYSICS3D_DRONEMODULE_H
