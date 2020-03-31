
#include "DroneModule.h"

namespace drone {

    DroneModule::DroneModule(double mass, const openglframework::Color& color,
                             const rp3d::Transform& defaultTransform, rp3d::DynamicsWorld* dynamicsWorld,
                             const std::string& meshFolderPath) :
            _physicsBody(new Sphere(0.01, mass, dynamicsWorld, meshFolderPath)),
            defaultTransform(defaultTransform) {

        _physicsBody->setTransform(defaultTransform);

        this->getPhysicsBody()->setColor(color);
        auto mRedColor = openglframework::Color(0.95f, 0, 0, 1.0f);
        this->getPhysicsBody()->setSleepingColor(mRedColor);

        rp3d::Material& material = getPhysicsBody()->getRigidBody()->getMaterial();
        material.setBounciness(rp3d::decimal(0.4));
    }

    rp3d::Transform DroneModule::getDefaultTransform() const {
        return defaultTransform;
    }

    DroneModule::~DroneModule() {
        delete _physicsBody;
    }

    Sphere* DroneModule::getPhysicsBody() const {
        return _physicsBody;
    }
}