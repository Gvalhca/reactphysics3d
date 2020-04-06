
#include "DroneModule.h"

namespace drone {

    
    DroneModule::DroneModule(double mass, const rp3d::Transform& defaultTransform,
                                           rp3d::DynamicsWorld* dynamicsWorld,
                                           const std::string& meshFolderPath, double radius,
                                           const openglframework::Color& color) :
            _physicsBody(new Sphere(radius, mass, dynamicsWorld, meshFolderPath)),
            _defaultTransform(defaultTransform) {

        _physicsBody->setTransform(defaultTransform);

        this->getPhysicsBody()->setColor(color);
        auto mRedColor = openglframework::Color(0.95f, 0, 0, 1.0f);
        this->getPhysicsBody()->setSleepingColor(mRedColor);

        rp3d::Material& material = getPhysicsBody()->getRigidBody()->getMaterial();
        material.setBounciness(rp3d::decimal(0.4));
    }

    
    rp3d::Transform DroneModule::getDefaultTransform() const {
        return _defaultTransform;
    }

    
    DroneModule::~DroneModule() {
        delete _physicsBody;
    }

//    Box* DroneModule::getPhysicsBody() const {
//        return _physicsBody;
//    }

    
    PhysicsObject* DroneModule::getPhysicsBody() const {
        return _physicsBody;
    }
    
    DroneModule::DroneModule(double mass, const rp3d::Transform& defaultTransform,
                             rp3d::DynamicsWorld* dynamicsWorld, const std::string& meshFolderPath,
                             const openglframework::Vector3& size, const openglframework::Color& color) :
            _physicsBody(new Box(size, mass, dynamicsWorld, meshFolderPath)),
            _defaultTransform(defaultTransform) {

    }
}