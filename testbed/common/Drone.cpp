//
// Created by kirill on 25.03.2020.
//

#include "Drone.h"

Drone::Drone(double frameSize, double droneMass, double motorRadius, double motorMass, rp3d::DynamicsWorld* world,
             const std::string& meshFolderPath) : _mass(droneMass) {
    PID hoverPID(0.75, 0.35, 0.35);
    _stabilizer = new Stabilizer(hoverPID);

    double arm = frameSize / 2;
    openglframework::Color mBlueColor = openglframework::Color(0, 0.66f, 0.95f, 1.0f);
    openglframework::Color mGreenColor = openglframework::Color(0.0f, 0.95f, 0.3f, 1.0f);
    openglframework::Color mYellowColor = openglframework::Color(0.9f, 0.88f, 0.145f, 1.0f);
    openglframework::Color mOrangeColor = openglframework::Color(0.9f, 0.35f, 0, 1.0f);

    // --------------- Create the central module --------------- //

    // Position of the module
    rp3d::Vector3 centralModulePosition(0, 0, 0);
    rp3d::Transform centralModuleTransform(centralModulePosition, rp3d::Quaternion::identity());

    // Create a module and a corresponding rigid in the dynamics world
    double centerMass = droneMass - 4 * motorMass;
    assert(centerMass > 0);
    auto* centralModule = new DroneModule(centerMass, mYellowColor, centralModuleTransform, world, meshFolderPath);
    droneModules.push_back(centralModule);


    // Creating motors
    /// Set maxpwm for motor to lift mass in liftCapability times more than droneMass
    int liftCapability = 3;
    double maxpwm = (droneMass * 9.81) * liftCapability / 4;
    // --------------- Create the front right motor --------------- //

    // Position of the motor
    rp3d::Vector3 motorPosition = rp3d::Vector3(1, 0, 1) * arm;
    rp3d::Transform motorTransform(motorPosition, rp3d::Quaternion::identity());

    // Create a sphere and a corresponding rigid in the dynamics world
    auto* motorFR = new Motor(motorRadius, motorMass, maxpwm, mBlueColor, motorTransform, world, meshFolderPath);
    motors.push_back(motorFR);

    // --------------- Create the front left motor --------------- //

    // Position of the motor
    motorPosition = rp3d::Vector3(-1, 0, 1) * arm;
    motorTransform = rp3d::Transform(motorPosition, rp3d::Quaternion::identity());

    // Create a motor and a corresponding rigid in the dynamics world
    auto* motorFL = new Motor(motorRadius, motorMass, maxpwm, mBlueColor, motorTransform, world, meshFolderPath);
    motors.push_back(motorFL);

    // --------------- Create the back right motor --------------- //

    // Position of the motor
    motorPosition = rp3d::Vector3(1, 0, -1) * arm;
    motorTransform = rp3d::Transform(motorPosition, rp3d::Quaternion::identity());

    // Create a motor and a corresponding rigid in the dynamics world
    auto* motorBR = new Motor(motorRadius, motorMass, maxpwm, mOrangeColor, motorTransform, world,
                              meshFolderPath);
    motors.push_back(motorBR);

    // --------------- Create the back left motor --------------- //

    // Position of the motor
    motorPosition = rp3d::Vector3(-1, 0, -1) * arm;
    motorTransform = rp3d::Transform(motorPosition, rp3d::Quaternion::identity());

    // Create a motor and a corresponding rigid in the dynamics world
    auto* motorBL = new Motor(motorRadius, motorMass, maxpwm, mOrangeColor, motorTransform, world,
                              meshFolderPath);
    motors.push_back(motorBL);

    // Add motors to droneModules
    droneModules.insert(droneModules.end(), motors.begin(), motors.end());

    // --------------- Create the central top fixed joint --------------- //
    // Create the joint in the dynamics world
    fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
            generateFrameInfo(motorFR, centralModule, centralModulePosition))));

    // --------------- Create the central bottom fixed joint --------------- //
    // Create the joint in the dynamics world
    fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
            generateFrameInfo(motorBL, centralModule, centralModulePosition))));

    // --------------- Create the central left fixed joint --------------- //
    // Create the joint in the dynamics world
    fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
            generateFrameInfo(motorBR, centralModule, centralModulePosition))));

    // --------------- Create the central right fixed joint --------------- //
    // Create the joint in the dynamics world
    fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
            generateFrameInfo(motorFL, centralModule, centralModulePosition))));
}

#define physBody droneModule->getPhysicsBody()

void Drone::render(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {
    for (auto& droneModule : droneModules) {
        physBody->render(shader, worldToCameraMatrix);
    }
}

void Drone::setTransform(const rp3d::Transform& transform) {
    for (auto& droneModule : droneModules) {
        rp3d::Transform defaultTransform = droneModule->getDefaultTransform();
        rp3d::Transform modifiedTransform = transform * defaultTransform;
        //_altitude = modifiedTransform.getPosition().y;
        physBody->setTransform(modifiedTransform);
    }
}

rp3d::FixedJointInfo Drone::generateFrameInfo(Drone::DroneModule* firstModule, Drone::DroneModule* secondModule,
                                              const rp3d::Vector3& anchorPoint) {
    rp3d::RigidBody* firstModuleBody = firstModule->getPhysicsBody()->getRigidBody();
    rp3d::RigidBody* secondModuleBody = secondModule->getPhysicsBody()->getRigidBody();
    rp3d::FixedJointInfo jointInfoTop(firstModuleBody, secondModuleBody, anchorPoint);
    jointInfoTop.isCollisionEnabled = false;
    return jointInfoTop;
}

void Drone::setMotorsMaxPwm(double maxpwm) {
    for (auto& motor : motors) {
        motor->setMaxPwm(maxpwm);
    }
}

void Drone::updatePhysics() {
    for (auto& motor : motors) {
        motor->updatePhysics();
    }
}

Drone::~Drone() {
    delete _stabilizer;
};


Drone::Motor::Motor(double propellerRadius, double mass, double maxpwm, const openglframework::Color& color,
                    const rp3d::Transform& defaultTransform, rp3d::DynamicsWorld* dynamicsWorld,
                    const std::string& meshFolderPath) : DroneModule(mass, color, defaultTransform, dynamicsWorld,
                                                                     meshFolderPath), _maxpwm(maxpwm) {}

Drone::DroneModule::DroneModule(double mass, const openglframework::Color& color,
                                const rp3d::Transform& defaultTransform, rp3d::DynamicsWorld* dynamicsWorld,
                                const std::string& meshFolderPath) :
        _physicsBody(new Sphere(0.2, mass, dynamicsWorld, meshFolderPath)),
        defaultTransform(defaultTransform) {

    _physicsBody->setTransform(defaultTransform);

    this->getPhysicsBody()->setColor(color);
    auto mRedColor = openglframework::Color(0.95f, 0, 0, 1.0f);
    this->getPhysicsBody()->setSleepingColor(mRedColor);

    rp3d::Material& material = getPhysicsBody()->getRigidBody()->getMaterial();
    material.setBounciness(rp3d::decimal(0.4));
}

rp3d::Transform Drone::DroneModule::getDefaultTransform() const {
    return defaultTransform;
}

Drone::DroneModule::~DroneModule() {
    delete _physicsBody;
}

Sphere* Drone::DroneModule::getPhysicsBody() const {
    return _physicsBody;
}

