//
// Created by kirill on 25.03.2020.
//

#include "Drone.h"

#define physBody droneModule->getPhysicsBody()

Drone::Drone(float frameSize, float droneMass, float motorRadius, float motorMass, rp3d::DynamicsWorld* world,
             const std::string& meshFolderPath) {

    // --------------- Create the central sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionCentralSphere(0, 0, 0);

    // Create a sphere and a corresponding rigid in the dynamics world
    float centerMass = droneMass - 4 * motorMass;
    assert(centerMass > 0);

    DroneModule* mCentralSphere = new DroneModule(0.1, centerMass, positionCentralSphere, world, meshFolderPath);
    droneModules.push_back(mCentralSphere);

    // --------------- Create the top sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionTopSphere(0, 0, frameSize / 2);

    // Create a sphere and a corresponding rigid in the dynamics world
    DroneModule* mTopSphere = new DroneModule(motorRadius, motorMass, positionTopSphere, world, meshFolderPath);
    droneModules.push_back(mTopSphere);

    // --------------- Create the bottom sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionBottomSphere(0, 0, -frameSize / 2);

    // Create a sphere and a corresponding rigid in the dynamics world
    DroneModule* mBottomSphere = new DroneModule(motorRadius, motorMass, positionBottomSphere, world, meshFolderPath);
    droneModules.push_back(mBottomSphere);

    // --------------- Create the left sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionLeftSphere(-frameSize / 2, 0, 0);

    // Create a sphere and a corresponding rigid in the dynamics world
    DroneModule* mLeftSphere = new DroneModule(motorRadius, motorMass, positionLeftSphere, world, meshFolderPath);
    droneModules.push_back(mLeftSphere);

    // --------------- Create the right sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionRightSphere(frameSize / 2, 0, 0);

    // Create a sphere and a corresponding rigid in the dynamics world
    DroneModule* mRightSphere = new DroneModule(motorRadius, motorMass, positionRightSphere, world, meshFolderPath);
    droneModules.push_back(mRightSphere);

    // --------------- Create the central top fixed joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* topSphereBody = mTopSphere->getPhysicsBody()->getRigidBody();
    rp3d::RigidBody* centralSphereBody = mCentralSphere->getPhysicsBody()->getRigidBody();
    const rp3d::Vector3& anchorPointWorldSpace1(positionCentralSphere);
    rp3d::FixedJointInfo jointInfoTop(topSphereBody, centralSphereBody, anchorPointWorldSpace1);
    jointInfoTop.isCollisionEnabled = false;

    // Create the joint in the dynamics world
    fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(jointInfoTop)));

    // --------------- Create the central bottom fixed joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* bottomSphereBody = mBottomSphere->getPhysicsBody()->getRigidBody();
    rp3d::FixedJointInfo jointInfoBottom(bottomSphereBody, centralSphereBody, anchorPointWorldSpace1);
    jointInfoBottom.isCollisionEnabled = false;

    // Create the joint in the dynamics world
    fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(jointInfoBottom)));

    // --------------- Create the central left fixed joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* leftSphereBody = mLeftSphere->getPhysicsBody()->getRigidBody();
    rp3d::FixedJointInfo jointInfoLeft(leftSphereBody, centralSphereBody, anchorPointWorldSpace1);
    jointInfoLeft.isCollisionEnabled = false;

    // Create the joint in the dynamics world
    fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(jointInfoLeft)));

    // --------------- Create the central right fixed joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* rightSphereBody = mRightSphere->getPhysicsBody()->getRigidBody();
    rp3d::FixedJointInfo jointInfoRight(rightSphereBody, centralSphereBody, anchorPointWorldSpace1);
    jointInfoRight.isCollisionEnabled = false;

    // Create the joint in the dynamics world
    fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(jointInfoRight)));

}

void Drone::render(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {
    for (auto& droneModule : droneModules) {
        physBody->render(shader, worldToCameraMatrix);
    }
}

///TODO: First apply defaultTransform() to every droneModule object, Then apply setTransform() to them
void Drone::setTransform(const rp3d::Transform& transform) {
    for (auto& droneModule : droneModules) {
        rp3d::Transform defaultTransform = rp3d::Transform(droneModule->getDefaultPosition(), rp3d::Quaternion::identity());
        physBody->setTransform(transform * defaultTransform);
    }
}

Drone::~Drone() = default;


Drone::Motor::Motor(float propellerRadius, float mass, const rp3d::Vector3& defaultPosition,
                    rp3d::DynamicsWorld* dynamicsWorld,
                    const std::string& meshFolderPath) : DroneModule(propellerRadius, mass, defaultPosition,
                                                                     dynamicsWorld,
                                                                     meshFolderPath) {}

Drone::DroneModule::DroneModule(float radius, float mass, const rp3d::Vector3& defaultPosition,
                                rp3d::DynamicsWorld* dynamicsWorld, const std::string& meshFolderPath) :
        physicsBody(new Sphere(radius, mass, dynamicsWorld, meshFolderPath)),
        defaultPosition(defaultPosition) {
    auto mYellowColor = openglframework::Color(0.9f, 0.88f, 0.145f, 1.0f);
    auto mRedColor = openglframework::Color(0.95f, 0, 0, 1.0f);
    initDroneModule(this, mYellowColor, mRedColor);
}

rp3d::Vector3 Drone::DroneModule::getDefaultPosition() const {
    return this->defaultPosition;
}

Drone::DroneModule::~DroneModule() {
    delete physicsBody;
}

Drone::DroneModule* Drone::DroneModule::initDroneModule(Drone::DroneModule* droneModule, const openglframework::Color& color,
                                                      const openglframework::Color& sleepingColor) {

    physBody->setTransform(rp3d::Transform(droneModule->getDefaultPosition(), rp3d::Quaternion::identity()));

    physBody->setColor(color);
    physBody->setSleepingColor(sleepingColor);

    rp3d::Material& materialSphere = physBody->getRigidBody()->getMaterial();
    materialSphere.setBounciness(rp3d::decimal(0.4));

    return droneModule;
}

Sphere* Drone::DroneModule::getPhysicsBody() const {
    return this->physicsBody;
}
