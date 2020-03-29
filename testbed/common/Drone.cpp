//
// Created by kirill on 25.03.2020.
//

#include "Drone.h"

#define physBody droneModule->getPhysicsBody()

Drone::Drone(float frameSize, float droneMass, float motorRadius, float motorMass, rp3d::DynamicsWorld* world,
             const std::string& meshFolderPath) {
    double arm = frameSize / 2;
    openglframework::Color mBlueColor = openglframework::Color(0, 0.66f, 0.95f, 1.0f);
    openglframework::Color mGreenColor = openglframework::Color(0.0f, 0.95f, 0.3f, 1.0f);
    openglframework::Color mYellowColor = openglframework::Color(0.9f, 0.88f, 0.145f, 1.0f);
    // --------------- Create the central sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionCentralSphere(0, 0, 0);
    rp3d::Transform transformCentralModule(positionCentralSphere, rp3d::Quaternion::identity());
    // Create a sphere and a corresponding rigid in the dynamics world
    float centerMass = droneMass - 4 * motorMass;
    assert(centerMass > 0);

    auto* mCentralSphere = new DroneModule(centerMass, mYellowColor, transformCentralModule, world, meshFolderPath);
    droneModules.push_back(mCentralSphere);

    // --------------- Create the top sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionTopSphere = rp3d::Vector3(1, 0, 1) * arm;
    rp3d::Transform transformTopModule(positionTopSphere, rp3d::Quaternion::identity());

    // Create a sphere and a corresponding rigid in the dynamics world
    auto* mTopSphere = new Motor(motorRadius, motorMass, mBlueColor, transformTopModule, world, meshFolderPath);
    droneModules.push_back(mTopSphere);

    // --------------- Create the bottom sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionBottomSphere = rp3d::Vector3(-1, 0, -1) * arm;
    rp3d::Transform transformBottomModule(positionBottomSphere, rp3d::Quaternion::identity());

    // Create a sphere and a corresponding rigid in the dynamics world
    auto* mBottomSphere = new Motor(motorRadius, motorMass, mGreenColor, transformBottomModule, world, meshFolderPath);
    droneModules.push_back(mBottomSphere);

    // --------------- Create the left sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionLeftSphere = rp3d::Vector3(1, 0, -1) * arm;
    rp3d::Transform transformLeftModule(positionLeftSphere, rp3d::Quaternion::identity());

    // Create a sphere and a corresponding rigid in the dynamics world
    auto* mLeftSphere = new Motor(motorRadius, motorMass, mBlueColor, transformLeftModule, world, meshFolderPath);
    droneModules.push_back(mLeftSphere);

    // --------------- Create the right sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionRightSphere = rp3d::Vector3(-1, 0, 1) * arm;
    rp3d::Transform transformRightModule(positionRightSphere, rp3d::Quaternion::identity());

    // Create a sphere and a corresponding rigid in the dynamics world
    auto* mRightSphere = new Motor(motorRadius, motorMass, mGreenColor, transformRightModule, world, meshFolderPath);
    droneModules.push_back(mRightSphere);

#if 0
    ///TODO: initFixedJoint()
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
#endif
}

void Drone::render(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {
    for (auto& droneModule : droneModules) {
        physBody->render(shader, worldToCameraMatrix);
    }
}

void Drone::setTransform(const rp3d::Transform& transform) {
    for (auto& droneModule : droneModules) {
        rp3d::Transform defaultTransform = droneModule->getDefaultTransform();
        physBody->setTransform(transform * defaultTransform);
    }
}

Drone::~Drone() = default;


Drone::Motor::Motor(float propellerRadius, float mass, const openglframework::Color& color,
                    const rp3d::Transform& defaultTransform, rp3d::DynamicsWorld* dynamicsWorld,
                    const std::string& meshFolderPath) : DroneModule(mass, color, defaultTransform, dynamicsWorld, meshFolderPath) {}

Drone::DroneModule::DroneModule(float mass, const openglframework::Color& color,
                                const rp3d::Transform& defaultTransform, rp3d::DynamicsWorld* dynamicsWorld,
                                const std::string& meshFolderPath) :
        physicsBody(new Sphere(0.1, mass, dynamicsWorld, meshFolderPath)),
        defaultTransform(defaultTransform) {

    initDroneModule(this, color);
}

rp3d::Transform Drone::DroneModule::getDefaultTransform() const {
    return this->defaultTransform;
}

Drone::DroneModule::~DroneModule() {
    delete physicsBody;
}

Drone::DroneModule*
Drone::DroneModule::initDroneModule(Drone::DroneModule* droneModule, const openglframework::Color& color) {

    physBody->setTransform(
            rp3d::Transform(droneModule->getDefaultTransform().getPosition(), rp3d::Quaternion::identity()));

    physBody->setColor(color);
    auto mRedColor = openglframework::Color(0.95f, 0, 0, 1.0f);
    physBody->setSleepingColor(mRedColor);

    rp3d::Material& materialSphere = physBody->getRigidBody()->getMaterial();
    materialSphere.setBounciness(rp3d::decimal(0.4));

    return droneModule;
}

Sphere* Drone::DroneModule::getPhysicsBody() const {
    return this->physicsBody;
}

