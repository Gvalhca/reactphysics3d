//
// Created by kirill on 25.03.2020.
//

#include "Drone.h"

#define physBody dronePart->getPhysicsBody()

Drone::Drone(float frameSize, float droneMass, float motorRadius, float motorMass, rp3d::DynamicsWorld* world,
             const std::string& meshFolderPath) {

    // --------------- Create the central sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionCentralSphere(0, 0, 0);

    // Create a sphere and a corresponding rigid in the dynamics world
    float centerMass = droneMass - 4 * motorMass;
    assert(centerMass > 0);

    DronePart* mCentralSphere = new DronePart(0.1, centerMass, positionCentralSphere, world, meshFolderPath);
    droneParts.push_back(mCentralSphere);

    // --------------- Create the top sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionTopSphere(0, 0, frameSize / 2);

    // Create a sphere and a corresponding rigid in the dynamics world
    DronePart* mTopSphere = new DronePart(motorRadius, motorMass, positionTopSphere, world, meshFolderPath);
    droneParts.push_back(mTopSphere);

    // --------------- Create the bottom sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionBottomSphere(0, 0, -frameSize / 2);

    // Create a sphere and a corresponding rigid in the dynamics world
    DronePart* mBottomSphere = new DronePart(motorRadius, motorMass, positionBottomSphere, world, meshFolderPath);
    droneParts.push_back(mBottomSphere);

    // --------------- Create the left sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionLeftSphere(-frameSize / 2, 0, 0);

    // Create a sphere and a corresponding rigid in the dynamics world
    DronePart* mLeftSphere = new DronePart(motorRadius, motorMass, positionLeftSphere, world, meshFolderPath);
    droneParts.push_back(mLeftSphere);

    // --------------- Create the right sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionRightSphere(frameSize / 2, 0, 0);

    // Create a sphere and a corresponding rigid in the dynamics world
    DronePart* mRightSphere = new DronePart(motorRadius, motorMass, positionRightSphere, world, meshFolderPath);
    droneParts.push_back(mRightSphere);

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
    for (auto& dronePart : droneParts) {
        physBody->render(shader, worldToCameraMatrix);
    }
}

///TODO: First apply defaultTransform() to every dronePart object, Then apply setTransform() to them
void Drone::setTransform(const rp3d::Transform& transform) {
    for (auto& dronePart : droneParts) {
        rp3d::Transform defaultTransform = rp3d::Transform(dronePart->getDefaultPosition(), rp3d::Quaternion::identity());
        physBody->setTransform(transform * defaultTransform);
    }
}

Drone::~Drone() = default;


Drone::Motor::Motor(float propellerRadius, float mass, const rp3d::Vector3& defaultPosition,
                    rp3d::DynamicsWorld* dynamicsWorld,
                    const std::string& meshFolderPath) : DronePart(propellerRadius, mass, defaultPosition,
                                                                   dynamicsWorld,
                                                                   meshFolderPath) {}

Drone::DronePart::DronePart(float radius, float mass, const rp3d::Vector3& defaultPosition,
                            rp3d::DynamicsWorld* dynamicsWorld, const std::string& meshFolderPath) :
        physicsBody(new Sphere(radius, mass, dynamicsWorld, meshFolderPath)),
        defaultPosition(defaultPosition) {
    auto mYellowColor = openglframework::Color(0.9f, 0.88f, 0.145f, 1.0f);
    auto mRedColor = openglframework::Color(0.95f, 0, 0, 1.0f);
    initDronePart(this, mYellowColor, mRedColor);
}

rp3d::Vector3 Drone::DronePart::getDefaultPosition() const {
    return this->defaultPosition;
}

Drone::DronePart::~DronePart() {
    delete physicsBody;
}

Drone::DronePart* Drone::DronePart::initDronePart(Drone::DronePart* dronePart, const openglframework::Color& color,
                                                  const openglframework::Color& sleepingColor) {

    physBody->setTransform(rp3d::Transform(dronePart->getDefaultPosition(), rp3d::Quaternion::identity()));

    physBody->setColor(color);
    physBody->setSleepingColor(sleepingColor);

    rp3d::Material& materialSphere = physBody->getRigidBody()->getMaterial();
    materialSphere.setBounciness(rp3d::decimal(0.4));

    return dronePart;
}

Sphere* Drone::DronePart::getPhysicsBody() const {
    return this->physicsBody;
}
