//
// Created by kirill on 25.03.2020.
//

#include "Drone.h"


Drone::Drone(float frameSize, float droneMass, float motorRadius, float motorMass, rp3d::DynamicsWorld* world,
             const std::string& meshFolderPath) {

    auto mYellowColor = openglframework::Color(0.9f, 0.88f, 0.145f, 1.0f);
    auto mRedColor = openglframework::Color(0.95f, 0, 0, 1.0f);
    auto mBlueColor = openglframework::Color(0.9f, 0.88f, 0.145f, 1.0f);

    Sphere* mCentralSphere;
    Sphere* mTopSphere;
    Sphere* mBottomSphere;
    Sphere* mLeftSphere;
    Sphere* mRightSphere;

    rp3d::FixedJoint* mFixedJointCentralTop;
    rp3d::FixedJoint* mFixedJointCentralBottom;
    rp3d::FixedJoint* mFixedJointCentralLeft;
    rp3d::FixedJoint* mFixedJointCentralRight;

    // --------------- Create the central sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionCentralSphere(0, 0, 0);

    // Create a sphere and a corresponding rigid in the dynamics world
    float centerMass = droneMass - 4 * motorMass;
    cout << centerMass;
    mCentralSphere = new Sphere(0.5, centerMass, world, meshFolderPath);
    mCentralSphere->setTransform(rp3d::Transform(positionCentralSphere, rp3d::Quaternion::identity()));

    mCentralSphere->setColor(mYellowColor);
    mCentralSphere->setSleepingColor(mRedColor);

    rp3d::Material& materialSphere = mCentralSphere->getRigidBody()->getMaterial();
    materialSphere.setBounciness(rp3d::decimal(0.4));
    droneParts.push_back(mCentralSphere);

    // --------------- Create the top sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionTopSphere(0, 0, frameSize / 2);

    // Create a sphere and a corresponding rigid in the dynamics world
    mTopSphere = new Sphere(1.0, 1.0, world, meshFolderPath);
    mTopSphere->setTransform(rp3d::Transform(positionTopSphere, rp3d::Quaternion::identity()));

    // Set the box color
    mTopSphere->setSleepingColor(mRedColor);
    mTopSphere->setColor(mBlueColor);

    // Change the material properties of the rigid body
    rp3d::Material& materialSphere2 = mTopSphere->getRigidBody()->getMaterial();
    materialSphere2.setBounciness(rp3d::decimal(0.4));
//    motors.push_back(mTopSphere);
    droneParts.push_back(mTopSphere);

    // --------------- Create the bottom sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionBottomSphere(0, 0, -frameSize / 2);

    // Create a sphere and a corresponding rigid in the dynamics world
    mBottomSphere = new Sphere(1.0, 1.0, world, meshFolderPath);
    mBottomSphere->setTransform(rp3d::Transform(positionBottomSphere, rp3d::Quaternion::identity()));

    // Set the box color
    mBottomSphere->setSleepingColor(mRedColor);
    mBottomSphere->setColor(mBlueColor);

    // Change the material properties of the rigid body
    rp3d::Material& materialBottomSphere = mBottomSphere->getRigidBody()->getMaterial();
    materialBottomSphere.setBounciness(rp3d::decimal(0.4));
    droneParts.push_back(mBottomSphere);

    // --------------- Create the left sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionLeftSphere(-frameSize / 2, 0, 0);

    // Create a sphere and a corresponding rigid in the dynamics world
    mLeftSphere = new Sphere(1.0, 1.0, world, meshFolderPath);
    mLeftSphere->setTransform(rp3d::Transform(positionLeftSphere, rp3d::Quaternion::identity()));

    // Set the box color
    mLeftSphere->setSleepingColor(mRedColor);
    mLeftSphere->setColor(mBlueColor);

    // Change the material properties of the rigid body
    rp3d::Material& materialLeftSphere = mLeftSphere->getRigidBody()->getMaterial();
    materialLeftSphere.setBounciness(rp3d::decimal(0.4));
    droneParts.push_back(mLeftSphere);

    // --------------- Create the right sphere --------------- //

    // Position of the sphere
    rp3d::Vector3 positionRightSphere(frameSize / 2, 0, 0);

    // Create a sphere and a corresponding rigid in the dynamics world
    mRightSphere = new Sphere(1.0, 0.5, world, meshFolderPath);
    mRightSphere->setTransform(rp3d::Transform(positionRightSphere, rp3d::Quaternion::identity()));

    // Set the box color
    mRightSphere->setSleepingColor(mRedColor);
    mRightSphere->setColor(mBlueColor);

    // Change the material properties of the rigid body
    rp3d::Material& materialRightSphere = mRightSphere->getRigidBody()->getMaterial();
    materialRightSphere.setBounciness(rp3d::decimal(0.4));
    droneParts.push_back(mRightSphere);

#if 0
    // --------------- Create the central top fixed joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* topSphereBody = mTopSphere->getRigidBody();
    rp3d::RigidBody* centralSphereBody = mCentralSphere->getRigidBody();
    const rp3d::Vector3& anchorPointWorldSpace1(positionCentralSphere);
    rp3d::FixedJointInfo jointInfoTop(topSphereBody, centralSphereBody, anchorPointWorldSpace1);
    jointInfoTop.isCollisionEnabled = false;

    // Create the joint in the dynamics world
    fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(jointInfoTop)));

    // --------------- Create the central bottom fixed joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* bottomSphereBody = mBottomSphere->getRigidBody();
    rp3d::FixedJointInfo jointInfoBottom(bottomSphereBody, centralSphereBody, anchorPointWorldSpace1);
    jointInfoBottom.isCollisionEnabled = false;

    // Create the joint in the dynamics world
    fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(jointInfoBottom)));

    // --------------- Create the central left fixed joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* leftSphereBody = mLeftSphere->getRigidBody();
    rp3d::FixedJointInfo jointInfoLeft(leftSphereBody, centralSphereBody, anchorPointWorldSpace1);
    jointInfoLeft.isCollisionEnabled = false;

    // Create the joint in the dynamics world
    fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(jointInfoLeft)));

    // --------------- Create the central right fixed joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* rightSphereBody = mRightSphere->getRigidBody();
    rp3d::FixedJointInfo jointInfoRight(rightSphereBody, centralSphereBody, anchorPointWorldSpace1);
    jointInfoRight.isCollisionEnabled = false;

    // Create the joint in the dynamics world
    fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(jointInfoRight)));
#endif
}

void Drone::render(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {
    for (auto & dronePart : droneParts) {
        dronePart->render(shader, worldToCameraMatrix);
    }
}

void Drone::setTransform(const rp3d::Transform& transform) {
    for (auto & dronePart : droneParts) {
        dronePart->setTransform(transform * dronePart->getTransform());
    }
}

Drone::~Drone() = default;
