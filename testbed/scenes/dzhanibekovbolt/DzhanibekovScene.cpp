
// Libraries
#include "DzhanibekovScene.h"
#include <cmath>

// Namespaces
using namespace openglframework;
using namespace dzhanibekovscene;

// Constructor
DzhanibekovScene::DzhanibekovScene(const std::string& name, EngineSettings& settings)
        : SceneDemo(name, settings, SCENE_RADIUS) {

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);

    // Gravity vector in the dynamics world
    rp3d::Vector3 gravity(0, rp3d::decimal(-9.81), 0);
//    rp3d::Vector3 gravity(0, 0, 0);

    rp3d::WorldSettings worldSettings;
    worldSettings.worldName = name;

    // Create the dynamics world for the physics simulation
    mPhysicsWorld = new rp3d::DynamicsWorld(gravity, worldSettings);

    //Create Drone
    createDrone();

    // Create the floor
    createFloor();

    // Get the physics engine parameters
    mEngineSettings.isGravityEnabled = getDynamicsWorld()->isGravityEnabled();
    rp3d::Vector3 gravityVector = getDynamicsWorld()->getGravity();
    mEngineSettings.gravity = openglframework::Vector3(gravityVector.x, gravityVector.y, gravityVector.z);
    mEngineSettings.isSleepingEnabled = getDynamicsWorld()->isSleepingEnabled();
    mEngineSettings.sleepLinearVelocity = getDynamicsWorld()->getSleepLinearVelocity();
    mEngineSettings.sleepAngularVelocity = getDynamicsWorld()->getSleepAngularVelocity();
    mEngineSettings.nbPositionSolverIterations = getDynamicsWorld()->getNbIterationsPositionSolver();
    mEngineSettings.nbVelocitySolverIterations = getDynamicsWorld()->getNbIterationsVelocitySolver();
    mEngineSettings.timeBeforeSleep = getDynamicsWorld()->getTimeBeforeSleep();
}

// Destructor
DzhanibekovScene::~DzhanibekovScene() {
#if 0
    // Destroy the joints
    getDynamicsWorld()->destroyJoint(mSliderJoint);
    getDynamicsWorld()->destroyJoint(mPropellerHingeJoint);
    getDynamicsWorld()->destroyJoint(mFixedJoint1);
    getDynamicsWorld()->destroyJoint(mFixedJoint2);
    for (int i = 0; i < NB_BALLSOCKETJOINT_BOXES - 1; i++) {
        getDynamicsWorld()->destroyJoint(mBallAndSocketJoints[i]);
    }

    // Destroy all the rigid bodies of the scene
    getDynamicsWorld()->destroyRigidBody(mSliderJointBottomBox->getRigidBody());
    getDynamicsWorld()->destroyRigidBody(mSliderJointTopBox->getRigidBody());
    getDynamicsWorld()->destroyRigidBody(mPropellerBox->getRigidBody());
    getDynamicsWorld()->destroyRigidBody(mFixedJointBox1->getRigidBody());
    getDynamicsWorld()->destroyRigidBody(mFixedJointBox2->getRigidBody());
    for (int i = 0; i < NB_BALLSOCKETJOINT_BOXES; i++) {
        getDynamicsWorld()->destroyRigidBody(mBallAndSocketJointChainBoxes[i]->getRigidBody());
    }

    delete mSliderJointBottomBox;
    delete mSliderJointTopBox;
    delete mPropellerBox;
    delete mFixedJointBox1;
    delete mFixedJointBox2;
    for (int i = 0; i < NB_BALLSOCKETJOINT_BOXES; i++) {
        delete mBallAndSocketJointChainBoxes[i];
    }
#endif
    // Destroy the floor
    getDynamicsWorld()->destroyRigidBody(mFloor->getRigidBody());
    delete mFloor;

    // Destroy the dynamics world
    delete getDynamicsWorld();
}

// Update the physics world (take a simulation step)
void DzhanibekovScene::updatePhysics() {
    double elapsedTime = static_cast<double>(mEngineSettings.elapsedTime) - simStartTime;
//    std::cout << "elapsed time: " << elapsedTime << std::endl;
    rp3d::Vector3 testPRY(1.0, 0.0, 0.0);
    if (elapsedTime > 5.0 && elapsedTime < 15.0) {
//        mDrone->getCentralModule()->_stabilizer->setTargetAxisPRY(testPRY);

//        mDrone->getCentralModule()->getPhysicsBody()->getRigidBody()->setAngularVelocity(rp3d::Vector3(0.0, 0.5, 0));
//        rp3d::Vector3 force = rp3d::Vector3(0, 0.0001, 0);
//        for (auto& droneModule : mDrone->getDroneModules()) {
//            droneModule->getPhysicsBody()->getRigidBody()->applyForceToCenterOfMass(force);
//        }
//        PhysicsObject* mDroneModule = mDrone->getDroneModules()[]->getPhysicsBody();
//        rp3d::Vector3 transformedForce = mDroneModule->getTransform().getOrientation() * force;
//        mDroneModule->getRigidBody()->applyForceToCenterOfMass(force);
//
//        mDrone->getMotors()[MOTOR_BL]->getPhysicsBody()->getRigidBody()->applyForceToCenterOfMass(force);
//        mDrone->getMotors()[MOTOR_BR]->getPhysicsBody()->getRigidBody()->applyForceToCenterOfMass(force);
//        force+=force;
    }

    if (elapsedTime > 7.0 && elapsedTime < 8.0) {
//        mDrone->getCentralModule()->_stabilizer->setTargetAxisPRY(-testPRY);
    }

    if (elapsedTime > 9.0) {
//        mDrone->getCentralModule()->_stabilizer->setTargetAxisPRY(rp3d::Vector3::zero());
    }

//    PhysicsObject* mCentralSphere = mDrone->getDroneModules()[0]->getPhysicsBody();
//    mCentralSphere->getRigidBody()->setLinearVelocity(rp3d::Vector3(0, 0, 0));

    mDrone->updatePhysics(mEngineSettings.timeStep);
    SceneDemo::updatePhysics();
}

// Reset the scene
void DzhanibekovScene::reset() {
    simStartTime = static_cast<double>(mEngineSettings.elapsedTime);

//    rp3d::Vector3 floorPosition(0, -5, 0);
//    mFloor->setTransform(rp3d::Transform(floorPosition,rp3d::Quaternion::identity()));

    // --------------- Drone --------------- //mFloor->setTransform(rp3d::Transform(floorPosition,rp3d::Quaternion::identity()));
    mDrone->reset();
    rp3d::Vector3 positionDrone(0, initialHeight, 0);
    mDrone->setTransform(rp3d::Transform(positionDrone, rp3d::Quaternion::identity()));
    mDrone->setFlightMode(STAB_HEIGHT);
}


// Create the floor
void DzhanibekovScene::createFloor() {

    // Create the floor
    rp3d::Vector3 floorPosition(0, 0, 0);
    mFloor = new Box(FLOOR_SIZE, FLOOR_MASS, getDynamicsWorld(), mMeshFolderPath);

    // Set the box color
    mFloor->setColor(mGreyColorDemo);
    mFloor->setSleepingColor(mGreyColorDemo);

//    mFloor->setTransform(rp3d::Transform(floorPosition,rp3d::Quaternion::identity()));
    // The floor must be a static rigid body
    mFloor->getRigidBody()->setType(rp3d::BodyType::STATIC);

    // Change the material properties of the rigid body
    rp3d::Material& material = mFloor->getRigidBody()->getMaterial();
    material.setBounciness(rp3d::decimal(0.3));
    mPhysicsObjects.push_back(mFloor);
}

void DzhanibekovScene::createDrone() {
    rp3d::Vector3 positionDrone(0, initialHeight, 0);
    QuadPIDs quadPids(PID(0.7, 0.35, 0.05),
                      PID(0.07, 0.035, 0.005),
                      PID(0.07, 0.035, 0.0005),
                      PID(7, 3.5, 3.5));
    double droneFrame = 0.088;
    double droneMass = 0.12;
    double motorMass = 0.01;
    double propellerRadius = 0.02;

    mDrone = new Drone(droneFrame, droneMass, propellerRadius, motorMass,
                       quadPids, getDynamicsWorld(), mMeshFolderPath);
    mDrone->setTransform(rp3d::Transform(positionDrone, rp3d::Quaternion::identity()));
    for (auto& mDroneModule : mDrone->getDroneModules()) {
        mPhysicsObjects.push_back(mDroneModule->getPhysicsBody());
    }
    mDrone->setFlightMode(STAB_HEIGHT);
}

static rp3d::Vector3 testPRY(0.0, 0.0, 0.0);

bool DzhanibekovScene::keyboardEvent(int key, int scancode, int action, int mods) {

    if (key == GLFW_KEY_W && action == GLFW_PRESS) {
        testPRY.x = 0.5;
    }

    if (key == GLFW_KEY_W && action == GLFW_RELEASE) {
        testPRY.x = 0.0;
    }

    if (key == GLFW_KEY_S && action == GLFW_PRESS) {
        testPRY.x = -0.5;
    }

    if (key == GLFW_KEY_S && action == GLFW_RELEASE) {
        testPRY.x = 0.0;
    }

    if (key == GLFW_KEY_A && action == GLFW_PRESS) {
        testPRY.y = -0.5;
    }

    if (key == GLFW_KEY_A && action == GLFW_RELEASE) {
        testPRY.y = 0.0;
    }

    if (key == GLFW_KEY_D && action == GLFW_PRESS) {
        testPRY.y = 0.5;
    }

    if (key == GLFW_KEY_D && action == GLFW_RELEASE) {
        testPRY.y = 0.0;
    }

    if (key == GLFW_KEY_Q && action == GLFW_PRESS) {
        testPRY.z = 5.0;
    }

    if (key == GLFW_KEY_Q && action == GLFW_RELEASE) {
        testPRY.z = 0.0;
    }

    if (key == GLFW_KEY_E && action == GLFW_PRESS) {
        testPRY.z = -5.0;
    }

    if (key == GLFW_KEY_E && action == GLFW_RELEASE) {
        testPRY.z = 0.0;
    }

    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        mDrone->setFlightMode(mDrone->getFlightMode() ? STAB : STAB_HEIGHT);
    }

    if (key == GLFW_KEY_R && action == GLFW_PRESS) {
        mDrone->setFlightMode(STAB);
        mDrone->setThrottle(mDrone->getThrottle() + 0.05);
    }

    if (key == GLFW_KEY_F && action == GLFW_PRESS) {
        mDrone->setFlightMode(STAB);
        mDrone->setThrottle(mDrone->getThrottle() - 0.05);
    }

//    if (key == GLFW_KEY_V && action == GLFW_PRESS) {
//        mDrone->getMotors()[MOTOR_BL]->getPhysicsBody()->getRigidBody()->applyForceToCenterOfMass(rp3d::Vector3(0.0, 0.5, 0.0));
//        mDrone->getMotors()[MOTOR_BR]->getPhysicsBody()->getRigidBody()->applyForceToCenterOfMass(rp3d::Vector3(0.0, 0.5, 0.0));
//    }


    mDrone->getCentralModule()->_stabilizer->setTargetAxisPRY(testPRY);

    return false;
}


#include "DzhanibekovScene.h"
