
// Libraries
#include "QuadScene.h"
#include <cmath>

// Namespaces
using namespace openglframework;
using namespace quadscene;

// Constructor
QuadScene::QuadScene(const std::string& name, EngineSettings& settings)
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
QuadScene::~QuadScene() {
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
    //Destroy the quad
    mDrone->destroyQuadModules(getDynamicsWorld());
    delete mDrone;

    // Destroy the floor
    getDynamicsWorld()->destroyRigidBody(mFloor->getRigidBody());
    delete mFloor;

    // Destroy the dynamics world
    delete getDynamicsWorld();
}

// Update the physics world (take a simulation step)
void QuadScene::updatePhysics() {
    double elapsedTime = static_cast<double>(mEngineSettings.elapsedTime) - simStartTime;

    mDrone->updatePhysics(mEngineSettings.timeStep);
    SceneDemo::updatePhysics();
}

// Reset the scene
void QuadScene::reset() {
    simStartTime = static_cast<double>(mEngineSettings.elapsedTime);

    // --------------- Drone --------------- //
    mDrone->reset();
    rp3d::Vector3 positionDrone(0, initialHeight, 0);
    mDrone->setTransform(rp3d::Transform(positionDrone, rp3d::Quaternion::identity()));
    mDrone->setFlightMode(STAB_HEIGHT);
}


// Create the floor
void QuadScene::createFloor() {

    // Create the floor
    rp3d::Vector3 floorPosition(0, 0, 0);
    mFloor = new Box(FLOOR_SIZE, FLOOR_MASS, getDynamicsWorld(), mMeshFolderPath);

    // Set the box color
    mFloor->setColor(mGreyColorDemo);
    mFloor->setSleepingColor(mGreyColorDemo);

    // The floor must be a static rigid body
    mFloor->getRigidBody()->setType(rp3d::BodyType::STATIC);

    // Change the material properties of the rigid body
    rp3d::Material& material = mFloor->getRigidBody()->getMaterial();
    material.setBounciness(rp3d::decimal(0.3));
    mPhysicsObjects.push_back(mFloor);
}

void QuadScene::createDrone() {
    rp3d::Vector3 positionDrone(0, initialHeight, 0);
    QuadPIDs quadPIDs(PID(0.07, 0.035, 0.015),
                      PID(0.07, 0.035, 0.015),
                      PID(0.07, 0.035, 0.0005),
                      PID(7, 3.5, 3.5));
    double droneFrame = 0.088;
    double droneMass = 0.12;
    double motorMass = 0.01;
    double propellerRadius = 0.02;

    mDrone = new Drone(droneFrame, droneMass, propellerRadius, motorMass,
                       quadPIDs, getDynamicsWorld(), mMeshFolderPath);
    mDrone->setTransform(rp3d::Transform(positionDrone, rp3d::Quaternion::identity()));
    for (auto& mDroneModule : mDrone->getDroneModules()) {
        mPhysicsObjects.push_back(mDroneModule->getPhysicsBody().get());
    }
    mDrone->setFlightMode(STAB_HEIGHT);
}

static rp3d::Vector3 testPRY(1500.0, 1500.0, 1500.0);
static double quadThrottle = 0.0;

bool QuadScene::keyboardEvent(int key, int scancode, int action, int mods) {

    if (action == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_W:
                testPRY.x = 2100;
                break;
            case GLFW_KEY_S:
                testPRY.x = 900;
                break;
            case GLFW_KEY_A:
                testPRY.y = 2100;
                break;
            case GLFW_KEY_D:
                testPRY.y = 900;
                break;
            case GLFW_KEY_Q:
                testPRY.z = 2100;
                break;
            case GLFW_KEY_E:
                testPRY.z = 900;
                break;
            case GLFW_KEY_SPACE:
                mDrone->setFlightMode(mDrone->getFlightMode() ? STAB : STAB_HEIGHT);
                break;
            case GLFW_KEY_R:
                mDrone->setFlightMode(STAB);
                quadThrottle = mDrone->getThrottle() + 50;
                break;
            case GLFW_KEY_F:
                mDrone->setFlightMode(STAB);
                quadThrottle = mDrone->getThrottle() - 50;
                break;
        }
    }

    if (action == GLFW_RELEASE) {
        switch (key) {
            case GLFW_KEY_W:
            case GLFW_KEY_S:
                testPRY.x = 1500.0;
                break;
            case GLFW_KEY_A:
            case GLFW_KEY_D:
                testPRY.y = 1500.0;
                break;
            case GLFW_KEY_Q:
            case GLFW_KEY_E:
                testPRY.z = 1500.0;
                break;
        }
    }

    mDrone->setInputParams(testPRY.x, testPRY.y, testPRY.z, quadThrottle);

    return false;
}


#include "QuadScene.h"
