//
// Created by kirill on 23.03.2020.
//

#ifndef REACTPHYSICS3D_DZHANIBEKOVSCENE_H
#define REACTPHYSICS3D_DZHANIBEKOVSCENE_H
#define TEST_FLAG 0
// Libraries
#include "openglframework.h"
#include "reactphysics3d.h"
#include "Box.h"
#include "SceneDemo.h"
#include "Sphere.h"
#include "Drone.h"
namespace dzhanibekovscene {

// Constants
    const float SCENE_RADIUS = 30.0f;
    const openglframework::Vector3 BOX_SIZE(2, 2, 2);           // Box dimensions in meters
    const openglframework::Vector3 FLOOR_SIZE(50, 0.5f, 50);    // Floor dimensions in meters
    const float BOX_MASS = 1.0f;                                // Box mass in kilograms
    const float FLOOR_MASS = 100.0f;                            // Floor mass in kilograms
    const int NB_BALLSOCKETJOINT_BOXES = 7;                     // Number of Ball-And-Socket chain boxes
    const int NB_HINGE_BOXES = 7;                               // Number of Hinge chain boxes

// Class DzhanibekovsScene
    class DzhanibekovScene : public SceneDemo {

    protected :

        // -------------------- Attributes -------------------- //

        /// Boxes of Ball-And-Socket joint chain
        Box* mBallAndSocketJointChainBoxes[NB_BALLSOCKETJOINT_BOXES];

        /// Boxes of the Hinge joint chain
        Box* mHingeJointChainBoxes[NB_HINGE_BOXES];

        /// Ball-And-Socket joints of the chain
        rp3d::BallAndSocketJoint* mBallAndSocketJoints[NB_BALLSOCKETJOINT_BOXES - 1];

        /// Hinge joints of the chain
        rp3d::HingeJoint* mHingeJoints[NB_HINGE_BOXES - 1];

        /// Bottom box of the Slider joint
        Box* mSliderJointBottomBox;

        /// Top box of the Slider joint
        Box* mSliderJointTopBox;

        /// Slider joint
        rp3d::SliderJoint* mSliderJoint;

        /// Propeller box
        Box* mPropellerBox;

        /// Box 1 of Fixed joint
        Box* mFixedJointBox1;

        /// Box 2 of Fixed joint
        Box* mFixedJointBox2;

        /// Hinge joint
        rp3d::HingeJoint* mPropellerHingeJoint;

        /// First Fixed joint
        rp3d::FixedJoint* mFixedJoint1;

        /// Second Fixed joint
        rp3d::FixedJoint* mFixedJoint2;

        /// Box for the floor
        Box* mFloor;

//        Sphere* mCentralSphere;
//        Sphere* mTopSphere;
//        Sphere* mBottomSphere;
//        Sphere* mLeftSphere;
//        Sphere* mRightSphere;
//        Line* mLineTopBottom;
//        Line* mLineLeftRight;
//
//        rp3d::FixedJoint* mFixedJointCentralTop;
//        rp3d::FixedJoint* mFixedJointCentralBottom;
//        rp3d::FixedJoint* mFixedJointCentralLeft;
//        rp3d::FixedJoint* mFixedJointCentralRight;
        float initialHeight = 5;

        Drone* mDrone;
        double simStartTime, simElapsedTime;

        // -------------------- Methods -------------------- //

        /// Create the boxes and joints for the Ball-and-Socket joint example
        void createBallAndSocketJoints();

        /// Create the boxes and joint for the Slider joint example
        void createSliderJoint();

        /// Create the boxes and joint for the Hinge joint example
        void createPropellerHingeJoint();

        /// Create the boxes and joint for the Fixed joint example
        void createFixedJoints();

        /// Create the floor
        void createFloor();

        void createDrone();

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        DzhanibekovScene(const std::string& name, EngineSettings& settings);

        /// Destructor
        virtual ~DzhanibekovScene() override;

        /// Update the physics world (take a simulation step)
        /// Can be called several times per frame
        virtual void updatePhysics() override;

        /// Reset the scene
        virtual void reset() override;

        /// Return all the contact points of the scene
//        virtual std::vector<ContactPoint> getContactPoints() override;
    };

// Return all the contact points of the scene
//    inline std::vector<ContactPoint> DzhanibekovScene::getContactPoints() {
//        return computeContactPointsOfWorld(getDynamicsWorld());
//    }

}


#endif //REACTPHYSICS3D_DZHANIBEKOVSCENE_H
