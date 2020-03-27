//
// Created by kirill on 25.03.2020.
//

#ifndef REACTPHYSICS3D_DRONE_H
#define REACTPHYSICS3D_DRONE_H

#include "reactphysics3d.h"
#include "openglframework.h"
#include "Sphere.h"

class Drone : public openglframework::Object3D {
private:

    class Motor : Sphere {
    };

    std::vector<Motor*> motors;
    std::vector<rp3d::FixedJoint*> fixedJoints;
    std::vector<PhysicsObject*> droneParts;


public:
    Drone(float frameSize, float droneMass, float motorRadius, float motorMass, rp3d::DynamicsWorld* world,
          const std::string& meshFolderPath);

    ~Drone();

    void render(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix);

    void setTransform(const rp3d::Transform& transform);

    inline std::vector<Motor*> getMotors() const {
        return motors;
    }

    inline std::vector<rp3d::FixedJoint*> getFixedJoints() const {
        return fixedJoints;
    }

    inline std::vector<PhysicsObject*> getDroneParts() const {
        return droneParts;
    }

};

#endif //REACTPHYSICS3D_DRONE_H
