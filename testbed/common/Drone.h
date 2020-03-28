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

    class DronePart {
    private:
        Sphere* physicsBody;
        rp3d::Vector3 defaultPosition;

    public:
        DronePart(float radius, float mass, const rp3d::Vector3& defaultPosition, rp3d::DynamicsWorld* dynamicsWorld,
                  const std::string& meshFolderPath);

        ~DronePart();

        rp3d::Vector3 getDefaultPosition() const;

        static DronePart* initDronePart(DronePart* dronePart, const openglframework::Color& color,
                                        const openglframework::Color& sleepingColor);

        Sphere* getPhysicsBody() const;
    };

    class Motor : public DronePart {
    private:
        double throttle = 0;
    public:
        Motor(float radius, float mass, const rp3d::Vector3& defaultPosition, rp3d::DynamicsWorld* dynamicsWorld,
              const std::string& meshFolderPath);
    };

    std::vector<Motor*> motors;
    std::vector<rp3d::FixedJoint*> fixedJoints;
    std::vector<DronePart*> droneParts;


public:
    Drone(float frameSize, float droneMass, float motorRadius, float motorMass, rp3d::DynamicsWorld* world,
          const std::string& meshFolderPath);

    ~Drone() override;

    void render(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix);

    void setTransform(const rp3d::Transform& transform);

    inline std::vector<Motor*> getMotors() const {
        return motors;
    }

    inline std::vector<rp3d::FixedJoint*> getFixedJoints() const {
        return fixedJoints;
    }

    inline std::vector<DronePart*> getDroneParts() const {
        return droneParts;
    }

};

#endif //REACTPHYSICS3D_DRONE_H
