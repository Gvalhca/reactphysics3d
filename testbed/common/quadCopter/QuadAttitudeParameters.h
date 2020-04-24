//
// Created by kirill on 02.04.2020.
//

#ifndef REACTPHYSICS3D_QUADATTITUDEPARAMETERS_H
#define REACTPHYSICS3D_QUADATTITUDEPARAMETERS_H


#include <reactphysics3d.h>
#include <vector>
#include "ScaleManager.h"
#include "QuadAngles.h"

namespace quad {
    typedef enum {
        PITCH = 0,
        ROLL = 1,
        YAW = 2,
        THROTTLE = 3
    } AttitudeParams;

    ///TODO: Unite _throttle and _quadAngles in one struct
    class QuadAttitudeParameters {
    private:
        double _altitude;
        QuadAngles _quadAngles;
        rp3d::Vector3 _angularVelocity;
        double _throttle;
        std::vector<std::shared_ptr<ScaleManager<double, double>>> _scaleFunctions;

        constexpr static double minInput = 900, maxInput = 2100;

        static double scaleInput(double value, double begin, double end);

        static double normalize(double value);

        static double constrainToRange(double value, double begin, double end);

    public:
        explicit QuadAttitudeParameters(double altitude = 0, const QuadAngles& quadAngles = QuadAngles::zero());

        ~QuadAttitudeParameters();

        QuadAttitudeParameters(const QuadAttitudeParameters&);

        void setInputParameters(const QuadAngles& quadAngles, double throttle, double maxThrottle);

        void setAltitude(double altitude);

        void setQuadAngles(const QuadAngles& quadAngles);

        void setAngularVelocity(const rp3d::Vector3& angularVelocity);

        double getAltitude() const;

        QuadAngles getQuadAngles() const;

        rp3d::Vector3 getAngularVelocity() const;

        double getThrottle() const;

        void setThrottle(double throttle);

        void reset();

        std::vector<std::shared_ptr<ScaleManager<double, double>>> getScaleFunctions() const;

        void setScaleFunctions(const ScaleManager<double, double>& pitchFunc,
                               const ScaleManager<double, double>& rollFunc,
                               const ScaleManager<double, double>& yawFunc,
                               const ScaleManager<double, double>& throttleFunc);
    };
}

#endif //REACTPHYSICS3D_QUADATTITUDEPARAMETERS_H
