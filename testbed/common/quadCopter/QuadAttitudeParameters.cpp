//
// Created by kirill on 02.04.2020.
//

#include "QuadAttitudeParameters.h"

namespace quad {
    QuadAttitudeParameters::QuadAttitudeParameters(double altitude,
                                                   const QuadAngles& quadAngles) : _altitude(altitude),
                                                                                   _quadAngles(quadAngles),
                                                                                   _throttle(0) {
        for (size_t i = 0; i <= THROTTLE; ++i) {
            _scaleFunctions.push_back(
                    std::make_shared<ScaleManager<double, double>>([](double x) { return x; }));
        }
    }

    void QuadAttitudeParameters::setInputParameters(const QuadAngles& quadAngles, double throttle, double maxThrottle) {
        for (int i = 0; i <= YAW; ++i) {
            double normalizedInput = scaleInput(constrainToRange(quadAngles[i], minInput, maxInput), -1, 1);
            _quadAngles[i] = (*_scaleFunctions[i])(normalizedInput) * (M_PI / 3);
        }
        _throttle = (*_scaleFunctions[THROTTLE])(scaleInput(constrainToRange(throttle, minInput, maxInput), 0, 1)) * maxThrottle;
    }

    QuadAttitudeParameters::QuadAttitudeParameters(const QuadAttitudeParameters& attitudeParameters) {
        _altitude = attitudeParameters._altitude;
        _quadAngles = attitudeParameters._quadAngles;
        _throttle = attitudeParameters._throttle;

        for (size_t i = 0; i <= THROTTLE; ++i) {
            _scaleFunctions.push_back(std::make_shared<ScaleManager<double, double>>(
                    attitudeParameters._scaleFunctions[i]->getFunction()));
        }
    }

    double QuadAttitudeParameters::getAltitude() const {
        return _altitude;
    }

    QuadAngles QuadAttitudeParameters::getQuadAngles() const {
        return _quadAngles;
    }

    void QuadAttitudeParameters::setAltitude(double altitude) {
        _altitude = altitude;
    }

    void QuadAttitudeParameters::setQuadAngles(const QuadAngles& quadAngles) {
        _quadAngles = quadAngles;
    }

    rp3d::Vector3 QuadAttitudeParameters::getAngularVelocity() const {
        return _angularVelocity;
    }

    void QuadAttitudeParameters::setAngularVelocity(const rp3d::Vector3& angularVelocity) {
        _angularVelocity = angularVelocity;
    }

    std::vector<std::shared_ptr<ScaleManager<double, double>>> QuadAttitudeParameters::getScaleFunctions() const {
        return _scaleFunctions;
    }

    void QuadAttitudeParameters::setScaleFunctions(const ScaleManager<double, double>& pitchFunc,
                                                   const ScaleManager<double, double>& rollFunc,
                                                   const ScaleManager<double, double>& yawFunc,
                                                   const ScaleManager<double, double>& throttleFunc) {
        _scaleFunctions[PITCH]->setFunction(pitchFunc);
        _scaleFunctions[ROLL]->setFunction(rollFunc);
        _scaleFunctions[YAW]->setFunction(yawFunc);
        _scaleFunctions[THROTTLE]->setFunction(throttleFunc);
    }

    QuadAttitudeParameters::~QuadAttitudeParameters() {
        _scaleFunctions.clear();
    }

    double QuadAttitudeParameters::getThrottle() const {
        return _throttle;
    }

    void QuadAttitudeParameters::setThrottle(double throttle) {
        _throttle = throttle;
    }

    /// Scale to range [begin, end]
    double QuadAttitudeParameters::scaleInput(double value, double begin, double end) {
        return (end - begin) * normalize(value) + begin;
    }

    void QuadAttitudeParameters::reset() {
        _altitude = 0;
        _quadAngles = QuadAngles::zero();
        _angularVelocity = rp3d::Vector3::zero();
        _throttle = 0;
    }

    double QuadAttitudeParameters::normalize(double value) {
        return (value - minInput) / (maxInput - minInput);
    }

    double QuadAttitudeParameters::constrainToRange(double value, double begin, double end) {
        return std::max(begin, std::min(value, end));
    }

}