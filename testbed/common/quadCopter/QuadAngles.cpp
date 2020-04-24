//
// Created by kirill on 24.04.2020.
//

#include "QuadAngles.h"

namespace quad {
    QuadAngles::QuadAngles() : _pitch(0.0), _roll(0.0), _yaw(0.0) {}

    QuadAngles::QuadAngles(double pitch, double roll, double yaw) : _pitch(pitch), _roll(roll), _yaw(yaw) {}

    QuadAngles::QuadAngles(const quad::QuadAngles& quadAngles) : _pitch(quadAngles._pitch),
                                                                 _roll(quadAngles._roll),
                                                                 _yaw(quadAngles._yaw) {}

    QuadAngles QuadAngles::zero() {
        return QuadAngles(0, 0, 0);
    }

    void QuadAngles::setAllAngles(double pitch, double roll, double yaw) {
        _pitch = pitch;
        _roll = roll;
        _yaw = yaw;
    }

    double QuadAngles::getPitch() const {
        return _pitch;
    }

    double QuadAngles::getRoll() const {
        return _roll;
    }

    double QuadAngles::getYaw() const {
        return _yaw;
    }

    void QuadAngles::setPitch(double value) {
        _pitch = value;
    }

    void QuadAngles::setRoll(double value) {
        _roll = value;
    }

    void QuadAngles::setYaw(double value) {
        _yaw = value;
    }

    const double& QuadAngles::operator[](int index) const {
        return (&_pitch)[index];
    }

    double& QuadAngles::operator[](int index) {
        return (&_pitch)[index];
    }

    QuadAngles& QuadAngles::operator=(const QuadAngles& quadAngles) {
        if (&quadAngles != this) {
            _pitch = quadAngles._pitch;
            _roll = quadAngles._roll;
            _yaw = quadAngles._yaw;
        }
        return *this;
    }
}