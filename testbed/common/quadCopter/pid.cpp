#include <iostream>
#include <cmath>
#include "pid.h"
#include <limits>

using namespace std;

class PIDImpl {
public:
    PIDImpl(double max, double min, double Kp, double Kd, double Ki);

    ~PIDImpl();

    PIDImpl(const PIDImpl&);

    double calculate(double dt, double wantedPoint, double currentPoint);

    void reset();

    void setMinMax(double min, double max);

private:
    double _dt;
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral;
};


PID::PID(double max, double min, double Kp, double Ki, double Kd) {
    pimpl = new PIDImpl(max, min, Kp, Kd, Ki);
}

PID::PID(double Kp, double Ki, double Kd) {
    pimpl = new PIDImpl(std::numeric_limits<double>::max(), std::numeric_limits<double>::min(), Kp, Kd, Ki);
}

double PID::calculate(double dt, double targetPoint, double currentPoint) {
    return pimpl->calculate(dt, targetPoint, currentPoint);
}

PID::~PID() {
    delete pimpl;
}

PID::PID(const PID& pid) : pimpl(new PIDImpl(*(pid.pimpl))) {}

void PID::reset() {
    pimpl->reset();
}

void PID::setMinMax(double min, double max) {
    pimpl->setMinMax(min, max);
}


/**
 * Implementation
 */
PIDImpl::PIDImpl(double max, double min, double Kp, double Kd, double Ki) :
        _dt(0),
        _max(max),
        _min(min),
        _Kp(Kp),
        _Kd(Kd),
        _Ki(Ki),
        _pre_error(0),
        _integral(0) {
}

double PIDImpl::calculate(double dt, double wantedPoint, double currentPoint) {
    _dt = dt;

    // Calculate error
    double error = wantedPoint - currentPoint;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
//    cout << "Pout: " << Pout << " Iout: " << Iout << " Dout: " << Dout << endl;
    double output = Pout + Iout + Dout;

//    cout << "error: " << error << " output: " << output << " max: " << _max << " min: " << _min << endl;

    // Restrict to max/min
    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::PIDImpl(const PIDImpl& pidImpl) {
    _dt = pidImpl._dt;
    _max = pidImpl._max;
    _min = pidImpl._min;
    _Kp = pidImpl._Kp;
    _Kd = pidImpl._Kd;
    _Ki = pidImpl._Ki;
    _pre_error = pidImpl._pre_error;
    _integral = pidImpl._integral;
}

void PIDImpl::reset() {
    _dt = 0;
    _pre_error = 0;
    _integral = 0;
}

void PIDImpl::setMinMax(double min, double max) {
    _min = min;
    _max = max;
}


PIDImpl::~PIDImpl() = default;

