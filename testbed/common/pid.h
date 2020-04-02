#ifndef PID_H
#define PID_H


#include <vector>

class PIDImpl;

class PID {
public:
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // dt -  loop interval time
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable
    PID(double max, double min, double Kp, double Kd, double Ki);

    PID(double Kp, double Kd, double Ki);

    PID(const PID&);

    // Returns the manipulated variable given a setpoint and current process value
    double calculate(double dt, double targetPoint, double currentPoint);

    ~PID();

    void reset();

    void setMinMax(double min, double max);
private:
    PIDImpl* pimpl;
};

#endif // PID_H
