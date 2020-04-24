//
// Created by kirill on 24.04.2020.
//

#ifndef REACTPHYSICS3D_QUADANGLES_H
#define REACTPHYSICS3D_QUADANGLES_H

namespace quad {
    class QuadAngles {
    private:
        double _pitch;
        double _roll;
        double _yaw;

    public:
        QuadAngles();

        QuadAngles(double pitch, double roll, double yaw);

        QuadAngles(const QuadAngles& quadAngles);

        ~QuadAngles() = default;

        static QuadAngles zero();

        double getPitch() const;

        double getRoll() const;

        double getYaw() const;

        void setPitch(double value);

        void setRoll(double value);

        void setYaw(double value);

        void setAllAngles(double pitch, double roll, double yaw);

        /// Overloaded operator for value access
        double& operator[] (int index);

        /// Overloaded operator for value access
        const double& operator[] (int index) const;

        /// Overloaded operator
        QuadAngles& operator=(const QuadAngles& quadAngles);
    };
}

#endif //REACTPHYSICS3D_QUADANGLES_H
