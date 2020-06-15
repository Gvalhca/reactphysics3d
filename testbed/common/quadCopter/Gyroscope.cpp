//
// Created by kirill on 03.04.2020.
//

#include "Gyroscope.h"


quad::QuadAngles quad::Gyroscope::toEulerAngles(const rp3d::Quaternion& q) const {
//    // roll (x-axis rotation)
//    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
//    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
//    angles.x = std::atan2(sinr_cosp, cosr_cosp);
//
//    // pitch (y-axis rotation)
//    double sinp = 2 * (q.w * q.y - q.z * q.x);
//    if (std::abs(sinp) >= 1)
//        angles.z = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//    else
//        angles.z = std::asin(sinp);
//
//    // yaw (z-axis rotation)
//    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
//    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
//    angles.y = std::atan2(siny_cosp, cosy_cosp);
//
//    rp3d::Vector3 attitude(0, 0, 0);
//    attitude.y = std::asin(orientMatrix[1][2]);
//    attitude.x = std::atan2(-orientMatrix[0][2] / std::cos(attitude.y),
//                            orientMatrix[2][2] / std::cos(attitude.y));
//    attitude.z = std::atan2(-orientMatrix[1][0] / std::cos(attitude.y),
//                            orientMatrix[1][1] / std::cos(attitude.y));
//
//    std::cout << "PRY from Matrix: " << (attitude * toDegrees).to_string() << std::endl;
//
//    // This doesn't work when we are rotated +-PI/2 by yaw
//    double sinr_cosp = 2 * (q.w * q.z + q.y * q.x);
//    double cosr_cosp = 1 - 2 * (q.z * q.z + q.y * q.y);
//    angles.y = std::atan2(sinr_cosp, cosr_cosp);
//    std::cout << "sinr_cosp: " << sinr_cosp << "cosr_cosp: " << cosr_cosp << std::endl;

    QuadAngles angles;
    double toDegrees = 180 / M_PI;

    // roll (x-axis rotation)
    // Proper roll calculation
    rp3d::Matrix3x3 orientMatrix = q.getMatrix();
    double phi = std::asin(orientMatrix[1][2]);
    angles.setRoll(-std::atan2(-orientMatrix[1][0] / std::cos(phi),
                               orientMatrix[1][1] / std::cos(phi)));

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.x - q.z * q.y);
    if (std::abs(sinp) >= 1)
        angles.setPitch(std::copysign(M_PI / 2, sinp)); // use 90 degrees if out of range
    else
        angles.setPitch(std::asin(sinp));

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.x + q.z * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.x * q.x);
    angles.setYaw(std::atan2(siny_cosp, cosy_cosp));
    angles.setYaw(std::cos(angles.getYaw()));

//    std::cout << "PRY from Quaternion: " << (angles * toDegrees).to_string() << std::endl;
//    angles.y = std::sin(angles.y);
//    std::cout << "PRY to quad: " << (angles).to_string() << std::endl;
    return angles;

}



quad::QuadAngles quad::Gyroscope::getPRYFromQuaternion(const rp3d::Quaternion& Q) const {
    double toDegrees = 180 / M_PI;
    double angleYaw = asin(2.0 * (Q.w * Q.y - Q.z * Q.x));
    rp3d::Quaternion qYaw = rp3d::Quaternion::fromEulerAngles(0.0, -angleYaw, 0.0);
    rp3d::Quaternion qZeroYaw = Q * qYaw;
    double pitch = atan2(2.0 * (qZeroYaw.w * qZeroYaw.x + qZeroYaw.y * qZeroYaw.z),
                         1.0 - 2.0 * (qZeroYaw.x * qZeroYaw.x + qZeroYaw.y * qZeroYaw.y));
    double roll = atan2(2.0 * (qZeroYaw.w * qZeroYaw.z + qZeroYaw.x * qZeroYaw.y),
                        1.0 - 2.0 * (qZeroYaw.y * qZeroYaw.y + qZeroYaw.z * qZeroYaw.z));


//    double pitchQ = atan2(2.0 * (Q.w * Q.x + Q.y * Q.z), 1.0 - 2.0 * (Q.x * Q.x + Q.y * Q.y));
//    double rollQ = atan2(2.0 * (Q.w * Q.z + Q.x * Q.y), 1.0 - 2.0 * (Q.y * Q.y + Q.z * Q.z));
//
//    std::cout << "PitchQ: " << pitchQ * toDegrees << " RollQ: " << rollQ * toDegrees << std::endl;
//    std::cout << "Pitch: " << pitch * toDegrees << " Roll: " << roll * toDegrees << std::endl;

//            yaw *= toDegrees;
//            pitch *= toDegrees;
//            roll *= toDegrees;

    return QuadAngles(pitch, roll, angleYaw);
}
