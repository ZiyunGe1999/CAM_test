#ifndef REGISTRATION_HELPER_FUNCTIONS_H
#define REGISTRATION_HELPER_FUNCTIONS_H

#include <Eigen/Dense>

Eigen::Quaterniond matrixToQuaternion(const Eigen::Matrix3d& R);

Eigen::Vector3d matrixToEuler(const Eigen::Matrix3d& R);

Eigen::Matrix3d quaternionToMatrix(const Eigen::Quaterniond& q);

Eigen::Matrix3d eulerToMatrix(double roll, double pitch, double yaw);

Eigen::Matrix4d inverse4dMatrix(const Eigen::Matrix4d &T);

#endif