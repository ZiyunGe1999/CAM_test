#include <registration/helper_functions.h>

Eigen::Quaterniond matrixToQuaternion(const Eigen::Matrix3d& R)
{
    Eigen::Quaterniond q(R);
    return q;
}

Eigen::Vector3d matrixToEuler(const Eigen::Matrix3d& R)
{
    double sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2(R(2,1) , R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2(R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return Eigen::Vector3d(x, y, z);
}

Eigen::Matrix3d quaternionToMatrix(const Eigen::Quaterniond& q)
{
    Eigen::Matrix3d R = q.toRotationMatrix();
    return R;
}

Eigen::Matrix3d eulerToMatrix(double roll, double pitch, double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d R = q.toRotationMatrix();
    return R;
}

Eigen::Matrix4d inverse4dMatrix(const Eigen::Matrix4d &T) {
    return T.inverse();
}
