#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <registration/HornMethod.h>
#include <registration/helper_functions.h>
#include<Eigen/Dense>
#include <Eigen/Geometry>

bool handle_function(registration::HornMethod::Request &req, registration::HornMethod::Response &res) {
    ROS_INFO("Received request. Processing frame1 (%s) and frame2 (%s)", req.frame1_path.c_str(), req.frame2_path.c_str());

    std::ifstream frame1_file(req.frame1_path);
    if (!frame1_file.is_open()) {
        ROS_ERROR("Failed to open frame 1 file at %s", req.frame1_path.c_str());
        return false;
    }
    std::ifstream frame2_file(req.frame2_path);
    if (!frame2_file.is_open()) {
        ROS_ERROR("Failed to open frame 2 file at %s", req.frame2_path.c_str());
        frame1_file.close();
        return false;
    }

    auto parseCsvFile = [](std::ifstream &file, Eigen::MatrixXd &points) {
        std::string line;
        std::getline(file, line);
        std::vector<std::vector<double>> points_vec;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string word;
            std::vector<double> point_vec;
            while (!ss.eof()) {
                std::getline(ss, word, ',');
                point_vec.push_back(std::stod(word));
            }
            if (point_vec.size() != 3) {
                ROS_ERROR("Point data must have three coordinates");
                return false;
            }
            points_vec.push_back(std::move(point_vec));
        }

        points = Eigen::MatrixXd(points_vec.size(), 3);
        for (int i = 0; i < points_vec.size(); i++) {
            points.row(i) = Eigen::VectorXd::Map(&points_vec[i][0], 3);
        }
        points.transposeInPlace();
        return true;
    };

    Eigen::MatrixXd points1;
    Eigen::MatrixXd points2;
    if (!parseCsvFile(frame1_file, points1) || !parseCsvFile(frame2_file, points2)) {
        frame1_file.close();
        frame2_file.close();
        return false;
    }
    frame1_file.close();
    frame2_file.close();

    Eigen::Vector3d frame1_centroid = points1.rowwise().mean();
    Eigen::Vector3d frame2_centroid = points2.rowwise().mean();

    Eigen::MatrixXd centered_frame1 = points1.colwise() - frame1_centroid;
    Eigen::MatrixXd centered_frame2 = points2.colwise() - frame2_centroid;

    Eigen::MatrixXd covariance = centered_frame1 * centered_frame2.transpose();

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    Eigen::Matrix3d R = V * U.transpose();
    Eigen::Vector3d t = frame2_centroid - R * frame1_centroid;

    // Eigen::Matrix4d T = Eigen::Matrix4d::Identity();;
    // T.block<3,3>(0,0) = R;
    // T.block<3,1>(0,3) = t;
    // std::cout << T << std::endl;
    // std::cout << T.inverse() << std::endl;

    res.pose.position.x = t[0];
    res.pose.position.y = t[1];
    res.pose.position.z = t[2];
    Eigen::Quaterniond quat(R);
    res.pose.orientation.x = quat.x();
    res.pose.orientation.y = quat.y();
    res.pose.orientation.z = quat.z();
    res.pose.orientation.w = quat.w();

    return true;
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "horn_method_service");
    ros::NodeHandle node_handle;
    ros::ServiceServer service = node_handle.advertiseService("horn_method", handle_function);

    ros::spin();

    return 0;
}