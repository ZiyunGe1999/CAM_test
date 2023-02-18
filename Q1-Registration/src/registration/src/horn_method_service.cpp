#include <ros/ros.h>
#include <registration/HornMethod.h>

bool handle_function(registration::HornMethod::Request &req, registration::HornMethod::Response &res) {
    ROS_INFO("Received request. Processing frame1 (%s) and frame2 (%s)", req.frame1_path.c_str(), req.frame2_path.c_str());

    return true;
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "horn_method_service");
    ros::NodeHandle node_handle;
    ros::ServiceServer service = node_handle.advertiseService("horn_method", handle_function);

    ros::spin();

    return 0;
}