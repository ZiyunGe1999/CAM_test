#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
// #include <pcl/io/stl_io.h>
#include <pcl/io/io.h>
#include <pcl/common/io.h>
#include <pcl/registration/icp.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

ros::Publisher pub;

void printPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
    for (const auto& point : cloud->points) {
        std::cout << "x: " << point.x << ", y: " << point.y << ", z: " << point.z 
                    << ", normal_x: " << point.normal_x << ", normal_y: " << point.normal_y << ", normal_z: " << point.normal_z
                    << std::endl;
    }
}

void alignClouds(const PointCloud::ConstPtr& stl_cloud, const PointCloud::ConstPtr& ply_cloud) {

    pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud(ply_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setKSearch(20);
    ne.compute(*sourceNormals);
    pcl::concatenateFields (*ply_cloud, *sourceNormals, *sourceNormals);

    pcl::PointCloud<pcl::PointNormal>::Ptr filtered_source(new pcl::PointCloud<pcl::PointNormal>);
    // std::vector<int> valid_indices;
    for (int i = 0; i < sourceNormals->size(); i++) {
        if (pcl::isFinite((*sourceNormals)[i])) {
            // valid_indices.push_back(i);
            filtered_source->push_back((*sourceNormals)[i]);
        }
    }
    // printPointCloud(filtered_source);
    pcl::io::savePLYFileASCII("/Q2-PointCloud_Alignment/src/data/source.ply", *filtered_source);

    pcl::PointCloud<pcl::PointNormal>::Ptr targetNormals(new pcl::PointCloud<pcl::PointNormal>);
    // pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud(stl_cloud);
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setKSearch(20);
    ne.compute(*targetNormals);
    pcl::concatenateFields (*stl_cloud, *targetNormals, *targetNormals);

    pcl::PointCloud<pcl::PointNormal>::Ptr filtered_target(new pcl::PointCloud<pcl::PointNormal>);
    // valid_indices.clear();
    for (int i = 0; i < targetNormals->size(); i++) {
        if (pcl::isFinite((*targetNormals)[i])) {
            // valid_indices.push_back(i);
            filtered_target->push_back((*targetNormals)[i]);
        }
    }
    // printPointCloud(filtered_target);
    pcl::io::savePLYFileASCII("/Q2-PointCloud_Alignment/src/data/target.ply", *filtered_target);

    ROS_INFO("Normals calculated");

    // Perform ICP
    // pcl::IterativeClosestPoint<PointT, PointT> icp;
    // icp.setInputSource(ply_cloud);
    // icp.setInputTarget(stl_cloud);
    // icp.align(aligned_cloud);

    // PointCloud aligned_cloud;
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
    icp.setInputSource (filtered_source);
    icp.setInputTarget (filtered_target);
    // icp.setSourceNormals (sourceNormals);
    // icp.setInputTargetNormals (targetNormals);
    pcl::PointCloud<pcl::PointNormal> aligned_cloud;
    icp.align (aligned_cloud);


    if (icp.hasConverged()) {
        ROS_INFO("ICP converged");
        ROS_INFO_STREAM("ICP score: " << icp.getFitnessScore());
    } else {
        ROS_WARN("ICP did not converge");
    }

    // Publish the aligned point cloud
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(aligned_cloud, output);
    pcl::io::savePLYFileASCII("/Q2-PointCloud_Alignment/src/data/aligned_cloud.ply", aligned_cloud);
    pub.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "icp_node");
    ros::NodeHandle nh;

    // Load the point clouds
    pcl::PolygonMesh stl_mesh;
    pcl::io::loadPolygonFileSTL("/Q2-PointCloud_Alignment/src/data/CAD.stl", stl_mesh);
    PointCloud::Ptr stl_cloud(new PointCloud());
    pcl::fromPCLPointCloud2(stl_mesh.cloud, *stl_cloud);
    // for (int i = 0; i < stl_cloud->size(); ++i) {
    // std::cout << "x: " << stl_cloud->points[i].x 
    //             << " y: " << stl_cloud->points[i].y 
    //             << " z: " << stl_cloud->points[i].z << std::endl;
    // }

    PointCloud::Ptr ply_cloud(new PointCloud());
    pcl::io::loadPCDFile<pcl::PointXYZ>("/Q2-PointCloud_Alignment/src/data/PointCloud.pcd", *ply_cloud);
    // pcl::PolygonMesh ply_mesh;
    // pcl::io::loadPolygonFile("/Q2-PointCloud_Alignment/src/data/PointCloud.ply", ply_mesh);
    // pcl::fromPCLPointCloud2(ply_mesh.cloud, *ply_cloud);
    // pcl::PLYReader Reader;
    // Reader.read("/Q2-PointCloud_Alignment/src/data/PointCloud.ply", *ply_cloud);
    // pcl::io::loadPLYFile<PointT>("/Q2-PointCloud_Alignment/src/data/PointCloud.ply", *ply_cloud);
    // for (int i = 0; i < ply_cloud->size(); ++i) {
    // std::cout << "x: " << ply_cloud->points[i].x 
    //             << " y: " << ply_cloud->points[i].y 
    //             << " z: " << ply_cloud->points[i].z << std::endl;
    // }


    // Publish the aligned point cloud to "/Question3/AlignedPointCloud"
    pub = nh.advertise<sensor_msgs::PointCloud2>("/Question2/AlignedPointCloud", 1);

    // Call ICP on the two point clouds
    alignClouds(stl_cloud, ply_cloud);

    ros::spin();
    return 0;
}
