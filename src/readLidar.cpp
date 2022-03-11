#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>     //Head files for pointcloud extraction

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>                //Head files for changing pointcloud into PCL format
#include <pcl/visualization/cloud_viewer.h>         //Head file for PCL visualizable


using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr convertByDepth(rosbag::View &view){        //Project by lidar depth information
    rosbag::View::iterator it = view.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);    // Define a ptr to read pointcloud information
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointXYZI p;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    float radius;
    float x;
    float y;
    float z;
    float theta;
    float R;
    for(;it != view.end();++it)
    {
        auto m = *it;
        sensor_msgs::PointCloud2::ConstPtr input = m.instantiate<sensor_msgs::PointCloud2>();
        pcl_conversions::toPCL(*input,pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);               // Chance into PCL format
        for(int pnum = 0;pnum < temp_cloud->points.size();pnum+=20){        // Traverse all the point in a message
            x = temp_cloud->points[pnum].x;
            if(x != 0){
                y = temp_cloud->points[pnum].y;
                z = temp_cloud->points[pnum].z;
                radius = sqrt(x*x + y*y + z*z);
                x = (x/radius);
                y = (y/radius);
                z = (z/radius);
                p.x = x;
                p.y = y;
                p.z = z;
                p.intensity = radius;
                cloud->points.push_back(p);
            }
        }
    }
    return cloud;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr convertByIntensity(rosbag::View &view){        //Project by lidar intensity information
    rosbag::View::iterator it = view.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointXYZI p;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    float radius;
    float x;
    float y;
    float z;
    float theta;
    float R;
    for(;it != view.end();++it)
    {
        auto m = *it;
        sensor_msgs::PointCloud2::ConstPtr input = m.instantiate<sensor_msgs::PointCloud2>();
        pcl_conversions::toPCL(*input,pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);  
        for(int pnum = 0;pnum < temp_cloud->points.size();pnum+=20){
            x = temp_cloud->points[pnum].x;
            if(x != 0){
                y = temp_cloud->points[pnum].y;
                z = temp_cloud->points[pnum].z;
                radius = sqrt(x*x + y*y + z*z);
                x = (x/radius);
                y = (y/radius);
                z = (z/radius);
                p.x = x;
                p.y = y;
                p.z = z;
                p.intensity = temp_cloud->points[pnum].intensity;
                cloud->points.push_back(p);
            }
        }
    }
    return cloud;
}
int main(int argc, char** argv){
    ros::init(argc, argv, "readLidar");             // Ros node initialization
    ros::NodeHandle nh;                             // NodeHandle initialization
    rosbag::Bag bag;
    string filePath = "/home/xwy/WS/fisheye_ws/src/data_process/data/20220223/";        //Should change the file path into your dataset
    string fileName = string(argv[1]);                                                  //Rosbag file name as the input
    int mode = atoi(argv[2]);                             // Decide projection mode: 0 for intensity, 1 for depth
    bag.open(filePath + fileName,rosbag::bagmode::Read);
    vector<std::string> topics;
    topics.push_back(string("/livox/lidar"));
    rosbag::View view(bag,rosbag::TopicQuery(topics));
    rosbag::View::iterator it = view.begin();
    pcl::visualization::CloudViewer viewer("Viewer");       // Visualize PCL pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    if(mode == 0){
        cloud = convertByIntensity(view);
    }
    else{
        cloud = convertByDepth(view);
    }
    viewer.showCloud(cloud);
    while (!viewer.wasStopped ())
    {

    }
    return 0;
}