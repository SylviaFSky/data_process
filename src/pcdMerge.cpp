#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>

using namespace std;
int main(){
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr input1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr input2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCDReader reader;
    if(reader.read("/home/xwy/WS/fisheye_ws/src/data_process/src/1.pcd", *input1) < 0){
        PCL_ERROR("File is not exist!");
        system("pause");
        return -1;
    }
    cout << "Input1 has " << input1->points.size() << "points" << endl;
    if(reader.read("/home/xwy/WS/fisheye_ws/src/data_process/src/2.pcd", *input2) < 0){
        PCL_ERROR("File is not exist!");
        system("pause");
        return -1;
    }
    cout << "Input2 has " << input2->points.size() << "points" << endl;

    for(int i = 0; i < input1->points.size(); i++){
        output->points.push_back(input1->points[i]);
    }
    for(int i = 0; i < input2->points.size(); i++){
        output->points.push_back(input2->points[i]);
    }
    cout << "Output has " << output->points.size() << "points" << endl;
    pcl::io::savePCDFileBinary("/home/xwy/WS/fisheye_ws/src/data_process/src/Output.pcd", *output);
    return 0;
}