#include <iostream>

#include <cmath>
#include <math.h>
#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>   
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#define PI 3.1415926
using namespace std;
using namespace cv;


int main(int argc, char** argv){
    ros::init(argc, argv, "readImage");
    ros::NodeHandle nh;
    string imageName = string(argv[1]);
    Mat img = imread("/home/xwy/WS/fisheye_ws/src/data_process/data/Image/" + imageName,1);
    int rows = img.rows;
    int cols = img.cols;
    pcl::PointXYZRGB p;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kd_input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::CloudViewer viewer("Viewer");
    int Blue, Green, Red;
    int x, y, z;
    double radius_temp;
    double radius;
    double t;
    double t1;
    double theta;

    float a = atof(argv[2]), b = atof(argv[3]), c = atof(argv[4]);
    t1 = a + b + c;
    for(int i = 0 ; i < rows; i++){

        for(int j = 0; j < cols ; j++){
            if(pow((i-1024),2) + pow((j-1124),2) <= pow(1024,2)){
                Blue = img.at<Vec3b>(i, j)[0];
                Green = img.at<Vec3b>(i, j)[1];
                Red = img.at<Vec3b>(i, j)[2];
                x = i-1024;
                y = j-1124;
                radius_temp = sqrt(pow(x, 2) + pow(y,2)); //点在平面上的半径
                t = (1024 - radius_temp )/ 1024;  //半径越大投影角度越小
                theta = (c + b * t + a * pow(t, 2)) / t1 * PI /2;
                // theta = acos(radius_temp / 1024);
                p.z = 1024 * sin(theta);
                radius = 1024 * cos(theta);
                p.x = x * radius / radius_temp;
                p.y = y * radius / radius_temp;
                p.r = Red;
                p.g = Green;
                p.b = Blue;
                kd_input_cloud->points.push_back(p);
            }
        }
            
    }
    viewer.showCloud(kd_input_cloud);
    while (!viewer.wasStopped ())
    {

    }
    return 0;
}