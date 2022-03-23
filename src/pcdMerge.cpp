#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>
#include <string>
#include <algorithm>
#include <sys/types.h> 
#include <dirent.h>
#include <typeinfo>
#include <iostream>

using namespace std;
#define GROUPSIZE 10    // You can change the size of a PCD group
#define MODE (S_IRWXU | S_IRWXG | S_IRWXO)

// Change your directory 
string inputDir = "/home/xwy/fisheye/Data/pcdOutput/";
string outputDir = "/home/xwy/WS/fisheye_ws/src/data_process/data/output/";

// Check if output directory exists, if not, create a new directory
void CheckOutputFolder(string outputDir){
    if(opendir(outputDir.c_str()) == NULL){             // The first parameter of 'opendir' is char *
        cout << outputDir << " not exsits!" << endl;
        int ret = mkdir(outputDir.c_str(), MODE);       // 'mkdir' used for creating new directory
    }
    else{
        cout << "outputDir already exists!" << endl;
    }
}

int readFileList(const std::string &folderPath,
                 std::vector<std::string> &vFileList)
{
    DIR *dp;
    struct dirent *dirp;
    if ((dp = opendir(folderPath.c_str())) == NULL)
    {
        return 0;
    }

    int num = 0;
    while ((dirp = readdir(dp)) != NULL)
    {
        std::string name = std::string(dirp->d_name);
        if (name != "." && name != "..")
        {
            vFileList.push_back(name);
            num++;
        }
    }
    closedir(dp);
    cout << "success!" << endl;

    return num;
}
int main(){
    pcl::PCDReader reader;      //used for read PCD files
    vector <string> nameList;
    CheckOutputFolder(outputDir);
    readFileList(inputDir, nameList);
    sort(nameList.begin(),nameList.end());      // sort file names by order
    int groupCount = nameList.size() / GROUPSIZE;
    
    // PCL PointCloud pointer. Remember that the pointer need to be given a new space
    pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
    int outputId = 0;
    int nameLength = GROUPSIZE * groupCount;
    auto nameIter = nameList.begin();
    for(int i = 0; i < groupCount; i++){
        for(int j = 0; j < GROUPSIZE; j++){
            string fileName = inputDir + *nameIter;
            cout << fileName << endl;
            if(reader.read(fileName, *input) < 0){      // read PCD files, and save PointCloud in the pointer
                PCL_ERROR("File is not exist!");
                system("pause");
                return -1;
            }
            int pointCount = input -> points.size();
            for(int k = 0; k < pointCount; k++){
                output -> points.push_back(input -> points[k]);
            }
            nameIter++;         
        }
        pcl::io::savePCDFileBinary(outputDir + to_string(i) + ".pcd", *output);     // Save PCD files with destination file name and PointCloud pointer
    }
    return 0;
}