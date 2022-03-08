#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main()
{
    //照片result1.jpg改变像素（尺寸大小）
    Mat src = imread("grab5.jpg");
    Size srcSize = Size(612, 512);  //填入任意指定尺寸
    resize(src, src, srcSize, 0, 0, INTER_LINEAR);
    cout << src.size() << endl;
    imshow("压缩图", src);
    imwrite("result5.jpg", src);  //保存图片
    waitKey(0);
    return 0;
}