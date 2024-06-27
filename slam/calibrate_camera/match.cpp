// https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html
// https://learnopencv.com/depth-perception-using-stereo-camera-python-c/
#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <boost/format.hpp>  // for formating strings

using namespace std;
using namespace cv;

int main(){

    Mat imgL = imread("../rec/rec_left.jpg",cv::IMREAD_GRAYSCALE);
    Mat imgR = imread("../rec/rec_right.jpg",cv::IMREAD_GRAYSCALE);
    imshow("left",imgR);
    waitKey(0);
    int numDisparities=16;
    int blockSize=21;
    static Ptr<StereoBM> stereoBM = StereoBM::create(numDisparities,blockSize);

    Mat disparity;
    stereoBM->compute(imgL,imgR,disparity);

    Mat imgDisparity8U;
    double minVal, maxVal;   
    minMaxLoc(disparity, &minVal, &maxVal);
    disparity.convertTo(imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));
    //imgDisparity16S.convertTo(imgDisparity8U, CV_8U);
    //namedWindow("windowDisparity", WINDOW_NORMAL);
    imshow("windowDisparity", imgDisparity8U);
    // imshow("disparity",disparity);
    waitKey(0);
    return 0;
}
