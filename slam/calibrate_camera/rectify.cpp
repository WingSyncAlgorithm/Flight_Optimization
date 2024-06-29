// https://shengyu7697.github.io/opencv-filestorage/
// https://blog.csdn.net/weixin_43196818/article/details/129184280
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

using namespace cv;
using namespace std;

int main(){
    // read image
    Mat left_img = imread("../picture/toyleft.jpg");
    resize(left_img,left_img,Size(290,220));
    Mat gray_left;
    cvtColor(left_img, gray_left, COLOR_BGR2GRAY);
    Mat right_img = imread("../picture/toyright.jpg");
    resize(right_img,right_img,Size(290,220));
    Mat gray_right;
    cvtColor(right_img, gray_right, COLOR_BGR2GRAY);
    FileStorage file("../cali_params.txt", cv::FileStorage::READ);
    // read matrix
    Mat camera_matrix_left,discoeffs_left,camera_matrix_right,discoeffs_right;
    Mat R,T,E,F;
    file ["camera_matrix_left"]>> camera_matrix_left;
    file [ "discoeffs_left" ]>> discoeffs_left;
    file [ "camera_matrix_right"]>> camera_matrix_right;
    file [ "discoeffs_right" ]>>discoeffs_right;
    file["R"]>>R;
    file["T"]>>T;
    file["E"]>>E;
    file["F"]>>F;
    //recificate
    Size img_size = gray_left.size();
    Mat Rl,Rr;
    Mat Pl,Pr;
    Mat Q;
    stereoRectify(camera_matrix_left,discoeffs_left,camera_matrix_right,discoeffs_right,img_size,R,T,Rl,Rr,Pl,Pr,Q);
    //remap image
    Mat map1_l,map2_l;
    Mat map1_r,map2_r;
    initUndistortRectifyMap(camera_matrix_left,discoeffs_left,Rl,Pl,img_size,CV_16SC2,map1_l,map2_l);
    initUndistortRectifyMap(camera_matrix_right,discoeffs_right,Rr,Pr,img_size,CV_16SC2,map1_r,map2_r);
    Mat rec_left,rec_right;
    remap(gray_left,rec_left,map1_l,map2_l,INTER_LINEAR);
    remap(gray_right,rec_right,map1_r,map2_r,INTER_LINEAR);
    //see origin
    Mat combine_img_origin;
    hconcat(left_img,right_img,combine_img_origin);
    int w_origin = combine_img_origin.cols;
    for(int i=1;i<=8;i++){
        int h=combine_img_origin.rows/8*i;
        line(combine_img_origin,Point2i(0,h),Point2i(w_origin,h),Scalar(0,0,255));
    }
    imshow("combine_origin",combine_img_origin);
    waitKey(0);
    //see result
    Mat combine_img;
    hconcat(rec_left,rec_right,combine_img);
    cvtColor(combine_img,combine_img,COLOR_GRAY2BGR);
    int w = combine_img.cols;
    for(int i=1;i<=8;i++){
        int h=combine_img.rows/8*i;
        line(combine_img,Point2i(0,h),Point2i(w,h),Scalar(0,0,255));
    }
    imshow("combine",combine_img);
    waitKey(0);
    imwrite("../rec/rec_left.jpg",rec_left);
    imwrite("../rec/rec_right.jpg",rec_right);
    return 0;
}