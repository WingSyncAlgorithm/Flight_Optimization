//https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html
//https://learnopencv.com/camera-calibration-using-opencv/
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

vector<int> is_number(string s) {
    /*Read the numbers from a string and store them sequentially in a vector.*/
    int count = 0;
    vector<int> v_int;
    for (char c : s) {
        if (isdigit(c)) {
            v_int.push_back(int(c-'0'));
        }
    }
    return v_int;
}

int getdir(string dir, map<int,string> &files,string foldname){

    /*Read the file names inside a folder.And sort the file names according to the numbers within them, storing them in a vector to return.*/

    DIR *dp;//創立資料夾指標
    struct dirent *dirp;
    if((dp = opendir(dir.c_str())) == NULL){
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }
    while((dirp = readdir(dp)) != NULL){//如果dirent指標非空
        boost::format fmt( "../%s/%s" );
        string str_temp = string(dirp->d_name);
        vector<int> vec_int = is_number(str_temp);
        int ind;
        if(!vec_int.empty()){
            ind = vec_int[0]*10 + vec_int[1];
        }
        if(str_temp.compare("..") != 0 && str_temp.compare(".") != 0){
            string f_patn_name = (fmt%foldname%str_temp).str();
            // files.push_back(f_patn_name);//將資料夾和檔案名放入vector
            files[ind] = f_patn_name;
        }
    }
    closedir(dp);//關閉資料夾指標
    return 0;
}

int map_to_vec(vector<string> &files_vec, map<int,string> &files){
    /*from map to vector*/
    for (const auto& s : files){
        if(s.second != ""){
            files_vec.push_back(s.second);
        }
    }
    return 0;
}

int main(){

    string dir_left = string("../left");
    string dir_right = string("../right");
    map<int,string> imageFiles_left_map;
    map<int,string> imageFiles_right_map;
    getdir(dir_left,imageFiles_left_map,"left");
    getdir(dir_right,imageFiles_right_map,"right");
    vector<string> imageFiles_left;
    vector<string> imageFiles_right;
    map_to_vec(imageFiles_left,imageFiles_left_map);
    map_to_vec(imageFiles_right,imageFiles_right_map);
    // imageFiles_left.push_back("../lefttest.jpg");
    int numImages = imageFiles_left.size();

    printf("find corner point left\n");
    vector<vector<Point2f>> imagePoints_seq_left;
    vector<vector<Point2f>> imagePoints_seq_right;
    Size boardSize(6, 9);

    Mat gray_left;
    for(int i = 0; i < numImages; i++){
        Mat image = imread(imageFiles_left[i]);
        cout<<imageFiles_left[i];
        cvtColor(image, gray_left, COLOR_BGR2GRAY);
        vector<Point2f> corners;
        bool found = findChessboardCorners(gray_left, boardSize, corners);
        if(found){
            printf(" found!\n");
            cornerSubPix(gray_left, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
            drawChessboardCorners(image, boardSize, corners, found);
            imshow("find",image);
            imagePoints_seq_left.push_back(corners);
            //waitKey(500);
        }
    }
    printf("find corner point right\n");
    Mat gray_right;
    for(int i = 0; i < numImages; i++){
        Mat image = imread(imageFiles_right[i]);
        cout<<imageFiles_right[i];
        cvtColor(image, gray_right, COLOR_BGR2GRAY);
        vector<Point2f> corners;
        bool found = findChessboardCorners(gray_right, boardSize, corners);
        if(found){
            printf(" found!\n");
            cornerSubPix(gray_right, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
            drawChessboardCorners(image, boardSize, corners, found);
            imshow("find",image);
            imagePoints_seq_right.push_back(corners);
            //waitKey(500);
        }
    }

    printf("start cali left\n");
    Size square_size_left = gray_left.size();
    vector<vector<Point3f>> object_point_left;
    Mat camera_matrix_left = Mat(3,3,CV_64F,Scalar::all(0));//intrinsics matrix
    Mat discoeffs_left = Mat(1,5,CV_64F,Scalar::all(0)); //distortion coefficients
    vector<Mat> tvec_left;
    vector<Mat> rvec_left;
    for(int t = 0; t < numImages; t++){
        vector<Point3f> temp_point_set;
        for(int i = 0; i < boardSize.height;i++){
            for(int j = 0;j < boardSize.width;j++){
                Point3f realpoint;
                realpoint.x = i*square_size_left.width;
                realpoint.y = j*square_size_left.height;
                realpoint.z = 0;
                temp_point_set.push_back(realpoint);
            }
        }
        object_point_left.push_back(temp_point_set);
    }
    calibrateCamera(object_point_left,imagePoints_seq_left,square_size_left,camera_matrix_left,discoeffs_left,tvec_left,rvec_left);
    std::cout << "cameraMatrix_left : " <<endl<< camera_matrix_left << std::endl;
    std::cout << "distCoeffs_left : "<<endl << discoeffs_left << std::endl;
    std::cout << "Rotation vector : "<<endl << rvec_left[0] << std::endl;
    std::cout << "Translation vector : "<<endl << tvec_left[0] << std::endl;  

    printf("start cali right\n");
    Size square_size_right = gray_right.size();
    vector<vector<Point3f>> object_point_right;
    Mat camera_matrix_right = Mat(3,3,CV_64F,Scalar::all(0));//intrinsics matrix
    Mat discoeffs_right = Mat(1,5,CV_64F,Scalar::all(0)); //distortion coefficients
    vector<Mat> tvec_right;
    vector<Mat> rvec_right;
    for(int t = 0; t < numImages; t++){
        vector<Point3f> temp_point_set;
        for(int i = 0; i < boardSize.height;i++){
            for(int j = 0;j < boardSize.width;j++){
                Point3f realpoint;
                realpoint.x = i*square_size_right.width;
                realpoint.y = j*square_size_right.height;
                realpoint.z = 0;
                temp_point_set.push_back(realpoint);
            }
        }
        object_point_right.push_back(temp_point_set);
    }
    calibrateCamera(object_point_right,imagePoints_seq_right,square_size_right,camera_matrix_right,discoeffs_right,tvec_right,rvec_right);
    std::cout << "cameraMatrix_right : " <<endl<< camera_matrix_right << std::endl;
    std::cout << "distCoeffs_right : "<<endl << discoeffs_right << std::endl;
    std::cout << "Rotation vector : "<<endl << rvec_right[0] << std::endl;
    std::cout << "Translation vector : "<<endl << tvec_right[0] << std::endl; 

    cout<<"start cali stereo"<<endl;
    Mat R,T,E,F;
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    stereoCalibrate(object_point_left,imagePoints_seq_left,imagePoints_seq_right,camera_matrix_left,discoeffs_left,camera_matrix_right,discoeffs_right,square_size_left,R,T,E,F,cv::CALIB_FIX_INTRINSIC);
    cout<<"R"<<R<<endl;
    cout<<"T"<<T<<endl;
    cout<<"E"<<E<<endl;
    cout<<"F"<<F<<endl;
    // Declare what you need
    cv::FileStorage file("../cali_params.ext", cv::FileStorage::WRITE);
    // Write to file!
    file << "camera_matrix_left" << camera_matrix_left;
    file << "camera_matrix_right"<< camera_matrix_right;
    file << "discoeffs_left" << discoeffs_left;
    file << "discoeffs_right" << discoeffs_right;
    file<<"R"<<R;
    file<<"T"<<T;
    file<<"E"<<E;
    file<<"F"<<F;
    // Close the file and release all the memory buffers
    file.release();
}
