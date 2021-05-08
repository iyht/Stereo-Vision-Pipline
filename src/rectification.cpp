
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char const *argv[])
{
    string left_calib_file_str = "../config/cam_left.yml";
    string right_calib_file_str = "../config/cam_right.yml";
    string extrinsic_file_str = "../config/extrinsic.yml";
    string out_file_str = "../config/result.yml";
    const char* leftcalib_file = left_calib_file_str.c_str();
    const char* rightcalib_file = right_calib_file_str.c_str();
    const char* extrinsic_file = extrinsic_file_str.c_str();
    const char* out_file = out_file_str.c_str();
    int width;
    int height;

    FileStorage fsl(leftcalib_file, FileStorage::READ);
    FileStorage fsr(rightcalib_file, FileStorage::READ);
    FileStorage fsx(extrinsic_file, FileStorage::READ);

    printf("Starting Calibration\n");
    Mat K1, K2, R, F, E;
    Mat T;
    Mat D1, D2;
    Mat width_mat, height_mat;
    fsl["K"] >> K1;
    fsr["K"] >> K2;
    fsl["D"] >> D1;
    fsr["D"] >> D2;
    fsx["R"] >> R;
    fsx["T"] >> T;

    fsl["width"] >> width_mat;
    fsl["height"] >> height_mat;

    width = static_cast<int>(width_mat.at<double>(0));
    height = static_cast<int>(height_mat.at<double>(0));

    Vec3d T_vec;
    T_vec[0] = T.at<double>(0);
    T_vec[1] = T.at<double>(1);
    T_vec[2] = T.at<double>(2);

    int flag = 0;
    flag |= CALIB_FIX_INTRINSIC;

    cout << "Read intrinsics" << endl;

    cout << "K1" << K1 << endl;
    cout << "K2" << K2 << endl;
    cout << "D1" << D1 << endl;
    cout << "D2" << D2 << endl;
    cout << "R" << R << endl;
    cout << "T" << T << endl;
    cout << "T_vec" << T_vec   << endl;

    cv::Mat R1, R2, P1, P2, Q;
    stereoRectify(K1, D1, K2, D2, Size(width, height), R, T_vec, R1, R2, P1, P2, Q);

    //// Calculate the undistortion and rectification transformation map
    //Mat map_x1, map_y1, map_x2, map_y2;
    //initUndistortRectifyMap(K1, D1, R1, P2, Size(width, height),CV_32FC1, map_x1, map_y1);
    //initUndistortRectifyMap(K2, D2, R2, P2, Size(width, height),CV_32FC1, map_x2, map_y2);


    cv::FileStorage fs1(out_file, cv::FileStorage::WRITE);

    fs1 << "K1" << K1;
    fs1 << "K2" << K2;
    fs1 << "D1" << D1;
    fs1 << "D2" << D2;
    fs1 << "R1" << R1;
    fs1 << "R2" << R2;
    fs1 << "P1" << P1;
    fs1 << "P2" << P2;
    fs1 << "Q" << Q;
    fs1 << "image_width" << width;
    fs1 << "image_height" << height;


    //fs1 << "map_x1" << map_x1;
    //fs1 << "map_y1" << map_y1;
    //fs1 << "map_x2" << map_x2;
    //fs1 << "map_y2" << map_y2;


    cout << "R1" << R1 << endl;
    cout << "R2" << R2 << endl;
    cout << "P1" << P1 << endl;
    cout << "P2" << P2 << endl;
    cout << "Q" << Q << endl;

    printf("Done Rectification\n");


    string left_result_str = "../config/left.yaml";
    const char* left_result = left_result_str.c_str();
    cv::FileStorage fl(left_result, cv::FileStorage::WRITE);
    fl << "image_width" << width;
    fl << "image_height" << height;
    fl << "camera_name" << "/stereo/left";
    fl << "camera_matrix" << K1;
    fl << "distortion_model" << "plumb_bob";
    fl << "distortion_coefficients" << D1;
    fl << "rectification_matrix" << R1;
    fl << "projection_matrix" << P1;


    string right_result_str = "../config/right.yaml";
    const char* right_result = right_result_str.c_str();
    cv::FileStorage fr(right_result, cv::FileStorage::WRITE);
    fr << "image_width" << width;
    fr << "image_height" << height;
    fr << "camera_name" << "/stereo/right";
    fr << "camera_matrix" << K2;
    fr << "distortion_model" << "plumb_bob";
    fr << "distortion_coefficients" << D2;
    fr << "rectification_matrix" << R2;
    fr << "projection_matrix" << P2;


    printf("Data save done\n");





    return 0;
}