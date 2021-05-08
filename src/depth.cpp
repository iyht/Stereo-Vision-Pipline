//
// Created by haotian on 2020-12-11.
//
#include <opencv2/core.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include "opencv2/calib3d.hpp"
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>

#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudacodec.hpp>
#include <opencv2/cudastereo.hpp>

#include <iostream>
#include <stdio.h>
#include <string>
#include <time.h>
#include <cmath>
#include <stdlib.h>
#include <libsgm.h>
#include <libsgm_wrapper.h>

#include <cuda_runtime.h>
#include <cuda.h>



using namespace std;
using namespace cv::ximgproc;
using namespace  cv;


#define ASSERT_MSG(expr, msg) \
if (!(expr)) { \
    std::cerr << msg << std::endl; \
    std::exit(EXIT_FAILURE); \
} \

struct device_buffer
{
    device_buffer() : data(nullptr) {}
    device_buffer(size_t count) { allocate(count); } void allocate(size_t count) { cudaMalloc(&data, count); }
    ~device_buffer() { cudaFree(data); }
    void* data;
};

// SGM
const int disp_size = 64;
int P1 = 44;
int P2 = 217;
int minDisparity = 1;
int uniqueness = 1;
int num_paths = 8;
int LR_max_diff = 1;
sgm::PathType path_type = num_paths == 8 ? sgm::PathType::SCAN_8PATH : sgm::PathType::SCAN_4PATH;

// SGBM
int SGBM_blocksize = 2;
int SGBM_minDisparity = 0;
int SGBM_P1 = 8*3*pow(SGBM_blocksize, 2);
int SGBM_P2 = 32*3*pow(SGBM_blocksize, 2);
int SGBM_numDisparity = 50;
int SGBM_Uniqueness = 1;
int SGBM_SpeckleWindowSize = 100;
int SGBM_SpeckleRange = 32;
int SGBM_Disp12MaxDiff = 5;
int SGBM_PreFilterCap = 63;


void SGBM(cv::Mat& grayL, cv::Mat& grayR, cv::Mat& disparity)
{
    Ptr<StereoSGBM> left_matcher = StereoSGBM::create(0,16,3);
    left_matcher->setMode(StereoSGBM::MODE_SGBM);
    left_matcher->setP1(SGBM_P1);
    left_matcher->setP2(SGBM_P2);
    left_matcher->setMinDisparity(SGBM_minDisparity);
    left_matcher->setNumDisparities(SGBM_numDisparity);
    left_matcher->setUniquenessRatio(SGBM_Uniqueness);
    left_matcher->setSpeckleWindowSize(SGBM_SpeckleWindowSize);
    left_matcher->setSpeckleRange(SGBM_SpeckleRange);
    left_matcher->setDisp12MaxDiff(SGBM_Disp12MaxDiff);
    left_matcher->setPreFilterCap(SGBM_PreFilterCap);
    left_matcher->setBlockSize(SGBM_blocksize);

    Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
    cv::Mat left_disparity, right_disparity, non_colored_dis;
    left_matcher->compute(grayR, grayL, left_disparity);

    left_disparity.convertTo(non_colored_dis, CV_8U, 255/(SGBM_numDisparity*16.));

    cv::namedWindow("disparity noncolored", cv::WINDOW_AUTOSIZE);

//	setMouseCallback("disparity noncolored", onMouse, &non_colored_dis);
    imshow("disparity noncolored", non_colored_dis);
    right_matcher->compute(grayR, grayL, right_disparity);

    Ptr<DisparityWLSFilter> wls_filter = createDisparityWLSFilter(left_matcher);
    wls_filter->filter(left_disparity,grayL,disparity,right_disparity);
    //normalize(disparity, disparity, 255, 0, NORM_MINMAX);

    disparity.convertTo(disparity, CV_8U, 255/(SGBM_numDisparity*16.));



}


void SGMCUDA(cv::Mat& left, cv::Mat& right, cv::Mat& disparity)
{
    
    int disp_size = 128;
    cv::cuda::GpuMat d_left(left);
    cv::cuda::GpuMat d_right(right);

    cv::cuda::GpuMat d_left_gray, d_right_gray, d_disparity;
    cv::cuda::cvtColor(d_left, d_left_gray, cv::COLOR_BGR2GRAY);
    cv::cuda::cvtColor(d_right, d_right_gray, cv::COLOR_BGR2GRAY);
    cout << d_left.type() << endl;
    cout << d_right.type() << endl;
    //ASSERT_MSG(!grayL.empty() && !grayR.empty(), "imread failed.");
    //ASSERT_MSG(grayL.size() == grayR.size() && grayL.type() == grayR.type(), "input images must be same size and type.");
    //ASSERT_MSG(grayL.type() == CV_8U || grayL.type() == CV_16U, "input image format must be CV_8U or CV_16U.");
    //ASSERT_MSG(disp_size == 64 || disp_size == 128, "disparity size must be 64 or 128.");

	//params
	int width = left.cols;
	int height = left.rows;
	const int input_depth = left.type() == CV_8U ? 8 : 16;
    const int input_bytes = input_depth * width * height / 8;
    const int output_depth = 16;
    const int output_bytes = output_depth * width * height / 8;
	sgm::StereoSGM::Parameters param(P1, P2, uniqueness/10, false, path_type, minDisparity, LR_max_diff);
    sgm::StereoSGM sgm(width, height, disp_size, input_depth, output_depth, sgm::EXECUTE_INOUT_CUDA2CUDA, param);

    sgm::LibSGMWrapper sgmw(128);

    //device_buffer d_grayL(input_bytes), d_grayR(input_bytes), d_disparity(output_bytes);
    //cv::Mat left_disparity(height, width, output_depth == 8 ? CV_8U : CV_16U);

	//left disparity
    //cv::Mat leftR = grayL.clone();
    //cv::Mat rightR = grayR.clone();
    //cudaMemcpy(d_grayL.data, leftR.data, input_bytes, cudaMemcpyHostToDevice);
    //cudaMemcpy(d_grayR.data, rightR.data, input_bytes, cudaMemcpyHostToDevice);
    sgmw.execute(d_left_gray, d_right_gray, d_disparity);
//cudaMemcpy(left_disparity.data, d_disparity.data, output_bytes, cudaMemcpyDeviceToHost);

    d_disparity.convertTo(d_disparity, CV_32F, 1.0/16.0);
    if (d_disparity.empty()) {
        std::cerr << "compute disparity failed!\n" << std::endl;
    }
    d_disparity.download(disparity);
    //cout << disparity << endl;
    disparity.convertTo(disparity, CV_8U);
    double min = 0;
    double max = 150;
    disparity = (255*(disparity-min))/(max-min);
    cv::applyColorMap(disparity,disparity, cv::COLORMAP_JET);

    //right disparity
//    device_buffer d_flippedRight(input_bytes), d_flippedLeft(input_bytes), d_RightDisparity(output_bytes);
//    cv::Mat rightDisparity(grayR.size(), CV_16S);
//	cv::Mat flippedRightDisparity(grayR.size(), CV_16S);
//    cv::Mat flippedLeft(grayL.size(), CV_8U);
//    cv::Mat flippedRight(grayR.size(), CV_8U);
//    cv::flip(grayL, flippedLeft, 1);
//    imshow("flipL", flippedLeft);
//    cv::flip(grayR, flippedRight, 1);
//    imshow("flipR", flippedRight);
//    cudaMemcpy(d_flippedLeft.data, flippedLeft.data, input_bytes, cudaMemcpyHostToDevice);
//    cudaMemcpy(d_flippedRight.data, flippedRight.data, input_bytes, cudaMemcpyHostToDevice);
//
//    sgm::StereoSGM sgmr(width, height, disp_size, input_depth, output_depth, sgm::EXECUTE_INOUT_CUDA2CUDA, param);
//    sgmr.execute(d_flippedRight.data, d_flippedLeft.data, d_RightDisparity.data);
//
//    cudaMemcpy(rightDisparity.data, d_RightDisparity.data, output_bytes, cudaMemcpyDeviceToHost);
    // flip back
//    flip(rightDisparity, flippedRightDisparity, 1);
//
//    cv::Mat disparity_8u, disparity_color;
//    left_disparity.convertTo(disparity_8u, CV_8U, 255. / (disp_size));
//    cv::imshow("dis", disparity_8u);
    
    // WSL
    //useWSLfilter(grayL, left_disparity, flippedRightDisparity, disparity);
        
    // Bilateral
    //useBilateralfilter(left_disparity, disparity);
    
    // Normalize
    //normalize(disparity, disparity, 255, 0, NORM_MINMAX);
//    disparity.convertTo(disparity, CV_8U, 255. /(disp_size));
    
}


int main(int argc, char **argv) {


    //load the map
    cv::FileStorage fs("./config/result.yml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        cout << "Could not open the extrinsics file" << endl;
        exit(1);
    }

    cv::Mat K1, K2, D1, D2, R1, R2, P1, P2, Q;

    int image_width = fs["image_width"];
    int image_height = fs["image_height"];
    fs["K1"] >> K1;
    fs["K2"] >> K2;
    fs["D1"] >> D1;
    fs["D2"] >> D2;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q;



    // Calculate the undistortion and rectification transformation map
    cv::Mat map_x1, map_y1, map_x2, map_y2;
    initUndistortRectifyMap(K1, D1, R1, P1, cv::Size(image_width, image_height), CV_32FC1, map_x1, map_y1);
    initUndistortRectifyMap(K2, D2, R2, P2, cv::Size(image_width, image_height), CV_32FC1, map_x2, map_y2);

    //std::cout << map_x1 << std::endl;
    //std::cout << map_x2 << std::endl;

    cv::VideoCapture cap(atoi(argv[1]), cv::CAP_V4L);
    if (!cap.isOpened()){
        cerr << "Error! Unable to open camera " << endl;
        exit(-1);
    }
    
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    std::cout << "FPS\n" << cap.get(cv::CAP_PROP_FPS) << std::endl;
    cv::Mat full, left, right;


    cv::namedWindow("disparity", cv::WINDOW_AUTOSIZE);
    //cv::createTrackbar("P1", "disparity", &P1, 500);
    //cv::createTrackbar("P1", "disparity", &P1, 500);
    //cv::createTrackbar("P2", "disparity", &P2, 500);
    //cv::createTrackbar("minDisparity", "disparity", &minDisparity, 100);
    //cv::createTrackbar("uniqueness", "disparity", &uniqueness, 100);

    //cv::createTrackbar("WSLsigma", "disparity", &WSLsigma, 20);
    //cv::createTrackbar("WSLlambda", "disparity", &WSLlambda, 1000000);
    //cv::createTrackbar("Bi_kernel_size", "disparity", &Bi_kernel_size, 20);
    //cv::createTrackbar("Bi_sigmacolor", "disparity", &Bi_sigmacolor, 500);
    //cv::createTrackbar("Bi_sigmaspatial", "disparity", &Bi_sigmaspatial, 500);

    cv::createTrackbar("SGBM_blocksize", "disparity", &SGBM_blocksize, 30);
    cv::createTrackbar("SGBM_minDisparity", "disparity", &SGBM_minDisparity, 50);
    cv::createTrackbar("SGBM_Uniqueness", "disparity", &SGBM_Uniqueness, 50);
    cv::createTrackbar("SGBM_numDisparity", "disparity", &SGBM_numDisparity, 300);


// check if both camear
    if (!cap.isOpened()) {
        cerr << "Error! Unable to open camera " << endl;
        return -1;
    }
    cv::Mat frameR, frameL, frameFull, frameR_rectified, frameL_rectified, disp;
    for (;;) {

        cap >> frameFull;
        frameL = frameFull(cv::Rect(0, 0, 640, 480));
        frameR = frameFull(cv::Rect(640, 0, 640, 480));
        cv::remap(frameR, frameR_rectified, map_x2, map_y2, cv::INTER_LINEAR);
        cv::remap(frameL, frameL_rectified, map_x1, map_y1, cv::INTER_LINEAR);

        //SGBM(frameR_rectified, frameL_rectified, disp);
        SGMCUDA(frameL_rectified, frameR_rectified, disp);
        imshow("disparity", disp);

        imshow("Right", frameR);
        imshow("Left", frameL);
        imshow("Right_rect", frameR_rectified);
        imshow("Left_rect", frameL_rectified);
        cv::waitKey(1);

    }

}

