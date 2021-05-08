#include <opencv2/core.hpp>
#include "opencv2/calib3d.hpp"
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/cudacodec.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>
#include <stdio.h>
#include <string>
#include <unistd.h>


using namespace cv;
using namespace std;

int main(int argc, char** argv)
{


    int c;
    int w; // weight of the board
    int h; // height of the board
    char *d;
    while ((c = getopt(argc, argv, "h:w:d:")) != -1) {
        switch(c)
            {
                case 'w':
                    w = atoi(optarg);
                    break;
                case 'h':
                    h = atoi(optarg);
                    break;
                case 'd':
                    d = optarg;
                    break;
                default:
                    fprintf(stderr, "Usage: ./read -w <board width> -h <borad height> -d <directory to store the image>");
                    exit(1);
            }
    }

    std::string saving_dir(d);

    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1);


    cv::VideoCapture cap(0, CAP_V4L);
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    std::cout << "FPS\n" << cap.get(CAP_PROP_FPS) << std::endl;
    Mat full, left, right;
 

    // check if both camear
    if (!cap.isOpened()){
        cerr << "Error! Unable to open camera " << endl;
        return -1;
    }

    cout << "Start grabbing" << endl
        << "Press s save image, space to terminate, other key to skip." << endl;

    cv::Mat frameR, frameL, frameFull;
    Size patternsize(h,w);
    vector<Point2f> cornersL;
    vector<Point2f> cornersR;

    int Saved = 0;
    for(;;)
    {

        cap >> frameFull;
        frameL = frameFull(Rect(0,0,640,480));
        frameR = frameFull(Rect(640,0,640,480));

        imshow("Right", frameR);
        imshow("Left", frameL);
        waitKey(1);


        cv::Mat grayR;
        cv::Mat grayL;

        cvtColor(frameR, grayR, cv::COLOR_BGR2GRAY);
        cvtColor(frameL, grayL, cv::COLOR_BGR2GRAY);

        
        bool foundL = findChessboardCorners(grayL, patternsize, cornersL);
        bool foundR = findChessboardCorners(grayR, patternsize, cornersR);

        if (foundL && foundR)
        {
            cornerSubPix(grayL, cornersL, Size(11,11), Size(-1,-1), criteria);
            cornerSubPix(grayR, cornersR, Size(11,11), Size(-1,-1), criteria);
            drawChessboardCorners(frameR, patternsize, Mat(cornersR), foundR);
            drawChessboardCorners(frameL, patternsize, Mat(cornersL), foundL);
            imshow("ChessboardL", grayL);
            imshow("ChessboardR", grayR);
            if ((char) waitKey(1) == 's')
            {
                Saved++ ;
                imwrite(saving_dir + "/right/chessboard-R" + std::to_string(Saved) + ".png", grayR);
                imwrite(saving_dir + "/left/chessboard-L" + std::to_string(Saved) + ".png", grayL);
                cout << " Saved the " << Saved <<"pairs" << endl;
            }
            else
            {
                cout << "Skip" << endl;
            }
        }

        if ((char) waitKey(1) == ' ')
        {
            break;
        }
    }
    cap.release();
    destroyAllWindows();
    return 0;
}

