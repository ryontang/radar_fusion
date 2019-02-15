#include <vector>

// Module "core"
#include <opencv2/core/core.hpp>


// Module "highgui"
#include <opencv2/highgui/highgui.hpp>

// Module "imgproc"s
#include <opencv2/imgproc/imgproc.hpp>

// Module "video"
#include <opencv2/video/video.hpp>

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>

using namespace cv;


class kf_var
{
public :
    int track_id; // num we can know if the object is match

    int stateSize = 4;
    int measSize = 2;
    int contrSize = 0;
    
    float T=60*0.001; //T : Operating rate of the system
    float test1;
    // unsigned int type = CV_32F;


    // KalmanFilter KF(int 2, int 1, 0);
   
    Mat state = cv::Mat::zeros(4, 1, CV_32F);
    Mat meas = cv::Mat::ones(2, 1, CV_32F);    // [v_rx,v_ry]


    // cv::KalmanFilter kf(4, 2, 0, CV_32F);
    // kalman kf(4, 2, 0, CV_32F);

    // cv::Mat state(stateSize, 1, type);
    
    // kalman.init( 4, 2, 0 )
    
    
    // kf.transitionMatrix = cv::Mat::zeros(stateSize, stateSize, type);
    // kf.transitionMatrix.at<float>(0) = 1.0f;
    // kf.transitionMatrix.at<float>(2) = T;
    // kf.transitionMatrix.at<float>(5) = 1.0f;
    // kf.transitionMatrix.at<float>(7) = T;
    // kf.transitionMatrix.at<float>(10) = 1.0f;
    // kf.transitionMatrix.at<float>(15) = 1.0f;

private :
  
  std::vector<int> Container;
};