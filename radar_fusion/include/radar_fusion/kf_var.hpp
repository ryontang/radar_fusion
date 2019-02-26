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
    
    // int test =21;
    // int a;
    // int add(int a, int b); 
    // float test1;
    
    Mat state = cv::Mat::zeros(4, 1, CV_32F);  // [x,v_x,y,v_y]
    Mat meas = cv::Mat::zeros(4, 1, CV_32F);    // [xr,vr_x,yr,vr_y]
    Mat error_cov_pre = cv::Mat::eye(4, 4, CV_32F);   

    // kalman.init( 4, 2, 0 )

   

private :
  
  std::vector<int> Container;
};

// int kf_var::add(int a, int b)  
// {
//   return a + b;
// }