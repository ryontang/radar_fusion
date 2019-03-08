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

#include <geometry_msgs/Vector3.h>


using namespace cv;

class tracking_measurement
{
  public :
    int id;
    Point3d position;
    Point3d velocity;
    geometry_msgs::Vector3 dimensions;
};


class kf_var
{
public :
    int track_id=-1; // num we can know if the object is match
    
    // int test =21;
    // int a;
    // int add(int a, int b); 
    // float test1;
    
    Mat state = cv::Mat::zeros(4, 1, CV_32F);  // [x,v_x,y,v_y]
    Mat meas = cv::Mat::zeros(4, 1, CV_32F);    // [xr,vr_x,yr,vr_y]
    Mat error_cov_pre = cv::Mat::eye(4, 4, CV_32F)*0.1;   //initial P
    float z ; 
    float vz ; 
  
private :
  
  std::vector<int> Container;
};

// int kf_var::add(int a, int b)  
// {
//   return a + b;
// }


int add(int a, int b , int c)  
{
  return a + b + c;
}

// void cal_the_nearest_point(const lidar_camera_point tmp_lidar_camera_point,
//                                  vector<lidar_camera_point> tmp_radar_point,
//                                  lidar_camera_point& nearest_point,float& min_distance)
// {  
  // vector<float> distance;
  
  // float distance_tmp;
  // float last_distance=0;

  // for(int i=0; i<tmp_radar_point.size(); i++ ){
  //    // cal the distance of two points
  //    distance_tmp=sqrt(pow(tmp_lidar_camera_point.x-tmp_radar_point.at(i).x,2)+
  //                      pow(tmp_lidar_camera_point.y-tmp_radar_point.at(i).y,2)+
  //                      pow(tmp_lidar_camera_point.z-tmp_radar_point.at(i).z,2));
  //    // cout<<distance_tmp<<endl;

  //    // find the min distance
  //    if (distance_tmp<min_distance && distance_tmp<Threshold){
  //      min_distance=distance_tmp;
  //      nearest_point.id=tmp_radar_point.at(i).id;
  //      nearest_point.x=tmp_radar_point.at(i).x;
  //      nearest_point.y=tmp_radar_point.at(i).y;
  //      nearest_point.z=tmp_radar_point.at(i).z;
  //   //  cout<<"------------------------------"<<endl;
  //   //  cout<<"最短距離"<<min_distance<<endl;
  //   //  cout<<nearest_point.id<<endl;
  //   //  cout<<"-----------------------"<<endl;

  //   //  cout<<nearest_point.x<<endl;
  //   //  cout<<nearest_point.y<<endl;
  //   //  cout<<nearest_point.z<<endl;
  //   //  cout<<"-----------------------"<<endl;
  //   //  cout<<tmp_lidar_camera_point.x<<endl;
  //   //  cout<<tmp_lidar_camera_point.y<<endl;
  //  / vector<float>::iterator distance_min=min_element(distance.begin(),distance.end());
  // // distance_MAX= *max_element(distance.begin(),distance.end());
  // // cout<< *distance_min <<endl;

// } //  cout<<tmp_lidar_camera_point.z<<endl;
  //   //  cout<<"------------------------------"<<endl;

  //   }
  //   distance.push_back(distance_tmp);
   
  // }
  
  // /