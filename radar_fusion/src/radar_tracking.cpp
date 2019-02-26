#include "ros/ros.h"
// #include "std_msgs/String.h"
#include <sstream>
#include <math.h>
#include <algorithm>

#include "/home/mec/catkin_ws/src/radar_fusion/radar_fusion/include/radar_fusion/kf_var.hpp"
// #include "kf_var.hpp"

// For tf
// #include <tf/tf.h>
// #include <tf/transform_listener.h>

//msgs from autoware_msgs
// #include "autoware_msgs/obj_label.h"
#include "radar_msgs/RadarTrackArray.h"
#include <geometry_msgs/Polygon.h>


// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

using namespace std;
using namespace cv;

static double Threshold = 1.0;//for fusion


float T=60*0.001; //T : Operating rate of the system


static ros::Publisher pub;
// static ros::Publisher marker_pub;

typedef struct _tracking_measurement{
  int id;
  Point3d position;
  Point3d velocity;
}tracking_measurement;

static vector<tracking_measurement> lc_point; 
static vector<tracking_measurement> radar_point; 

// 3 sensors point
static vector<tracking_measurement> three_sensors_point; 

// For tf
// static tf::StampedTransform transformRadar2Map;

static std::string object_type="person";
static ros::Time image_obj_tracked_time;

// void cal_the_centerpoint(const autoware_msgs::obj_label::ConstPtr& msg);

tracking_measurement tmp_lidarcamera;
tracking_measurement tmp_radar;

int lc_obj_num;
int radar_obj_num;
// tracking_measurement nearest_point;
tracking_measurement tmp_fusion_result_point;

tracking_measurement test_strust;

//kalman filter
KalmanFilter kf(4, 4, 0,CV_64F);
//kalman filter static mat
Mat matrix_F = Mat::zeros(4, 4, CV_32F);
Mat matrix_H = Mat::zeros(4, 4, CV_32F);
Mat matrix_Q = Mat::zeros(4, 4, CV_32F);
Mat matrix_R = Mat::zeros(4, 4, CV_32F);


// 0 and 1
vector<kf_var> kalman_var_tmp[2];
bool switch_kf_tmp_bool=false;

// save the new point(tmp) to kalman_var_tmp
kf_var  kf_var_tmp;

//Because the radar provide four points for the rect
//so this func calculas the cebter of the rect
void cal_the_centerpoint(const geometry_msgs::Polygon& tmp_radar_rect,geometry_msgs::Point32& tmp_point)
{
  // geometry_msgs::Point tmpPoint;
  // cout << msg->tracks.at(0).track_shape.points[0]<<endl;
  float tmp_x,tmp_y,tmp_z;
  tmp_x=tmp_y=tmp_z=0;

  for(int i=0; i<4 ;i++){
     // cout<<tmp_radar_rect.points[i].x<<endl;
     tmp_x=tmp_x+tmp_radar_rect.points[i].x;
     tmp_y=tmp_y+tmp_radar_rect.points[i].y;
     tmp_z=tmp_z+tmp_radar_rect.points[i].z;
  }
  
  tmp_point.x=tmp_x/4;
  tmp_point.y=tmp_y/4;
  tmp_point.z=tmp_z/4;

  //check tmp_point
  // cout<<"----------------------------"<<endl;
  // cout<<tmp_point<<endl;
}


// void cal_the_nearest_point(const tracking_measurement tmp_lidar_camera_point,
//                                  vector<tracking_measurement> tmp_radar_point,
//                                  tracking_measurement& nearest_point,float& min_distance)
// {  
//   vector<float> distance;
  
//   float distance_tmp;
//   float last_distance=0;

//   for(int i=0; i<tmp_radar_point.size(); i++ ){
//      // cal the distance of two points
//      distance_tmp=sqrt(pow(tmp_lidar_camera_point.x-tmp_radar_point.at(i).x,2)+
//                        pow(tmp_lidar_camera_point.y-tmp_radar_point.at(i).y,2)+
//                        pow(tmp_lidar_camera_point.z-tmp_radar_point.at(i).z,2));
//      // cout<<distance_tmp<<endl;

//      // find the min distance
//      if (distance_tmp<min_distance && distance_tmp<Threshold){
//        min_distance=distance_tmp;
//        nearest_point.id=tmp_radar_point.at(i).id;
//        nearest_point.x=tmp_radar_point.at(i).x;
//        nearest_point.y=tmp_radar_point.at(i).y;
//        nearest_point.z=tmp_radar_point.at(i).z;
//     //  cout<<"------------------------------"<<endl;
//     //  cout<<"最短距離"<<min_distance<<endl;
//     //  cout<<nearest_point.id<<endl;
//     //  cout<<"-----------------------"<<endl;

//     //  cout<<nearest_point.x<<endl;
//     //  cout<<nearest_point.y<<endl;
//     //  cout<<nearest_point.z<<endl;
//     //  cout<<"-----------------------"<<endl;
//     //  cout<<tmp_lidar_camera_point.x<<endl;
//     //  cout<<tmp_lidar_camera_point.y<<endl;
//     //  cout<<tmp_lidar_camera_point.z<<endl;
//     //  cout<<"------------------------------"<<endl;

//     }
//     distance.push_back(distance_tmp);
   
//   }
  
// }//cal_the_nearest_point




// void linear_weighting_fusing(const tracking_measurement point1,
//                                    tracking_measurement point2,
//                                    tracking_measurement& point_result)
// {
//   // cout<<"check" <<endl;
//   float w=0.5; //weighting
//   point_result.x=(point1.x)*w+(point2.x)*(1-w);
//   point_result.y=(point1.y)*w+(point2.y)*(1-w);
//   point_result.z=(point1.z)*w+(point2.z)*(1-w);
  
// }


// //Because the radar provide four points for the rect
// //so this func calculas the cebter of the rect
// void cal_the_centerpoint(const geometry_msgs::Polygon& tmp_radar_rect,
//                                 geometry_msgs::Point32& tmp_point)
// {
//   // geometry_msgs::Point tmpPoint;
//   // cout << msg->tracks.at(0).track_shape.points[0]<<endl;
//   float tmp_x,tmp_y,tmp_z;
//   tmp_x=tmp_y=tmp_z=0;

//   for(int i=0; i<4 ;i++){
//      // cout<<tmp_radar_rect.points[i].x<<endl;
//      tmp_x=tmp_x+tmp_radar_rect.points[i].x;
//      tmp_y=tmp_y+tmp_radar_rect.points[i].y;
//      tmp_z=tmp_z+tmp_radar_rect.points[i].z;
//   }
  
//   tmp_point.x=tmp_x/4;
//   tmp_point.y=tmp_y/4;
//   tmp_point.z=tmp_z/4;
// }


//callback
//////////////////////////////////////////////////////////////////

void detected_obj_new_callback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg)
{
    // cout << "check 1" << endl;
    lc_obj_num = msg->objects.size();// number of radar tracking objects
    // cout<<"number of radar tracking objects: " << lc_obj_num << endl;
    
    for (int i=0; i<lc_obj_num ; i++){
      tmp_lidarcamera.id= msg->objects[i].id;
      tmp_lidarcamera.position.x=msg->objects[i].pose.position.x;
      tmp_lidarcamera.position.y=msg->objects[i].pose.position.y;
      tmp_lidarcamera.position.z=msg->objects[i].pose.position.z;
      tmp_lidarcamera.velocity.x=msg->objects[i].velocity.linear.x;
      tmp_lidarcamera.velocity.y=msg->objects[i].velocity.linear.y;
      tmp_lidarcamera.velocity.z=msg->objects[i].velocity.linear.z;

      // cout<<"id: "<< tmp_lidarcamera.id << endl;
      // cout<<"Position: " << tmp_lidarcamera.position << endl;
      // cout<<"Velocity: " << tmp_lidarcamera.velocity << endl;   

      lc_point.push_back(tmp_lidarcamera);
    }
    
    // cout<< "----------------------" << endl;
    // cout<< *msg << endl;
}

// for radar 
void radar_tracks_callback(const radar_msgs::RadarTrackArray::ConstPtr& msg)
{
    // cout << "check 2" << endl;
    radar_point.clear();

    radar_obj_num=msg->tracks.size(); // number of radar tracking objects
    // cout<< *msg << endl;

    geometry_msgs::Polygon tmp_radar_rect;   //four point of the object 
    geometry_msgs::Point32 tmp_point;        //the center of the object
    tmp_point.x=tmp_point.y=tmp_point.z=0;   //initial the point

    for(int i=0; i<radar_obj_num ; i++){
   
      tmp_radar_rect= msg->tracks.at(i).track_shape;
      cal_the_centerpoint(tmp_radar_rect,tmp_point);//now the point center point of the radar track is "tmp_point"
      
      tmp_radar.id = msg->tracks.at(i).track_id;
      tmp_radar.position.x=tmp_point.x;
      tmp_radar.position.y=tmp_point.y;
      tmp_radar.position.z=tmp_point.z;
      tmp_radar.velocity.x=msg->tracks.at(i).linear_velocity.x;     
      tmp_radar.velocity.y=msg->tracks.at(i).linear_velocity.y;     
      tmp_radar.velocity.z=msg->tracks.at(i).linear_velocity.z;    

      // check tmp_radar
      // cout<<tmp_radar.id<<endl;
      // cout<<tmp_radar.position<<endl;
      // cout<<tmp_radar.velocity<<endl;
      // cout<<"----------------------"<<endl;

      radar_point.push_back(tmp_radar);
    }    
  
}

//////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
  
  ros::init(argc, argv, "radar_tracking");
  ros::NodeHandle n;

  //set kalman filter

    // Transition State Matrix F
    // Note: set dT at each processing step!
    // [ 1 dT 0  0  ]
    // [ 0 1  0  0  ]
    // [ 0 0  1  dT ]
    // [ 0 0  0  1  ]
    // Mat matrix_F = Mat::zeros(4, 4, CV_32F);
    matrix_F.at<float>(0) = 1.0;
    matrix_F.at<float>(1) = T;
    matrix_F.at<float>(5) = 1.0f;
    matrix_F.at<float>(11) = T;
    matrix_F.at<float>(10) = 1.0f;
    matrix_F.at<float>(15) = 1.0f;

    // Measure Matrix H
    // [ 1 0 0 0 ]
    // [ 0 1 0 0 ]
    // [ 0 0 1 0 ]
    // [ 0 0 0 1 ]
    // Mat matrix_H = Mat::zeros(4, 4, CV_32F);
    matrix_H.at<float>(0) =  1.0;
    matrix_H.at<float>(5) =  1.0;
    matrix_H.at<float>(10) =  1.0;
    matrix_H.at<float>(15) =  1.0;

    // Process Noise Covariance Matrix Q
    // [ T^3/3   T^2/2   0       0     ]
    // [ T^2/2   T       0       0     ]*q
    // [ 0       0       T^3/3   T^2/2 ]
    // [ 0       0       T^2/2   T     ]
    // PSD: q

    // Mat matrix_Q = Mat::zeros(4, 4, CV_32F);
    matrix_Q.at<float>(0)  = pow(T,3)/3;
    matrix_Q.at<float>(1)  = pow(T,2)/2;;
    matrix_Q.at<float>(4)  = pow(T,2)/2;
    matrix_Q.at<float>(5)  = T;
    matrix_Q.at<float>(10) = pow(T,3)/3;
    matrix_Q.at<float>(11) = pow(T,2)/2;
    matrix_Q.at<float>(14) = pow(T,2)/2;
    matrix_Q.at<float>(15) = T;
    
    

  // This data has been converted to "delphi_esr" coordinate system.
  ros::Subscriber detected_obj_new_sub    =   n.subscribe("/detected_objects_new", 1, detected_obj_new_callback);
  ros::Subscriber radar_tracks_sub =  n.subscribe("/as_tx/radar_tracks", 1, radar_tracks_callback);

  //pub = n.advertise<autoware_msgs::obj_label>("obj_label_radarfusion",1);
  //marker_pub = n.advertise<visualization_msgs::MarkerArray>("obj_label_marker_radarfusion", 1);

  cout<<"Subscriber OK"<< endl;
  ros::Rate loop_rate(15); //HZ

  vector<kf_var> kalman_var_tmp_part;
  kf_var try1;
  
  while (ros::ok())
  {
    /////////////////////////////////////////////////////////////////////
    /////////////////////     practice     /////////////////////////////
    // try1.state={2;1;1;1};

    // kalman_var_tmp.at(switch_kf_tmp_bool).push_back(try1);
    // cout <<  kalman_var_tmp.at(switch_kf_tmp_bool).at(0).state<<endl;
    // cout << kalman_var_tmp <<endl;
    // cout << tmp_vec_kfvar_A.at(0).state.at<float>(0,0) <<endl;
    // kalman_var_tmp.at(switch_kf_tmp_bool).at(0).state.at<float>(0,0)=1;
    // cout <<  kf_var::test    <<endl;

    // int num1=1;
    // int num2=2;
    // int num3;
    /////////////////////////////////////////////////////////////////////

    // cout << try1.test  <<endl;
   
    // test_strust.velocity.x=121;

    // num3=try1.add(num1,num2);
    // cout << test_strust.velocity.x  <<endl;

    // kalman_var_tmp_sub.clear();
    
    // kalman_var_tmp_part.push_back(try1);
    // kalman_var_tmp.at(0).push_back(try1);
    // cout << kalman_var_tmp_sub.at(0).state <<endl;
    // cout << kalman_var_tmp.at(0) <<endl;
    
    cout << "---"  <<endl;

    kalman_var_tmp[switch_kf_tmp_bool].push_back(try1);
    // cout <<  "test:  "  << kalman_var_tmp[switch_kf_tmp_bool].at(0).test<<endl;
    cout <<  "length:"  << kalman_var_tmp[switch_kf_tmp_bool].size()<<endl; 

    kalman_var_tmp[0].clear();
    //  cout << "test2:" << kalman_var_tmp.at(switch_kf_tmp_bool).size()<<endl;
    //  kalman_var_tmp.at(switch_kf_tmp_bool).clear();


    // cout << matrix_F  <<endl;
    // cout << matrix_H  <<endl;
    // cout << matrix_Q  <<endl;
    kf.transitionMatrix =matrix_F;
    kf.measurementMatrix=matrix_H;  


    // check if lc_point is in kalman_var_tmp[switch_kf_tmp_bool]
    /////////////////////////////////////////////////////////////////////////////
    //If the ans is YES 
    //1. check the nearest point
    //2. use kal
    //3. store the point and data in [!kalman_var_tmpswitch_kf_tmp_bool]
    //Is the ans is NO
    ////////////////////////////////////////////////////////////////////////////
    //1.store the NEW point and data in [!kalman_var_tmpswitch_kf_tmp_bool]
    //
    ////////////////////////////////////////////////////////////////////////////

    for (int i=0 ; i<lc_obj_num ; i++){
      //check every "lc_point"
      for (int j=0; j<kalman_var_tmp[switch_kf_tmp_bool].size(); j++){
      //check if "lc_point" is in  the last tmp vector 
        if (lc_point.at(i).id==kalman_var_tmp[switch_kf_tmp_bool].at(j).track_id){            
            // calculate the nearest radar point
            // radar_point
            // ss

        }
        else{
            // set lc_point into kalman_var_tmp
            kf_var_tmp.track_id = lc_point.at(i).id;
            // [x,v_x,y,v_y]
            kf_var_tmp.state.at<float>(0,0)=lc_point.at(i).position.x;
            kf_var_tmp.state.at<float>(1,0)=lc_point.at(i).position.y; 
            kf_var_tmp.state.at<float>(2,0)=lc_point.at(i).velocity.x; 
            kf_var_tmp.state.at<float>(3,0)=lc_point.at(i).velocity.y; 

            // [xr,vr_x,yr,vr_y]
            kf_var_tmp.meas.at<float>(0,0)=radar_point.at(i).position.x;
            kf_var_tmp.meas.at<float>(1,0)=radar_point.at(i).position.y; 
            kf_var_tmp.meas.at<float>(2,0)=radar_point.at(i).velocity.x; 
            kf_var_tmp.meas.at<float>(3,0)=radar_point.at(i).velocity.y;

            // kf_var_tmp.error_cov_pre=

            kalman_var_tmp[!switch_kf_tmp_bool].push_back(kf_var_tmp);
           
            // error_cov_pre
            // kf_var_tmp.state.at<float>(3,0)=2;//test lest the mat be [0,0,0,2]
            // cout << kf_var_tmp.state <<endl;
            
        }
      }
      
      //  if (lc_point.at(i)==kalman_var_tmp.at(switch_kf_tmp_bool).)

    }//end for (int i=0 ; i<lc_obj_num ; i++)

  
   
    
    cout << switch_kf_tmp_bool  <<endl;

    // change to the other tmp_vector
    kalman_var_tmp[switch_kf_tmp_bool].clear(); //this may cause memery dump
    switch_kf_tmp_bool=!switch_kf_tmp_bool;

    ros::spinOnce();
    loop_rate.sleep();
  
  }


  return 0;
}
