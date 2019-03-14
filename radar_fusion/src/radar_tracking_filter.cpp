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
#include <std_msgs/Header.h>

//time Synchronize
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>



#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

using namespace std;
using namespace cv;
using namespace message_filters;

static double Threshold = 0.5;//for fusion


float T=60*0.001; //T : Operating rate of the system


static ros::Publisher pub;
static ros::Publisher pub_jsk_tracked_objects_new_;

// typedef struct _tracking_measurement{
//   int id;
//   Point3d position;
//   Point3d velocity;
//   geometry_msgs::Vector3 dimensions;
// }tracking_measurement;


static vector<tracking_measurement> lc_point; 
static vector<tracking_measurement> radar_point; 


static std::string object_type="person";
static ros::Time image_obj_tracked_time;


tracking_measurement tmp_lidarcamera;
tracking_measurement tmp_radar;
tracking_measurement nearest_tracking_point;


//for show
jsk_recognition_msgs::BoundingBoxArray tracked_boxes_new;//wei


int lc_obj_num;
int radar_obj_num;
// tracking_measurement nearest_point;
tracking_measurement tmp_fusion_result_point;

// tracking_measurement test_strust;

std_msgs::Header header_tmp;

//kalman filter
KalmanFilter kf(4, 4, 0,CV_64F);
//kalman filter static mat
Mat matrix_F = Mat::zeros(4, 4, CV_32F);
Mat matrix_H = Mat::zeros(4, 4, CV_32F);
Mat matrix_Q = Mat::zeros(4, 4, CV_32F);
Mat matrix_R = Mat::zeros(4, 4, CV_32F);

float min_distance;
float weighting = 0.5; 

// 0 and 1
vector<kf_var> kalman_var_tmp[2];
bool switch_kf_tmp_bool=false;

// save the new point(tmp) to kalman_var_tmp
kf_var  kf_var_tmp;

Mat measurement  = Mat::zeros(4, 1, CV_32F);    // [xr,vr_x,yr,vr_y]

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

  // check tmp_point
  // cout<<"----------------------------"<<endl;
  // cout<<tmp_point<<endl;
}


void cal_nearest_point_tracking(const tracking_measurement tmp_lidar_camera_point,
                                 vector<tracking_measurement> tmp_radar_point,
                                 tracking_measurement& nearest_point,float& min_distance)
{  
  vector<float> distance;
  min_distance=10.0;
  float distance_tmp;
  // float last_distance=0;

  for(int i=0; i<tmp_radar_point.size(); i++ ){
     // cal the distance of two points
    //  distance_tmp=sqrt(pow(tmp_lidar_camera_point.position.x-tmp_radar_point.at(i).position.x,2)+
    //                    pow(tmp_lidar_camera_point.position.y-tmp_radar_point.at(i).position.y,2)+
    //                    pow(tmp_lidar_camera_point.position.z-tmp_radar_point.at(i).position.z,2));
     distance_tmp=sqrt(pow(tmp_lidar_camera_point.position.x-tmp_radar_point.at(i).position.x,2)+
                       pow(tmp_lidar_camera_point.position.y-tmp_radar_point.at(i).position.y,2)
                       );
    //  cout<<distance_tmp<<endl;

     // find the min distance
     if (distance_tmp<min_distance && distance_tmp<Threshold){
       min_distance=distance_tmp;
       nearest_point.id=tmp_radar_point.at(i).id;
       nearest_point.position.x=tmp_radar_point.at(i).position.x;
       nearest_point.position.y=tmp_radar_point.at(i).position.y;
       nearest_point.position.z=tmp_radar_point.at(i).position.z; 
       nearest_point.velocity.x=tmp_radar_point.at(i).velocity.x;
       nearest_point.velocity.y=tmp_radar_point.at(i).velocity.y;
       nearest_point.velocity.z=tmp_radar_point.at(i).velocity.z;
    
    }
    // cout<<min_distance<<endl;

    distance.push_back(distance_tmp);
   
  }
  
}//cal_the_nearest_point



//callback
//////////////////////////////////////////////////////////////////

void callback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg_detect, const radar_msgs::RadarTrackArray::ConstPtr& msg)
{
  // Solve all of perception here...
  cout<< "check callback" << endl;
}

// void detected_obj_new_callback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg)
// {
//     // cout << "check 1" << endl;
//     lc_obj_num = msg->objects.size();// number of radar tracking objects
//     // cout<<"number of radar tracking objects: " << lc_obj_num << endl;
//     lc_point.clear();
//     for (int i=0; i<lc_obj_num ; i++){
//       tmp_lidarcamera.id= msg->objects[i].id;
//       tmp_lidarcamera.position.x=msg->objects[i].pose.position.x;
//       tmp_lidarcamera.position.y=msg->objects[i].pose.position.y;
//       tmp_lidarcamera.position.z=msg->objects[i].pose.position.z;
//       tmp_lidarcamera.velocity.x=msg->objects[i].velocity.linear.x;
//       tmp_lidarcamera.velocity.y=msg->objects[i].velocity.linear.y;
//       tmp_lidarcamera.velocity.z=msg->objects[i].velocity.linear.z;
//       tmp_lidarcamera.dimensions=msg->objects[i].dimensions;
//       // cout<<"id: "<< tmp_lidarcamera.id << endl;
//       // cout<<"Position: " << tmp_lidarcamera.position << endl;
//       // cout<<"Velocity: " << tmp_lidarcamera.velocity << endl;   

//       lc_point.push_back(tmp_lidarcamera);
//     }
//     header_tmp=msg->header;
//     // cout<< "----------------------" << endl;
//     // cout<< *msg << endl;
// }

// // for radar 
// void radar_tracks_callback(const radar_msgs::RadarTrackArray::ConstPtr& msg)
// {
//     // cout << "check 2" << endl;
//     radar_point.clear();

//     radar_obj_num=msg->tracks.size(); // number of radar tracking objects
//     // cout<< *msg << endl;

//     geometry_msgs::Polygon tmp_radar_rect;   //four point of the object 
//     geometry_msgs::Point32 tmp_point;        //the center of the object
//     tmp_point.x=tmp_point.y=tmp_point.z=0;   //initial the point

//     for(int i=0; i<radar_obj_num ; i++){
   
//       tmp_radar_rect= msg->tracks.at(i).track_shape;
//       cal_the_centerpoint(tmp_radar_rect,tmp_point);//now the point center point of the radar track is "tmp_point"
      
//       tmp_radar.id = msg->tracks.at(i).track_id;
//       tmp_radar.position.x=tmp_point.x;
//       tmp_radar.position.y=tmp_point.y;
//       tmp_radar.position.z=tmp_point.z;
//       tmp_radar.velocity.x=msg->tracks.at(i).linear_velocity.x;     
//       tmp_radar.velocity.y=msg->tracks.at(i).linear_velocity.y;     
//       tmp_radar.velocity.z=msg->tracks.at(i).linear_velocity.z;    

//       // check tmp_radar
//       // cout<<tmp_radar.id<<endl;
//       // cout<<tmp_radar.position<<endl;
//       // cout<<tmp_radar.velocity<<endl;
//       // cout<<"----------------------"<<endl;

//       radar_point.push_back(tmp_radar);
//     }    
  
// }

//////////////////////////////////////////////////////////////////
//
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
    Mat matrix_F = Mat::zeros(4, 4, CV_32F);
    matrix_F.at<float>(0) = 1.0;
    matrix_F.at<float>(1) = T/10;
    matrix_F.at<float>(5) = 1.0f;
    matrix_F.at<float>(11) = T/10;
    matrix_F.at<float>(10) = 1.0f;
    matrix_F.at<float>(15) = 1.0f;

    // Measure Matrix H
    // [ 1 0 0 0 ]
    // [ 0 1 0 0 ]
    // [ 0 0 1 0 ]
    // [ 0 0 0 1 ]
    Mat matrix_H = Mat::zeros(4, 4, CV_32F);
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

    Mat matrix_Q = Mat::zeros(4, 4, CV_32F);
    matrix_Q.at<float>(0)  = pow(T,3)/3;
    matrix_Q.at<float>(1)  = pow(T,2)/2;;
    matrix_Q.at<float>(4)  = pow(T,2)/2;
    matrix_Q.at<float>(5)  = T;
    matrix_Q.at<float>(10) = pow(T,3)/3;
    matrix_Q.at<float>(11) = pow(T,2)/2;
    matrix_Q.at<float>(14) = pow(T,2)/2;
    matrix_Q.at<float>(15) = T;
    matrix_Q=225*matrix_Q;

    // Measures Noise Covariance Matrix R
    // cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    // R   
    // [ 0.5   0       0       0     ]
    // [ 0     0.12    0       0     ]
    // [ 0     0       0.5     0     ]
    // [ 0     0       0       0.12  ]
    Mat matrix_R = Mat::zeros(4, 4, CV_32F);
    matrix_R.at<float>(0)  = 0.5 ;
    matrix_R.at<float>(5)  = 0.12;
    matrix_R.at<float>(10)  = 0.5 ;
    matrix_R.at<float>(15)  = 0.12;
    

  // This data has been converted to "delphi_esr" coordinate system.
  // ros::Subscriber detected_obj_new_sub    =   n.subscribe("/detected_objects_new", 1, detected_obj_new_callback);
  // ros::Subscriber radar_tracks_sub =  n.subscribe("/as_tx/radar_tracks", 1, radar_tracks_callback);

  message_filters::Subscriber<autoware_msgs::DetectedObjectArray> detected_obj_new_sub(n, "/detected_objects_new", 1);             // topic1 输入
  message_filters::Subscriber<radar_msgs::RadarTrackArray> radar_tracks_sub(n, "/as_tx/radar_tracks", 1);   // topic2 输入



  //pub = n.advertise<autoware_msgs::obj_label>("obj_label_radarfusion",1);
  pub_jsk_tracked_objects_new_ = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_boxes_tracked/radar_fusion",1);//wei

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
    /////////////////////////////////////////////////////////////////////


    // int num1=1; int num2=2;  int num3=3;
    // cout << "check func"  <<add(num1,num2,num3) <<endl;



    // cout << try1.test  <<endl;
   
    // test_strust.velocity.x=121;


    // kalman_var_tmp_sub.clear();
    
    // kalman_var_tmp_part.push_back(try1);
    // kalman_var_tmp.at(0).push_back(try1);
    // cout << kalman_var_tmp_sub.at(0).state <<endl;
    // cout << kalman_var_tmp.at(0) <<endl;
    
    // cout << "---"  <<endl;

    // kalman_var_tmp[switch_kf_tmp_bool].push_back(try1);
    // cout <<  "test:  "  << kalman_var_tmp[switch_kf_tmp_bool].at(0).track_id<<endl;
    // cout <<  "test:  "  << kalman_var_tmp[!switch_kf_tmp_bool].at(0).track_id<<endl;
    // cout <<  "length:"  << kalman_var_tmp[switch_kf_tmp_bool].size()<<endl; 

    // kalman_var_tmp[0].clear();
    //  cout << "test2:" << kalman_var_tmp.at(switch_kf_tmp_bool).size()<<endl;
    //  kalman_var_tmp.at(switch_kf_tmp_bool).clear();


    // cout << matrix_F  <<endl;
    // cout << matrix_H  <<endl;
    // cout << matrix_Q  <<endl;
    kf.transitionMatrix =matrix_F;
    kf.measurementMatrix=matrix_H;  
    kf.processNoiseCov=matrix_Q;
    kf.measurementNoiseCov=matrix_R;
    // check if lc_point is in kalman_var_tmp[switch_kf_tmp_bool]
    /////////////////////////////////////////////////////////////////////////////
    //If the ans is YES 
    //1. check the nearest point
    //2. use kal
    //3. store the point and data in [!kalman_var_tmpswitch_kf_tmp_bool]
    ////////////////////////////////////////////////////////////////////////////
    //If the ans is NO
    //1. store the NEW point and data in [!kalman_var_tmpswitch_kf_tmp_bool]
    //
    ////////////////////////////////////////////////////////////////////////////

    cout << "---------------------------------------------------------------------------" << endl;  
    cout << "size of lc objects "<< lc_obj_num << endl;
    for (int i=0 ; i<lc_obj_num ; i++){

      //for show bounging box on RViz
    	jsk_recognition_msgs::BoundingBox tracked_box_new;
      autoware_msgs::DetectedObjectArray detected_objects_new;

      //check every "lc_point"
      cout<< "Size of kalman_var_tmp[1/0]: " << kalman_var_tmp[switch_kf_tmp_bool].size()<<endl;

      bool flag_exist_inlast = false; // initialize the flag every time

      for (int j=0; j<kalman_var_tmp[switch_kf_tmp_bool].size(); j++){
      //check if "lc_point" is in  the last tmp vector 
        
        cout << "----- check with the last vector -----" << endl;  
        
        if (lc_point.at(i).id==kalman_var_tmp[switch_kf_tmp_bool].at(j).track_id){            
          flag_exist_inlast = true; 
          cout <<"flag_exist_inlast: " << flag_exist_inlast << endl;
          // cout << "//////////////////" << endl;  
          // cout << lc_point.at(i).id << endl;  
          // cout << "j: " <<j <<endl;
          // cout << kalman_var_tmp[switch_kf_tmp_bool].at(j).track_id << endl; 
          // cout << "//////////////////" << endl;  
      
          // calculate the nearest radar point
          // radar_point
          // cout << "length of radar_point: " << radar_point.size() << endl;
          nearest_tracking_point={};
          // tmp_fusion_result_point={};
          cal_nearest_point_tracking(lc_point.at(i),radar_point,nearest_tracking_point,min_distance);
          
          if(min_distance<Threshold ){
          cout << "check dis " << endl;
          cout << min_distance << endl;

              //start kf
              cout << "////////////  start kf  ////////////////" << endl << endl;  
               
              // cout << "lc: "<< (lc_point.at(i).position.x) << endl;
              // cout << "state last: "<<(kalman_var_tmp[switch_kf_tmp_bool].at(j).state.at<float>(0,0)) << endl;
       
              // 1. setup the initial state
              // Previous state of kf
              kf.statePost.at<float>(0,0)=(lc_point.at(i).position.x)*weighting +
                                          (kalman_var_tmp[switch_kf_tmp_bool].at(j).state.at<float>(0,0))*(1-weighting);                
              kf.statePost.at<float>(1,0)=(lc_point.at(i).velocity.x)*weighting +
                                          (kalman_var_tmp[switch_kf_tmp_bool].at(j).state.at<float>(1,0))*(1-weighting);
              kf.statePost.at<float>(2,0)=(lc_point.at(i).position.y)*weighting +
                                          (kalman_var_tmp[switch_kf_tmp_bool].at(j).state.at<float>(2,0))*(1-weighting);
              kf.statePost.at<float>(3,0)=(lc_point.at(i).velocity.y)*weighting +
                                          (kalman_var_tmp[switch_kf_tmp_bool].at(j).state.at<float>(3,0))*(1-weighting);
             
              //I don't know why the input cannot be load, so I have to use the func below 
              kf.statePost=(Mat_<float>(4,1) << kf.statePost.at<float>(0,0),
                                                kf.statePost.at<float>(1,0),
                                                kf.statePost.at<float>(2,0),
                                                kf.statePost.at<float>(3,0)) ;

              kf.errorCovPost=   kalman_var_tmp[switch_kf_tmp_bool].at(j).error_cov_pre; //P

              // cout << "state: "<< kf.statePost.at<float>(0,0) << endl;
              // cout << "state: "<< kf.statePost << endl;
                     

              cout << "state: "<< kf.statePost << endl;

              // cout << "Q: "<< kf.processNoiseCov     << endl;
              // cout << "F: "<< kf.transitionMatrix    << endl;
              // cout << "H: "<< kf.measurementMatrix   << endl;
              // cout << "R: "<< kf.measurementNoiseCov << endl;


              // 2.kalman prediction
              Mat prediction = kf.predict();
              cout << "new state: "<< kf.statePost << endl;
              // cout << "new statePre: "<< kf.statePre << endl;

              // 3.update measurement
              measurement =(Mat_<float>(4,1) << nearest_tracking_point.position.x,
                                                nearest_tracking_point.velocity.x,
                                                nearest_tracking_point.position.y,
                                                nearest_tracking_point.velocity.y) ;
           

              cout << "check measurement: " << measurement << endl  << endl;  

              // 4.update
              kf.correct(measurement);    
              cout << "new state: "<< kf.statePost << endl;

              cout << "////////////  end kf  ////////////////" << endl;  

              kf_var_tmp.track_id=lc_point.at(i).id;
              kf_var_tmp.state=kf.statePost ;              // [x,v_x,y,v_y]
              kf_var_tmp.z=lc_point.at(i).position.z;
              kf_var_tmp.error_cov_pre = kf.errorCovPost;
              // cout << "errorCovPre: "<< kf.errorCovPre << endl;
              // cout << "errorCovPost: "<< kf.errorCovPost << endl;

              kalman_var_tmp[!switch_kf_tmp_bool].push_back(kf_var_tmp);

          }     

        }// end if

        // else{
        //     // set lc_point into kalman_var_tmp
        //     cout << flag_exist_inlast << endl;

        //     kf_var_tmp.track_id = lc_point.at(i).id;
        //     // [x,v_x,y,v_y]
        //     kf_var_tmp.state.at<float>(0,0)=lc_point.at(i).position.x;
        //     kf_var_tmp.state.at<float>(1,0)=lc_point.at(i).velocity.x; 
        //     kf_var_tmp.state.at<float>(2,0)=lc_point.at(i).position.y; 
        //     kf_var_tmp.state.at<float>(3,0)=lc_point.at(i).velocity.y; 

        //     // [xr,vr_x,yr,vr_y]
        //     // kf_var_tmp.meas.at<float>(0,0)=radar_point.at(i).position.x;
        //     // kf_var_tmp.meas.at<float>(1,0)=radar_point.at(i).position.y; 
        //     // kf_var_tmp.meas.at<float>(2,0)=radar_point.at(i).velocity.x; 
        //     // kf_var_tmp.meas.at<float>(3,0)=radar_point.at(i).velocity.y;

        //     // kf_var_tmp.error_cov_pre=

        //     kalman_var_tmp[!switch_kf_tmp_bool].push_back(kf_var_tmp);
        //     // cout << "check id: " << kalman_var_tmp[!switch_kf_tmp_bool].at(0).track_id << endl;
        //     // error_cov_pre
        //     // kf_var_tmp.state.at<float>(3,0)=2;//test lest the mat be [0,0,0,2]
        //     // cout << kf_var_tmp.state <<endl;

        // }// end else
        
        // cout << "----- finish check -----" << endl;  
         if (flag_exist_inlast == true){
           break;
         }

      } //end for (int j=0; j<kalman_var_tmp[switch_kf_tmp_bool].size(); j++)

      if (flag_exist_inlast == false){
           // set lc_point into kalman_var_tmp
            cout <<"flag_exist_inlast: " <<flag_exist_inlast << endl;

            kf_var_tmp.track_id = lc_point.at(i).id;
            // [x,v_x,y,v_y]
            kf_var_tmp.state.at<float>(0,0)=lc_point.at(i).position.x;
            kf_var_tmp.state.at<float>(1,0)=lc_point.at(i).velocity.x; 
            kf_var_tmp.state.at<float>(2,0)=lc_point.at(i).position.y; 
            kf_var_tmp.state.at<float>(3,0)=lc_point.at(i).velocity.y; 
            kf_var_tmp.z=lc_point.at(i).position.z;

            kalman_var_tmp[!switch_kf_tmp_bool].push_back(kf_var_tmp);
            // cout << "check id: " << kalman_var_tmp[!switch_kf_tmp_bool].at(0).track_id << endl;
            // error_cov_pre
            // kf_var_tmp.state.at<float>(3,0)=2;//test lest the mat be [0,0,0,2]
            // cout << kf_var_tmp.state <<endl;
      }
        
        //BBOXES
	    	tracked_box_new.pose.position.x =kf_var_tmp.state.at<float>(0,0);
 	    	tracked_box_new.pose.position.y =kf_var_tmp.state.at<float>(2,0);
	    	tracked_box_new.pose.position.z =kf_var_tmp.z;
        double sec =ros::Time::now().toSec();
        // double nsec =ros::Time::now();

	    	tracked_box_new.header.stamp.sec = sec;
        // tracked_box_new.header.stamp.nsec = nsec;
	    	tracked_box_new.header.frame_id = "delphi_esr";

	    	tracked_box_new.label = kf_var_tmp.track_id;
	    	tracked_box_new.value =kf_var_tmp.track_id;
	    	tracked_box_new.dimensions = lc_point.at(i).dimensions;
        //
	    	tracked_boxes_new.boxes.push_back(tracked_box_new);
        tracked_boxes_new.header=tracked_box_new.header;    
 	    	//END BBOXES

  
    }//end for (int i=0 ; i<lc_obj_num ; i++)

    //plot the objects
   
    pub_jsk_tracked_objects_new_.publish(tracked_boxes_new);
    tracked_boxes_new={};
    
    // cout << switch_kf_tmp_bool  <<endl;
    // change to the other tmp_vector
    kalman_var_tmp[switch_kf_tmp_bool].clear(); //this may cause memery dump
    switch_kf_tmp_bool=!switch_kf_tmp_bool;
        cout << "check 5"  <<endl;


// message_filters::Subscriber<autoware_msgs::DetectedObjectArray> detected_obj_new_sub(n, "/detected_objects_new", 1);             // topic1 输入
//   message_filters::Subscriber<radar_msgs::RadarTrackArray> radar_tracks_sub(n, "/as_tx/radar_tracks", 1);   // topic2 输入
  
  typedef sync_policies::ApproximateTime<autoware_msgs::DetectedObjectArray,  radar_msgs::RadarTrackArray> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), detected_obj_new_sub, radar_tracks_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));


///
  // TimeSynchronizer<autoware_msgs::DetectedObjectArray, radar_msgs::RadarTrackArray> sync(detected_obj_new_sub, radar_tracks_sub, 10);
  //         cout << "check 6"  <<endl;

  // sync.registerCallback(boost::bind(&callback, _1, _2));
  // message_filters::Subscriber<Detected_Obj> detected_obj_new_sub(n, "/detected_objects_new", 1);             // topic1 输入
  // message_filters::Subscriber<Radar> radar_tracks_sub(n, "/as_tx/radar_tracks", 1);   // topic2 输入
    cout << "check 7"  <<endl;
  ros::spinOnce();

    // ros::spin();
    cout << "check 8"  <<endl;

    loop_rate.sleep();
    cout << "check 9"  <<endl;

  }


  return 0;
}
