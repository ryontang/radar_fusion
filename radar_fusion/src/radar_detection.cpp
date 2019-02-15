#include "ros/ros.h"
// #include "std_msgs/String.h"
#include <sstream>
#include <math.h>
#include <algorithm>

//for tf
#include <tf/tf.h>
#include <tf/transform_listener.h>

//msgs from autoware_msgs
#include "autoware_msgs/obj_label.h"
#include "radar_msgs/RadarTrackArray.h"
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//msgs to publish


using namespace std;
static double Threshold = 1.0;//for fusion


static ros::Publisher pub;
static ros::Publisher marker_pub;

//Data type for lidar camera fusion point
typedef struct _lidar_camera_point{
  int id;
  float x;
  float y;
  float z;
}lidar_camera_point;

static vector<lidar_camera_point> lc_point; 
static vector<lidar_camera_point> radar_point; 

// 3 sensors point
static vector<lidar_camera_point> three_sensors_point; 

// For tf
static tf::StampedTransform transformRadar2Map;

static std::string object_type="person";
static ros::Time image_obj_tracked_time;

// void cal_the_centerpoint(const autoware_msgs::obj_label::ConstPtr& msg);

lidar_camera_point tmp_lidarcamera;
lidar_camera_point tmp_radar;
lidar_camera_point nearest_point;
lidar_camera_point tmp_fusion_result_point;

void cal_the_nearest_point(const lidar_camera_point tmp_lidar_camera_point,
                                 vector<lidar_camera_point> tmp_radar_point,
                                 lidar_camera_point& nearest_point,float& min_distance)
{  
  vector<float> distance;
  
  float distance_tmp;
  float last_distance=0;

  for(int i=0; i<tmp_radar_point.size(); i++ ){
     // cal the distance of two points
     distance_tmp=sqrt(pow(tmp_lidar_camera_point.x-tmp_radar_point.at(i).x,2)+
                       pow(tmp_lidar_camera_point.y-tmp_radar_point.at(i).y,2)+
                       pow(tmp_lidar_camera_point.z-tmp_radar_point.at(i).z,2));
     // cout<<distance_tmp<<endl;

     // find the min distance
     if (distance_tmp<min_distance && distance_tmp<Threshold){
       min_distance=distance_tmp;
       nearest_point.id=tmp_radar_point.at(i).id;
       nearest_point.x=tmp_radar_point.at(i).x;
       nearest_point.y=tmp_radar_point.at(i).y;
       nearest_point.z=tmp_radar_point.at(i).z;
    //  cout<<"------------------------------"<<endl;
    //  cout<<"最短距離"<<min_distance<<endl;
    //  cout<<nearest_point.id<<endl;
    //  cout<<"-----------------------"<<endl;

    //  cout<<nearest_point.x<<endl;
    //  cout<<nearest_point.y<<endl;
    //  cout<<nearest_point.z<<endl;
    //  cout<<"-----------------------"<<endl;
    //  cout<<tmp_lidar_camera_point.x<<endl;
    //  cout<<tmp_lidar_camera_point.y<<endl;
    //  cout<<tmp_lidar_camera_point.z<<endl;
    //  cout<<"------------------------------"<<endl;

    }
    distance.push_back(distance_tmp);
   
  }
  
  // vector<float>::iterator distance_min=min_element(distance.begin(),distance.end());
  // distance_MAX= *max_element(distance.begin(),distance.end());
  // cout<< *distance_min <<endl;

}


static visualization_msgs::MarkerArray convert_marker_array(const autoware_msgs::obj_label& src)
{
  visualization_msgs::MarkerArray ret;
  int index = 0;

  std_msgs::ColorRGBA color_white;
  color_white.r = 1.0f;
  color_white.g = 1.0f;
  color_white.b = 1.0f;
  color_white.a = 0.7f;

  std_msgs::ColorRGBA color_red;
  color_red.r = 1.0f;
  color_red.g = 0.0f;
  color_red.b = 0.0f;
  color_red.a = 0.7f;

  std_msgs::ColorRGBA color_blue;
  color_blue.r = 0.0f;
  color_blue.g = 0.0f;
  color_blue.b = 1.0f;
  color_blue.a = 0.7f;

  std_msgs::ColorRGBA color_green;
  color_green.r = 0.0f;
  color_green.g = 1.0f;
  color_green.b = 0.0f;
  color_green.a = 0.7f;

  for (const auto& reproj_pos : src.reprojected_pos)
    {
      visualization_msgs::Marker marker;
      /* Set frame ID */
      marker.header.frame_id = "map";

      /* Set namespace adn id for this marker */
      marker.ns = object_type;
      marker.id = index;
      index++;

      /* set color */
      if (object_type == "car") {
        /* Set marker shape */
        marker.type = visualization_msgs::Marker::SPHERE;

        /* set pose of marker  */
        marker.pose.position = reproj_pos;

        /* set scale of marker */
        marker.scale.x = (double)1.5;
        marker.scale.y = (double)1.5;
        marker.scale.z = (double)1.5;

        marker.color = color_blue;
      }
      else if (object_type == "person") {
        /* Set marker shape */
        marker.type = visualization_msgs::Marker::CUBE;

        /* set pose of marker  */
        marker.pose.position = reproj_pos;

        /* set scale of marker */
        marker.scale.x = (double)0.7;
        marker.scale.y = (double)0.7;
        marker.scale.z = (double)1.8;

        marker.color = color_white ;
        //  cout<<"check marker"<<endl;

      }
      else {
        marker.color = color_green;
      }

      marker.lifetime = ros::Duration(0.3);

      ret.markers.push_back(marker);
    }

  return ret;
}

void linear_weighting_fusing(const lidar_camera_point point1,
                                   lidar_camera_point point2,
                                   lidar_camera_point& point_result)
{
  // cout<<"check" <<endl;
  float w=0.5; //weighting
  point_result.x=(point1.x)*w+(point2.x)*(1-w);
  point_result.y=(point1.y)*w+(point2.y)*(1-w);
  point_result.z=(point1.z)*w+(point2.z)*(1-w);
  
}


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



//callback
//////////////////////////////////////////////////////////////////
void obj_label_callback(const autoware_msgs::obj_label::ConstPtr& msg)
{
    // cout << "check 1" << endl;
    lc_point.clear();//reset the struct
    int obj_num=msg->obj_id.size(); // number of people
    // cout << msg/->obj_id.size()<< endl; 
    image_obj_tracked_time = msg->header.stamp;
      //  cout << image_obj_tracked_time<< endl; 

    
    //load the data from message to struct(lc_point)
    for(int i=0; i<obj_num ; i++){
       tmp_lidarcamera.id=msg->obj_id.at(i);  
       tmp_lidarcamera.x=msg->reprojected_pos.at(i).x;
       tmp_lidarcamera.y=msg->reprojected_pos.at(i).y;
       tmp_lidarcamera.z=msg->reprojected_pos.at(i).z;
       lc_point.push_back(tmp_lidarcamera);
       
    }
    // cout<<"------------------"<<endl;
    // cout << tmp.x << endl;
    // cout << tmp.z << endl;
    // cout << tmp.y << endl;

    // //check the data(lc_point)
    //  for(int j=0; j<obj_num ; j++){
    //     cout << lc_point[j].id <<endl;
    //     cout << lc_point[j].x <<endl;
    //     cout << lc_point[j].y <<endl;
    //     cout << lc_point[j].z <<endl;
    //  }  
    // cout << *msg << endl; //print all
    // cout << msg->obj_id << endl;
}

// for radar 
void radar_tracks_callback(const radar_msgs::RadarTrackArray::ConstPtr& msg)
{
    // cout << "check 2" << endl;
    radar_point.clear();
    int radar_obj_num=msg->tracks.size(); // number of radar tracking objects
    // cout<< *msg << endl;

    geometry_msgs::Polygon tmp_radar_rect;   
    geometry_msgs::Point32 tmp_point;
    tmp_point.x=tmp_point.y=tmp_point.z=0;


    for(int i=0; i<radar_obj_num ; i++){
   
      tmp_radar_rect= msg->tracks.at(i).track_shape;
      cal_the_centerpoint(tmp_radar_rect,tmp_point);//now the point center point of the radar track is "tmp_point"
      
      /* convert from "delphi_esr" coordinate system to "map" coordinate system */
      tf::Vector3 pos_in_esr_coord(tmp_point.x, tmp_point.y, tmp_point.z);
        
      static tf::TransformListener listener;
      try {
          listener.lookupTransform("map", "delphi_esr", ros::Time(0), transformRadar2Map);
      }
      catch (tf::TransformException ex) {
          ROS_INFO("%s", ex.what());
          return;
      }
      tf::Vector3 converted = transformRadar2Map * pos_in_esr_coord; //change th point to "Map" coordinate

      tmp_point.x = converted.x();
      tmp_point.y = converted.y();
      tmp_point.z = converted.z();
      cout<<"------------"<<endl;
      
      tmp_radar.id = msg->tracks.at(i).track_id;
      tmp_radar.x=tmp_point.x;
      tmp_radar.y=tmp_point.y;
      tmp_radar.z=tmp_point.z;

      //check tmp_radar
      // cout<<tmp_radar.id<<endl;
      // cout<<tmp_radar.x<<endl;
      // cout<<tmp_radar.y<<endl;
      // cout<<tmp_radar.z<<endl;
      radar_point.push_back(tmp_radar);

    }    
    
}

//////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
  
  ros::init(argc, argv, "radar_detection");
  ros::NodeHandle n;

  autoware_msgs::obj_label obj_label_msg;
  visualization_msgs::MarkerArray obj_label_marker_msgs;

  
  cout<<"start Subscriber"<< endl;

  // ros::Publisher  chatter_pub =  n.advertise<delay_test::report>("/delay_report", 1000);
  
  // This data has been converted from "camera" coordinate system to "map" coordinate system.
  ros::Subscriber obj_label_sub    =   n.subscribe("/obj_person/obj_label", 1, obj_label_callback);
  ros::Subscriber radar_tracks_sub =   n.subscribe("/as_tx/radar_tracks", 1, radar_tracks_callback);

  pub = n.advertise<autoware_msgs::obj_label>("obj_label_radarfusion",1);
  marker_pub = n.advertise<visualization_msgs::MarkerArray>("obj_label_marker_radarfusion", 1);


  cout<<"Subscriber OK"<< endl;

  ros::Rate loop_rate(15); //HZ
  
  while (ros::ok())
  {
    // cal the nearest point of each lc_point
    // lc_point  radar_point
    // lidar_camera_point nearest_point;
    geometry_msgs::Point tmpPoint_for_fusion;
    obj_label_msg={};
    
    for (int i=0; i<lc_point.size();i++){
       nearest_point={};
       tmp_fusion_result_point={};
       float min_distance=10.0;
       // cout<<"------------------------------------------------"<<endl;
       cal_the_nearest_point(lc_point.at(i),radar_point,nearest_point,min_distance);
       if(min_distance<Threshold){
          linear_weighting_fusing(lc_point.at(i),nearest_point,tmp_fusion_result_point);

          // cout<<"---結果---"<<endl;
          // cout<<tmp_fusion_result_point.x<<endl;
          // cout<<tmp_fusion_result_point.y<<endl;
          // cout<<tmp_fusion_result_point.z<<endl;
          tmpPoint_for_fusion.x=tmp_fusion_result_point.x;
          tmpPoint_for_fusion.y=tmp_fusion_result_point.y;
          tmpPoint_for_fusion.z=tmp_fusion_result_point.z;

          
          obj_label_msg.reprojected_pos.push_back(tmpPoint_for_fusion);
          obj_label_msg.obj_id.push_back(lc_point.at(i).id);
          
          // obj_label_marker_msgs = convert_marker_array(obj_label_msg);
          // cout<< obj_label_msg.header.stamp<<endl;
          // cout<< obj_label_msg.type<<endl;
          // cout<< obj_label_msg.reprojected_pos<<endl;
          // cout<< obj_label_msg.obj_id<<endl;
          
          

          // marker_pub.publish(obj_label_marker_msgs);
  
       } 
    }

    obj_label_msg.type="person";
    obj_label_msg.header.stamp = image_obj_tracked_time;
    pub.publish(obj_label_msg);

    obj_label_marker_msgs = convert_marker_array(obj_label_msg);
    marker_pub.publish(obj_label_marker_msgs);
   
    ros::spinOnce();
    loop_rate.sleep();
  
  }


  return 0;
}
