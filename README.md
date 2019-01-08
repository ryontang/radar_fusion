### Package 


### Subscribed Topics


| Topic Name | Message Type | Remarks |
| -------- | -------- | -------- |
| /obj_person/obj_label     | [autoware_msgs/obj_label ](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/ObjLabel.msg)    | ---     |
| /as_tx/radar_tracks     | [radar_msgs/RadarTrackArray](http://docs.ros.org/indigo/api/radar_msgs/html/msg/RadarTrackArray.html) | ---     |


### Published Topics
| Topic Name | Message Type | Remarks |
| -------- | -------- | -------- |
| /obj_label_radarfusion   | [autoware_msgs/obj_label ](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/ObjLabel.msg)    | ---     |
| /obj_label_marker_radarfusion    | [radar_msgs/RadarTrackArray](http://docs.ros.org/indigo/api/radar_msgs/html/msg/RadarTrackArray.html) | ---     |

### Process

This package provide preliminary result for the feasibility of whole system.

The position of pedestrians can be calculated from the lidar and camera data, and radar provide the tracking objects without label. This ”radar_fusion” node translates the position to the same coordinate system, and figures out the nearest point to process.

![](https://i.imgur.com/D8pCDIr.png)

### Result
![](https://i.imgur.com/LKWWdgV.png)

![](https://i.imgur.com/BM0dyH6.jpg)
