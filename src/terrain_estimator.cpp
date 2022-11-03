#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Range.h>

#include <algorithm>

tf2_ros::Buffer tfBuffer;
ros::Publisher  pub;

void rangeCallback(const sensor_msgs::Range::ConstPtr& msg){

  static float terrain_alt = 0.0f;
  bool update_terrain_estimate = true;

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  sensor_msgs::Range rangeMsg;

  // Take in the raw rangefinder reading
  float range = msg->range;
  float min_range = std::max(msg->min_range,0.5f);// / 100.0f,0.5f);
  float max_range = msg->max_range;// / 100.0f;
  //max_range = 50.0f;

  // If we're outside the range of the rangefinder, reject the new data and just publish the old values
  if (range < min_range) update_terrain_estimate = false;
  if (range > max_range) update_terrain_estimate = false;

  // Get the tilt angle of the UAV
  if (update_terrain_estimate) 
  {
    try
    {
      geometry_msgs::TransformStamped quadPose = tfBuffer.lookupTransform("map", "uav1", msg->header.stamp, ros::Duration(1.0));
      tf2::Quaternion quat_tf;
      tf2::convert(quadPose.transform.rotation, quat_tf);  // Check correct one
      tf2::Matrix3x3 m(quat_tf);
      double roll, pitch, yaw, tilt;
      m.getRPY(roll, pitch, yaw);
      tilt = atanf(sqrtf(tanf(roll)*tanf(roll)+tanf(pitch)*tanf(pitch)));

      // Only update the terrain estimate if we're not too tilted (causes bad stuff)
      if (tilt < 45.0/57.3)
      {
        
        // Calculate the terrain altitude relative to home
        float tilt_factor = cosf(tilt);
        //ROS_INFO("Range: %6.2f, Alt: %6.2f, Tilt Factor: %6.3f",range,quadPose.transform.translation.z,tilt_factor);
        float terrain_alt_raw = quadPose.transform.translation.z - range*tilt_factor;  
        const float filter_const = 0.75f; 

        terrain_alt = terrain_alt*filter_const + terrain_alt_raw*(1-filter_const); 
      
      }

    } catch (tf2::TransformException &ex) {}
  }

  // Broadcast the terrain tf2
  transformStamped.header.stamp = msg->header.stamp;
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "terrain";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = terrain_alt;
  
  transformStamped.transform.rotation.w = 1.0;
  transformStamped.transform.rotation.x = 0.0;
  transformStamped.transform.rotation.y = 0.0;
  transformStamped.transform.rotation.z = 0.0;

  br.sendTransform(transformStamped);

  // Publish the terrain alt
  rangeMsg.header.stamp = msg->header.stamp;
  rangeMsg.header.frame_id = "terrain";
  rangeMsg.range = transformStamped.transform.translation.z;

  pub.publish(rangeMsg);

  // Debugging
  //printf("Terrain Alt: %6.1f  [ m ]\n", transformStamped.transform.translation.z );

}

int main(int argc, char** argv){
  ros::init(argc, argv, "terrain_broadcaster");
    
  ros::NodeHandle node;

  // Initialise uav buffer
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Subscribe to the rangefinder topic
  ros::Subscriber sub = node.subscribe("mavros/rangefinder/rangefinder", 1, &rangeCallback);
  pub = node.advertise<sensor_msgs::Range>("mavros/rangefinder/terrain_height", 3);

  // Spin forever!
  ros::spin();

  return 0;

};

