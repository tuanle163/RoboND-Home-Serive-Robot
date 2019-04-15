#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include "nav_msgs/Odometry.h"


float robot_current_x, robot_current_y; 
float pickup_distance, dropoff_distance;
float dropoff_x = -2.0; 
float dropoff_y = -1.0;
float pickup_x = 3.0;
float pickup_y = 1.0;
float threshold = 0.7;
bool reach_pickup = false;
bool reach_dropoff = false;

void position_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  robot_current_x = msg->pose.pose.position.x;
  robot_current_y = msg->pose.pose.position.y;
  
  if (!reach_pickup && !reach_dropoff)
  {
  	pickup_distance = pow(pow((robot_current_x - pickup_x), 2) + pow((robot_current_y - pickup_y), 2), 0.5);
    ROS_INFO("Distance to pickup: ([%.3f])", pickup_distance);
    if (pickup_distance < threshold) 
    {
      reach_pickup = true;
      ROS_WARN_ONCE("The Robot start to pick up object.");
    }
  }
  if (reach_pickup && !reach_dropoff)
  {
   dropoff_distance = pow(pow((robot_current_x - dropoff_x), 2) + pow((robot_current_y - dropoff_y), 2), 0.5);
    ROS_INFO("Distance to drop off: ([%.3f])", dropoff_distance);
   if (dropoff_distance < threshold)
   {
     reach_dropoff = true;
     reach_pickup = false;
     ROS_WARN_ONCE("The Robot reaches drop off zone and drop the package.");
   }
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("/odom", 1000, position_callback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0;

	// Set position and orientation of the package
    marker.pose.position.x = pickup_x;
    marker.pose.position.y = pickup_y;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

  while (ros::ok())
  {
    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    if (reach_pickup)
    {
      ros::Duration(6.0).sleep();
      marker.action = visualization_msgs::Marker::DELETE;
      
      ROS_WARN_ONCE("The robot picks up the package.");
    }
    if (reach_dropoff)
    {
     marker.pose.position.x = dropoff_x;
     marker.pose.position.y = dropoff_y;
     marker.action = visualization_msgs::Marker::ADD;
     ros::Duration(1.0).sleep();
     ROS_WARN_ONCE("The robot drops the package.");
    }
    marker_pub.publish(marker);
    
    /*
    // Display marker at pickup location
    ROS_INFO("Place the marker at pick up place.");
    marker.pose.position.x = 1.00;
    marker.pose.position.y = 0.00;
    marker.action = visualization_msgs::Marker::ADD;
    
    // Publish pickup marker
    marker_pub.publish(marker);

    // Pause 5 seconds
    ros::Duration(5.0).sleep();
    // Hide the marker
    ROS_INFO("Hide the marker");
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);

    // Pause 5 seconds
    ros::Duration(5.0).sleep();
    // Publish the marker at the drop off
    ROS_INFO("Place the marker at the drop off.");
    marker.pose.position.x = 3.00;
    marker.pose.position.y = 0.00;
    marker.action = visualization_msgs::Marker::ADD;
    // Display marker at drop-off location
    marker_pub.publish(marker);
    */

    ros::spinOnce();
    //r.sleep();
  }
}