#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

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
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

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

    ros::Duration(5.0).sleep();
    r.sleep();
  }
}