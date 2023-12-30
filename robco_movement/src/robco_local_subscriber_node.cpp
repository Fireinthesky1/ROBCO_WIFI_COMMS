#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

void robco_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("I Heard: Linear=(%.2f,%.2f,%.2f)\nAngular=(%.2f, %.2f, %.2f)\n",
           msg-> linear.x, msg-> linear.y, msg-> linear.z,
           msg->angular.x, msg->angular.y, msg->angular.z);
}

int main(int argc, char **argv)
{

  // Initialize this ros node
  ros::init(argc, argv, "robco_local_subscriber_node");

  // get a node handle
  ros::NodeHandle node_handle;

  // subscribe to the robco movement node (queue size of 1000)
  ros::Subscriber sub = node_handle.subscribe("jetauto_controller/cmd_vel",
                                              10,
                                              robco_callback);

  // enter a loop pumping callbacks
  ros::spin();

  return 0;
  
}
