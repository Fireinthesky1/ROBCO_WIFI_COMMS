#include "ros/ros.h"
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include "geometry_msgs/Twist.h"

#define LINEAR_VELOCITY   0.1
#define ANGULAR_VELOCITY  1.0

int get_key(void)
{
  
  fd_set set;
  struct timeval timeout;
  int rv;
  int buff = 0;
  int len = 1;
  int filedesc = 0;
  FD_ZERO(&set);
  FD_SET(filedesc, &set);

  timeout.tv_sec  = 0;
  timeout.tv_usec = 1000;

  rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

  //TODO::UNDERSTAND THIS EVIL VUDOO MAGIC
  static struct termios old = {0};

  if(tcgetattr(filedesc, &old) < 0)
    ROS_ERROR("tcsetattr()");

  old.c_lflag    &= ~ICANON;
  old.c_lflag    &= ~ECHO;
  old.c_cc[VMIN]  = 1;
  old.c_cc[VTIME] = 0;

  if(tcsetattr(filedesc, TCSANOW, &old) < 0)
    ROS_ERROR("tcsetattr ICANON");

  if(rv == -1)
    ROS_ERROR("select");
  else if(rv == 0)
    ROS_INFO("no_key_pressed");
  else
    read(filedesc, &buff, len);

  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  
  if(tcsetattr(filedesc, TCSADRAIN, &old) < 0)
    ROS_ERROR("tcsetattr ~ICANON");
  return (buff);
}

void update_message(geometry_msgs::Twist *msg, int key)
{
  switch(key)
    {

    case 'w':
      // move the robot forward 
      msg->linear.x  = LINEAR_VELOCITY;
      msg->linear.y  = 0.0;
      msg->linear.z  = 0.0;
      msg->angular.x = 0.0;
      msg->angular.z = 0.0;
      msg->angular.y = 0.0;
      ROS_INFO("MOVING FORWARD");
      break;
      
    case 'a':
      // move robot left
      msg->linear.x  = 0.0;
      msg->linear.y  = LINEAR_VELOCITY;
      msg->linear.z  = 0.0;
      msg->angular.x = 0.0;
      msg->angular.z = 0.0;
      msg->angular.y = 0.0;
      ROS_INFO("MOVING LEFT");
      break;

    case 's':
      // move robot backward
      msg->linear.x  = -LINEAR_VELOCITY;
      msg->linear.y  = 0.0;
      msg->linear.z  = 0.0;
      msg->angular.x = 0.0;
      msg->angular.z = 0.0;
      msg->angular.y = 0.0;
      ROS_INFO("MOVING BACKWARD");
      break;

    case 'd':
      // move robot right
      msg->linear.x  = 0.0;
      msg->linear.y  = -LINEAR_VELOCITY;
      msg->linear.z  = 0.0;
      msg->angular.x = 0.0;
      msg->angular.z = 0.0;
      msg->angular.y = 0.0;
      ROS_INFO("MOVING RIGHT");
      break;

    default:
      // stop the robot
      msg->linear.x  = 0.0;
      msg->linear.y  = 0.0;
      msg->linear.z  = 0.0;
      msg->angular.x = 0.0;
      msg->angular.z = 0.0;
      msg->angular.y = 0.0;
      break;
    }      
}

int main(int argc, char **argv)
{

  // Initialize the robco_move_node
  ros::init(argc, argv, "robco_movement_node");

  // Get a handle for the robco_move_node
  ros::NodeHandle node_handle;

  // Create a Publisher object
  // This publisher publishes type geometry_msg::Twist
  // This publisher publishes to topic cmd_vel
  ros::Publisher cmd_vel_pub =
    node_handle.advertise<geometry_msgs::Twist>("jetauto_controller/cmd_vel", 10);

  //============================================================================

  // First create a twist message
  geometry_msgs::Twist robco_movement_command;

  // set loop rate (here we will do 1 per second)
  ros::Rate loop_rate(1);
    
  while(ros::ok())
    {
      // get the keystroke
      int key = get_key();
      
      // update message
      update_message(&robco_movement_command, key);
      
      // publish this twist message to cmd_vel topic
      cmd_vel_pub.publish(robco_movement_command);
      ROS_INFO("Published Twist Msgs:\nLinear:[%.2f,%.2f,%.2f]\nAngular:[%.2f,%.2f,%.2f]\n\n\n",
               robco_movement_command.linear.x,
               robco_movement_command.linear.y,
               robco_movement_command.linear.z,
               robco_movement_command.angular.x,
               robco_movement_command.angular.y,
               robco_movement_command.angular.z);
      
      // process callbacks if any
      ros::spinOnce();

      loop_rate.sleep();
      
    }

  return 0;
}
