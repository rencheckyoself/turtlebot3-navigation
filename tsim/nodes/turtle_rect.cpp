
#include "ros/ros.h"
#include "std_srvs/Empty.h"

// Read in yaml file and print via ros info


int main(int argc, char **argv)
{

  ros::init(argc, argv, "turtle_rect");
  ros::NodeHandle n;

  // Retrieve params from Parameter Server
  int x, y, width, height, trans_vel, rot_vel, frequency;

  n.getParam("x", x);
  ROS_INFO("Got Param: %d", x);

  n.getParam("y", y);
  ROS_INFO("Got Param: %d", y);

  n.getParam("width", width);
  ROS_INFO("Got Param: %d", width);

  n.getParam("height", height);
  ROS_INFO("Got Param: %d", height);

  n.getParam("trans_vel", trans_vel);
  ROS_INFO("Got Param: %d", trans_vel);

  n.getParam("rot_vel", rot_vel);
  ROS_INFO("Got Param: %d", rot_vel);

  n.getParam("frequency", frequency);
  ROS_INFO("Got Param: %d", frequency);
}

// lift pen
// teleport to lower left corner
// replace pen

// wait for service to appear

// publish velocity commands to turtle node
