// \file
/// \brief Publishes attributes of discovered landmarks straight from gazebo (no noise)
///
/// PARAMETERS:
///
/// PUBLISHES:
/// SUBSCRIBES:
/// FUNCTIONS:
/// SERVICES:
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <math.h>
#include <string>
#include <vector>
#include <functional>  // To use std::bind
#include <algorithm>  // to use std::find_if

#include <boost/iterator/zip_iterator.hpp>

#include "nuslam/TurtleMap.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>

// Global Vars
bool callback_flag = false;
nuslam::TurtleMap map;
std::string robot_name = "diff_drive";
std::string landmark_name = "cylinder";

void gazebo_callback(const gazebo_msgs::ModelStates &model)
{
  // First, find current Diff Drive Robot Pose
  auto dd_it = std::find(model.name.begin(), model.name.end(), robot_name);
  auto dd_index = std::distance(model.name.begin(), dd_it);
  geometry_msgs::Pose dd_pose = model.pose.at(dd_index);
  // Used to populate TurtleMap msg
  std::vector<double> radii;
  std::vector<double> x_pts;
  std::vector<double> y_pts;
  auto it = model.name.begin();
  // Find all instances of "cylinder" as the first 8 characters and push back into list of landmarks for TurtleMap
  while ((it = std::find_if(it, model.name.end(), [](std::string model_name){return !model_name.find(landmark_name); })) < model.name.end())
  {
      // Find pose of landmark relative to Diff Drive Robot and store in map
      // std::cout << "NAME: " << *it << std::endl;
      auto index = std::distance(model.name.begin(), it);
      double x_pos = model.pose.at(index).position.x - dd_pose.position.x;
      double y_pos = model.pose.at(index).position.y - dd_pose.position.y;
      double radius = 0.1;
      // Populate Vectors
      radii.push_back(radius);
      x_pts.push_back(x_pos);
      y_pts.push_back(y_pos);
      it++;
  }
  // Popoulate TurtleMap msg
  map.radii.clear();
  map.radii = radii;
  map.x_pts.clear();
  map.x_pts = x_pts;
  map.y_pts.clear();
  map.y_pts = y_pts;
  callback_flag = true;
}
int main(int argc, char** argv)
/// The Main Function ///
{
  ROS_INFO("STARTING NODE: landmarks");
  double frequency = 60.0;
  std::string frame_id_ = "base_scan";
  ros::init(argc, argv, "analysis"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  ros::NodeHandle nh_("~"); // get a handle to ROS
  // Parameters
  nh_.getParam("frequency", frequency);
  nh_.getParam("landmark_frame_id", frame_id_);
  // Publish TurtleMap data wrt this frame
  map.header.frame_id = frame_id_;
  // Init Publishers
  ros::Publisher landmark_pub = nh_.advertise<nuslam::TurtleMap>("landmarks", 1);
  // Init ModelState Subscriber
  ros::Subscriber gzb_sub = nh.subscribe("/gazebo/model_states", 1, gazebo_callback);
  ros::Rate rate(frequency);
  // Main While
  while (ros::ok())
  {
  	ros::spinOnce();
    if (callback_flag)
    {
      map.header.stamp = ros::Time::now();
      landmark_pub.publish(map);
      callback_flag = false;
    }
    rate.sleep();
  }
  return 0;
}
