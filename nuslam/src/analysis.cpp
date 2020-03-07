/// \file
/// \brief This node takes the information from gazebo and publishes it for debuggin SLAM node
///
/// PARAMETERS:
/// PUBLISHES:
///   landmarks: (nuslam/TurtleMap) The groundtruth landmark information
/// SUBSCRIBES:
///   gazebo/model_states (gazebo_msgs/ModelStates) The groundtruth position of all of the landmarks
/// SERIVCES:
///

// TODO: Add a radius limit

#include <ros/ros.h>
#include <string.h>
#include <vector>

#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "nuslam/TurtleMap.h"

#include "rigid2d/rigid2d.hpp"

ros::Publisher landmark_pub;
std::string landmark_frame_id = "base_scan";
std::string robot_name = "diff_drive";

void callback_gazebo_data(const gazebo_msgs::ModelStates &data)
{
  std::vector<std::string> model_names = data.name;
  std::vector<double> radius;
  std::vector<geometry_msgs::Point> cyl_centers;
  geometry_msgs::Point center;
  center.z = 0;

  auto dd_iter = std::find(data.name.begin(), data.name.end(), robot_name);
  auto dd_index = std::distance(data.name.begin(), dd_iter);
  auto dd_pose = data.pose.at(dd_index);

  nuslam::TurtleMap map;

  for(unsigned int i =0; i < model_names.size(); i++)
  {
    //if this model is a cylinder, get the pose
    if(model_names.at(i).compare(0, 8, "cylinder"))
    {
      center.x = data.pose.at(i).position.x - dd_pose.position.x;
      center.y = data.pose.at(i).position.y - dd_pose.position.y;
      cyl_centers.push_back(center);
      radius.push_back(0.1);
    }
  }

  map.header.frame_id = landmark_frame_id;
  map.header.stamp = ros::Time::now();
  map.centers = cyl_centers;
  map.radii = radius;

  map.header.stamp = ros::Time::now();
  landmark_pub.publish(map);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "analysis");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  np.getParam("landmark_frame_id", landmark_frame_id);
  np.getParam("robot_name", robot_name);

  landmark_pub = n.advertise<nuslam::TurtleMap>("landmarks", 1);
  ros::Subscriber sub_gazebo = n.subscribe("gazebo/model_states", 1, callback_gazebo_data);

  ros::spin();
  return 0;
}
