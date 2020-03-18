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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "nuslam/TurtleMap.h"

#include "rigid2d/rigid2d.hpp"

ros::Publisher landmark_pub;
std::string landmark_frame_id = "base_scan";
std::string robot_name = "diff_drive";

int radius_threshold = 0;

static double getYawFromPose(geometry_msgs::Pose pose)
{
  auto r = 0.0, p = 0.0, y = 0.0;
  tf2::Quaternion quat_tf2(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3 heading(quat_tf2);
  heading.getRPY(r,p,y);

  return y;
}

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

  auto yaw = getYawFromPose(dd_pose);

  rigid2d::Transform2D T_mr(rigid2d::Pose2D(yaw, dd_pose.position.x, dd_pose.position.y));

  // ROS_INFO_STREAM("Gazb x: "<< dd_pose.position.x << " Gazb y: " << dd_pose.position.y);
  // ROS_INFO_STREAM("T_mr x: "<< T_mr.displacement().x << " T_mr y: " << T_mr.displacement().y);

  nuslam::TurtleMap map;

  for(unsigned int i =0; i < model_names.size()-1; i++)
  {
    //if this model is a cylinder, get the pose
    if(!model_names.at(i).compare(0, 8, "cylinder"))
    {

      // center.x = data.pose.at(i).position.x - dd_pose.position.x;
      // center.y = data.pose.at(i).position.y - dd_pose.position.y;
      // double dist_from_bot = sqrt(center.x*center.x + center.y*center.y);

      rigid2d::Transform2D T_ml(rigid2d::Pose2D(0, data.pose.at(i).position.x, data.pose.at(i).position.y));

      // ROS_INFO_STREAM("Gazb x: "<< data.pose.at(i).position.x << " Gazb y: " << data.pose.at(i).position.y);
      // ROS_INFO_STREAM("T_ml x: "<< T_ml.displacement().x << " T_ml y: " << T_ml.displacement().y);

      rigid2d::Transform2D T_rl = T_mr.inv() * T_ml;

      center.x = T_rl.displacement().x;
      center.y = T_rl.displacement().y;

      cyl_centers.push_back(center);
      radius.push_back(0.01);
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
  np.getParam("radius_threshold", radius_threshold);

  ROS_INFO_STREAM("ANALYSIS: Got landmark frame: " << landmark_frame_id);
  ROS_INFO_STREAM("ANALYSIS: Got robot name: " << robot_name);
  ROS_INFO_STREAM("ANALYSIS: Got radius threshold: " << radius_threshold);

  ros::service::waitForService("/gazebo/get_model_state");

  landmark_pub = n.advertise<nuslam::TurtleMap>("landmark_data", 1);
  ros::Subscriber sub_gazebo = n.subscribe("gazebo/model_states", 1, callback_gazebo_data);

  ros::spin();
  return 0;
}