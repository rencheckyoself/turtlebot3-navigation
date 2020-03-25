/// \file
/// \brief This node takes the information from gazebo and publishes it for debuggin SLAM node
///
/// PUBLISHES:
///   landmarks: (nuslam/TurtleMap) The groundtruth landmark information
/// SUBSCRIBES:
///   gazebo/model_states (gazebo_msgs/ModelStates) The groundtruth position of all of the landmarks


#include <ros/ros.h>
#include <string.h>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "nuslam/TurtleMap.h"
#include "nav_msgs/Path.h"

#include "rigid2d/rigid2d.hpp"

/// publishers for data
ros::Publisher landmark_pub, gt_path_pub;

/// frame to publish landmarks in
std::string landmark_frame_id = "base_scan";

/// frame to publish landmarks in
std::string path_frame_id = "map";

/// robot name from urdf
std::string robot_name = "diff_drive";

/// groundtruth poses over time
std::vector<geometry_msgs::PoseStamped> gt_points;

/// radius threshold
double radius_threshold = 0;

/// \brief Get the yaw from a ros pose message
///
static double getYawFromPose(geometry_msgs::Pose pose)
{
  auto r = 0.0, p = 0.0, y = 0.0;
  tf2::Quaternion quat_tf2(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3 heading(quat_tf2);
  heading.getRPY(r,p,y);

  return y;
}

/// \brief Convert gazebo data to TurtleMap format
///
void callback_gazebo_data(const gazebo_msgs::ModelStates &data)
{
  std::vector<std::string> model_names = data.name;
  std::vector<double> radius;
  std::vector<geometry_msgs::Point> cyl_centers;
  geometry_msgs::Point center;

  geometry_msgs::PoseStamped gt_point;

  center.z = 0;

  auto dd_iter = std::find(data.name.begin(), data.name.end(), robot_name);
  auto dd_index = std::distance(data.name.begin(), dd_iter);
  auto dd_pose = data.pose.at(dd_index);

  auto yaw = getYawFromPose(dd_pose);

  rigid2d::Transform2D T_mr(rigid2d::Pose2D(yaw, dd_pose.position.x, dd_pose.position.y));

  nuslam::TurtleMap map;
  nav_msgs::Path gt_path;

  gt_point.header.frame_id = path_frame_id;
  gt_point.header.stamp = ros::Time::now();

  gt_point.pose = dd_pose;

  gt_points.push_back(gt_point);

  gt_path.header.frame_id = path_frame_id;
  gt_path.header.stamp = ros::Time::now();

  gt_path.poses = gt_points;

  gt_path_pub.publish(gt_path);

  for(unsigned int i =0; i < model_names.size()-1; i++)
  {
    //if this model is a cylinder, get the pose
    if(!model_names.at(i).compare(0, 8, "cylinder"))
    {

      rigid2d::Transform2D T_ml(rigid2d::Pose2D(0, data.pose.at(i).position.x, data.pose.at(i).position.y));

      // ROS_INFO_STREAM("Gazb x: "<< data.pose.at(i).position.x << " Gazb y: " << data.pose.at(i).position.y);
      // ROS_INFO_STREAM("T_ml x: "<< T_ml.displacement().x << " T_ml y: " << T_ml.displacement().y);

      rigid2d::Transform2D T_rl = T_mr.inv() * T_ml;

      center.x = T_rl.displacement().x;
      center.y = T_rl.displacement().y;

      double dist = std::sqrt(center.x*center.x + center.y*center.y);

      // if it is within the radius threshold add the landmark to the
      if(dist < radius_threshold)
      {
        cyl_centers.push_back(center);
        radius.push_back(0.05);
      }
    }
  }

  map.header.frame_id = landmark_frame_id;
  map.header.stamp = ros::Time::now();
  map.centers = cyl_centers;
  map.radii = radius;

  map.header.stamp = ros::Time::now();
  landmark_pub.publish(map);
}

/// \brief main function
///
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
  gt_path_pub = n.advertise<nav_msgs::Path>("groundtruth_path", 1);
  ros::Subscriber sub_gazebo = n.subscribe("gazebo/model_states", 1, callback_gazebo_data);

  ros::spin();
  return 0;
}
