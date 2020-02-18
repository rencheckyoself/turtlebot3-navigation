/// \file
/// \brief This node publishes a twist to drive the turtlebot along a path of waypoints
///
/// PARAMETERS:
///     /frac_val: (double) proportion of max speed to use
///     /avel_lim: (double) max robot angular velocity
///     /tvel_lim: (double) max robot linear velocity
///     /frequency: (int) frequency to publish at
///     waypoint_x: (double) A list of the x coordinates for a series of waypoints
///     waypoint_y: (double) A list of the y coordinates for a series of waypoints
/// PUBLISHES:
///     /cmd_vel: (geometry_msgs/Twist) the twist command
/// SUBSCRIBES:
///     /odom: (nav_msgs/Odometry) the current estimated pose of the robot
/// SERIVCES:
///     /start: (std_srvs/Empty) Starts the movement of the robot
///     /stop: (std_srvs/Empty) Stops the movement of the robot

#include <ros/ros.h>
#include <iostream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/waypoints.hpp"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <std_srvs/Empty.h>

#include "rigid2d/SetPose.h"
#include "rigid2d/Pose.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

static double frac_val = 0;
static int robot_state = 0;
static double avel_lim = 0;
static double tvel_lim = 0;
static double frequency = 0;

static std::vector<double> waypoint_x;
static std::vector<double> waypoint_y;
static std::vector<rigid2d::Vector2D> waypoint_list;
static rigid2d::Pose init_pos;
static rigid2d::Pose2D expected_pose;

ros::Publisher marker_pub;

static int cycles_to_stop = 1;

/// \breif Callback for the start service
bool callback_start(std_srvs::Empty::Request &, std_srvs::Empty::Response&)
{

  ros::NodeHandle temp_n;
  ros::ServiceClient temp_pose = temp_n.serviceClient<rigid2d::SetPose>("set_pose");
  ros::Rate r(5);
  ros::service::waitForService("set_pose");

  rigid2d::SetPose ps;

  // Reset Pose
  ROS_INFO_STREAM("Init Pose: " << init_pos.x << " " << init_pos.y);
  ps.request.pose.x = init_pos.x;
  ps.request.pose.y = init_pos.y;
  ps.request.pose.ang = init_pos.ang;
  temp_pose.call(ps);

  // set robot state
  robot_state = 1;

  // Publish markers if first start
  if(cycles_to_stop == 1)
  {
    // Publish the markers
    for(unsigned int i = 0; i < waypoint_list.size(); i++)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/odom";
      marker.header.stamp = ros::Time::now();

      marker.ns = "waypoints";
      marker.id = i;

      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = waypoint_list.at(i).x;
      marker.pose.position.y = waypoint_list.at(i).y;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 1;

      marker.scale.x = 0.02;
      marker.scale.y = 0.02;
      marker.scale.z = .5;

      marker.color.r = 1.0f;
      marker.color.b = 0.0f;
      marker.color.g = 0.0f;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration(); //infinite lifetime

      marker_pub.publish(marker);
      r.sleep();
    }
  }

  return 1;
}

/// \breif Callback for the odom subscriber
void callback_pose(nav_msgs::Odometry::ConstPtr odom)
{
  expected_pose.x = odom->pose.pose.position.x;
  expected_pose.y = odom->pose.pose.position.y;

  // Extract the robot heading
  auto r = 0.0, p = 0.0, y = 0.0;
  tf2::Quaternion quat_tf2(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
  tf2::Matrix3x3 heading(quat_tf2);
  heading.getRPY(r,p,y);

  expected_pose.th = y;
}

/// \breif Callback for the start service
bool callback_stop(std_srvs::Empty::Request &, std_srvs::Empty::Response&)
{
  robot_state = -1;
  return 1;
}

/// \breif main function to create the real_waypoints node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "real_waypoints");
  ros::NodeHandle n;

  double kp = 0;

  // Get Parameters
  n.getParam("frac_val", frac_val);
  n.getParam("avel_lim", avel_lim);
  n.getParam("tvel_lim", tvel_lim);
  n.getParam("frequency", frequency);
  n.getParam("kp", kp);
  n.getParam("waypoint_x", waypoint_x);
  n.getParam("waypoint_y", waypoint_y);

  ROS_INFO_STREAM("REAL_WAY: Got Frac Val: " << frac_val);
  ROS_INFO_STREAM("REAL_WAY: Got Lin. Vel Limit: " << tvel_lim);
  ROS_INFO_STREAM("REAL_WAY: Got Ang. Vel Limit: " << avel_lim);
  ROS_INFO_STREAM("REAL_WAY: Frequency: " << frequency);
  ROS_INFO_STREAM("REAL_WAY: Prop. Gain: " << kp);

  // Assemble Waypoint Vector
  rigid2d::Vector2D buf;
  for(unsigned int i = 0; i<waypoint_x.size(); i++)
  {
    buf.x = waypoint_x.at(i);
    buf.y = waypoint_y.at(i);

    waypoint_list.push_back(buf);

    ROS_INFO_STREAM("REAL_WAY: Waypoint " << i << " read as " << waypoint_list.at(i));
  }

  // ROS Initializations
  ros::Rate r(frequency);
  ros::Publisher pub_cmd = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber sub_pose = n.subscribe("/odom", 1, callback_pose);
  ros::ServiceServer client_start = n.advertiseService("start", callback_start);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Initialize waypoint following
  rigid2d::Waypoints path(waypoint_list, frequency, tvel_lim * frac_val, avel_lim * frac_val);
  path.setGains(kp);
  path.setThresholds(0.025, 0.01);

  rigid2d::Vector2D init_target = path.getTarget();
  rigid2d::Vector2D target;

  // set the initial pose to the first waypoint
  init_pos.x = init_target.x;
  init_pos.y = init_target.y;
  init_pos.ang = 0;

  ROS_INFO_STREAM("REAL_WAY: Init Pose Beginning: " << init_pos.x << " " << init_pos.y);

  geometry_msgs::Twist send_cmd;

  while(ros::ok())
  {

    switch (robot_state)
    {
      case -1:
      // Stopped. Must call start service to begin

        send_cmd.linear.x = 0;
        send_cmd.angular.z = 0;

        // Set init_pos to the expected_pose so the start service can resume based on the robots current position
        init_pos.x = expected_pose.x;
        init_pos.y = expected_pose.y;
        init_pos.ang = expected_pose.th;
        break;

      case 0:
      // Standby. Must call start service to begin
        send_cmd.linear.x = 0;
        send_cmd.angular.z = 0;

        break;

      case 1:
      // Robot is running.
      // get the velcotiy command to move towards the target
        send_cmd = rigid2d::Twist2DtoGeoTwist(path.nextWaypoint(expected_pose));

        break;
    }

    // pubish the twist command
    pub_cmd.publish(send_cmd);

    // check if the robot has completed one loop
    if(path.getCycles() >= cycles_to_stop)
    {
      robot_state = -1;
      ROS_INFO_STREAM("Robot has completed the path!");
      cycles_to_stop++;
    }

    ros::spinOnce();
    r.sleep();
  }
}
