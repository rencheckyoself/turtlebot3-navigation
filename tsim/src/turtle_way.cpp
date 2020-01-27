/// \file
/// \brief This file contains the source code for the turtle_way node. It reads in a list of waypoints and sends commands to turtlesim to follow that path
///
/// PARAMETERS:
///     waypoint_x (int) A list of the x coordinates for a series of waypoints
///     waypoint_y (int) A list of the y coordinates for a series of waypoints
///     trans_vel (int) The translational velocity of the robot
///     rot_vel: (int) The rotational velocity of the robot
///     frequency (int) The frequency of the control loop
///     wheel_radius (double) The radius of the wheels
///     wheel_base (double) The distance between two wheels
/// PUBLISHES:
///     /pose_error (tsim/PoseError): The positional error of the turtle at each cycle.
///     /turtle1/cmd_vel (geometry_msgs/Twist): The velcotiy command to control the turtle.
/// SUBSCRIBES:
///     /turtle1/pose (turtlesim/Pose): Retrieves the current pose of the turtle
/// SERVICES:
///     /traj_reset (std_srvs/Empty): resets the turtle to the beginning of the trajectory (lower left corner of the rectangle)

#include <ros/ros.h>
#include "std_srvs/Empty.h"

#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "turtlesim/Pose.h"

#include "geometry_msgs/Twist.h"

#include "tsim/PoseError.h"

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"


// State Machine Variables
static int g_turtle_state = -1;
static int g_cycle_cnt = 0;

// global variables to hold the x and y parameters
static std::vector<int> waypoint_x;
static std::vector<int> waypoint_y;
static std::vector<rigid2d::Vector2D> waypoint_list;

// Variables used to calculate the positional error
static rigid2d::Pose2D current_pose;
static rigid2d::Pose2D expected_pose;
static rigid2d::Pose2D error_pose;
static tsim::PoseError error;

static ros::ServiceClient client_telabs;
static ros::ServiceClient client_pen;

static double trans_vel, rot_vel, wheel_base, wheel_radius;
static double frequency;

/// \brief A helper function to teleport the turtle to the beginning of the trajectory
///
void teleport_turtle(rigid2d::Pose2D tele)
{
  // Create the service clients
  ros::NodeHandle temp_n;

  //wait for turtlesim to advertise the service
  ros::service::waitForService("turtle1/teleport_absolute");
  ros::service::waitForService("turtle1/set_pen");

  // Create service client variables
  turtlesim::SetPen pen;
  turtlesim::TeleportAbsolute tel;

  // Turn the pen off
  pen.request.off = 1;
  client_pen.call(pen);

  // teleport to lower left corner
  tel.request.x = tele.x;
  tel.request.y = tele.y;
  tel.request.theta = tele.th;
  client_telabs.call(tel);

  // Turn the pen on
  pen.request.off = 0;
  client_pen.call(pen);

  //reset the state machine
  g_turtle_state = -1;
  g_cycle_cnt = 0;

  //update the expected position of the turtle
  expected_pose.x = tele.x;
  expected_pose.y = tele.y;
  expected_pose.th = tele.th;
}

/// \brief Helper function to calculate the current positional error
///
void calc_error()
{
  // Calculate the error
  error_pose.x = std::fabs(current_pose.x - expected_pose.x);
  error_pose.y = std::fabs(current_pose.y - expected_pose.y);
  error_pose.th = std::fabs(current_pose.th - expected_pose.th);

  // update the PoseError variable to publish
  error.x_error = error_pose.x;
  error.y_error = error_pose.y;
  error.theta_error = error_pose.th;
}

/// \brief Callback function for the /turtle1/pose subscriber.
///
void callback_pose(const turtlesim::Pose & msg)
{
  // Update the current pose
  current_pose.x = msg.x;
  current_pose.y = msg.y;
  current_pose.th = msg.theta;
}

/// \brief Main function to create the turtle_rect node.
///
int main(int argc, char **argv)
{

  //Initializations
  ros::init(argc, argv, "turtle_way");

  ros::NodeHandle n;

  ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 2);
  ros::Publisher pub_error = n.advertise<tsim::PoseError>("pose_error", 2);

  ros::Subscriber sub_pose = n.subscribe("turtle1/pose", 1, callback_pose);

  client_telabs = n.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
  client_pen = n.serviceClient<turtlesim::SetPen>("turtle1/set_pen");


  // Retrieve params from Parameter Server

  n.getParam("trans_vel", trans_vel);
  n.getParam("rot_vel", rot_vel);
  n.getParam("frequency", frequency);
  n.getParam("waypoint_x", waypoint_x);
  n.getParam("waypoint_y", waypoint_y);

  n.getParam("wheel_radius", wheel_radius);
  n.getParam("wheel_base", wheel_base);


  //Print params from the Server.
  ROS_INFO_STREAM("Got trans_vel param: " << trans_vel);
  ROS_INFO_STREAM("Got rot_vel param: " << rot_vel);
  ROS_INFO_STREAM("Got frequency param: " << frequency);
  ROS_INFO_STREAM("Got wheel radius param: " << wheel_radius);
  ROS_INFO_STREAM("Got wheel base param: " << wheel_base);

  rigid2d::Vector2D buf;
  for(unsigned int i = 0; i<waypoint_x.size(); i++)
  {
    buf.x = waypoint_x.at(i);
    buf.y = waypoint_y.at(i);

    waypoint_list.push_back(buf);

    ROS_INFO_STREAM("Waypoint " << i << " read as " << waypoint_list.at(i));
  }

  rigid2d::Twist2D v_command;
  rigid2d::Vector2D target;

  //Set the loop rate
  ros::Rate r(frequency);

  // initialize the Waypoints class to determine twist command
  rigid2d::Waypoints path(waypoint_list, frequency, trans_vel, rot_vel);
  target = path.getTarget();

  // initialize the Diff Drive object to track the turtles progress
  rigid2d::Pose2D init_pos(0, target.x, target.y);
  rigid2d::DiffDrive turt(init_pos, wheel_base, wheel_radius);

  teleport_turtle(init_pos);

  // Begin normal routine of checking estimate pose and determines velocity command to get to current target
  while(ros::ok())
  {

    // call check if we are at the target and calculate the correct twist
    v_command = path.nextWaypoint(expected_pose);

    // Publish the velocity command to move the turtle in turtlesim
    pub_vel.publish(rigid2d::Twist2DtoGeoTwist(v_command));

    // Update the estimated pose using the Diff Drive object

    turt.feedforward(v_command.scaleTwist(1/frequency));
    expected_pose = turt.pose();

    calc_error();

    //publish error
    pub_error.publish(error);
    // ROS_INFO("state: %d \t x: %f \t y: %f \t ang: %f", g_turtle_state, error_pose.x, error_pose.y, error_pose.ang);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
