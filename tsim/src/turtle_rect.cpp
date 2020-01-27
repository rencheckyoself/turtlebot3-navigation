/// \file
/// \brief This file contains the source code for the turtle_rect node. It pulls in parameters set from the yaml file and uses feed forward control to drive the turtle in a rectangle. It also calculates the positional error over time.
///
/// PARAMETERS:
///     g_x (int) The x coordinate of the lower left corner of a rectangle
///     g_y (int) The y coordinate of the lower left corner of a rectangle
///     width (int) The width of the rectangle
///     height (int) The height of the rectangle
///     trans_vel (int) The translational velocity of the robot
///     rot_vel: (int) The rotational velocity of the robot
///     frequency (int) The frequency of the control loop
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


/// \brief A struct to hold related positional values
///
/// \param x - xposition
/// \param y - yposition
/// \param ang - heading angle in radians
///
struct PoseVals
{
  double x;
  double y;
  double ang;
};


// State Machine Variables
int g_turtle_state = -1;
int g_cycle_cnt = 0;

// global variables to hold the x and y parameters
int g_x, g_y;

// Variables used to calculate the positional error
PoseVals current_pose;
PoseVals expected_pose;
PoseVals error_pose;
tsim::PoseError error;

/// \brief A helper function to teleport the turtle to the beginning of the trajectory
///
void teleport_turtle()
{

  // Create the service clients
  ros::NodeHandle temp_n;

  ros::ServiceClient temp_client_telabs = temp_n.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
  ros::ServiceClient temp_client_pen = temp_n.serviceClient<turtlesim::SetPen>("turtle1/set_pen");

  //wait for turtlesim to advertise the service
  ros::service::waitForService("turtle1/teleport_absolute");
  ros::service::waitForService("turtle1/set_pen");

  // Create service client variables
  turtlesim::SetPen pen;
  turtlesim::TeleportAbsolute tel;

  // Turn the pen off
  pen.request.off = 1;
  temp_client_pen.call(pen);

  // teleport to lower left corner
  tel.request.x = g_x;
  tel.request.y = g_y;
  tel.request.theta = 0;
  temp_client_telabs.call(tel);

  // Turn the pen on
  pen.request.off = 0;
  temp_client_pen.call(pen);

  //reset the state machine
  g_turtle_state = -1;
  g_cycle_cnt = 0;

  //update the expected position of the turtle
  expected_pose.x = g_x;
  expected_pose.y = g_y;
  expected_pose.ang = 0;

}

/// \brief Helper function to calculate the current positional error
///
void calc_error()
{
  // Calculate the error
  error_pose.x = std::fabs(current_pose.x - expected_pose.x);
  error_pose.y = std::fabs(current_pose.y - expected_pose.y);
  error_pose.ang = std::fabs(current_pose.ang - expected_pose.ang);

  // update the PoseError variable to publish
  error.x_error = error_pose.x;
  error.y_error = error_pose.y;
  error.theta_error = error_pose.ang;
}

bool callback_reset_traj(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  teleport_turtle();

  return 1;
}

/// \brief Callback function for the /turtle1/pose subscriber.
///
void callback_pose(const turtlesim::Pose & msg)
{
  // Update the current pose
  current_pose.x = msg.x;
  current_pose.y = msg.y;
  current_pose.ang = msg.theta;
}

/// \brief Main function to create the turtle_rect node.
///
int main(int argc, char **argv)
{

  //Initializations
  ros::init(argc, argv, "turtle_rect");

  ros::NodeHandle n;

  ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 2);
  ros::Publisher pub_error = n.advertise<tsim::PoseError>("pose_error", 2);

  ros::Subscriber sub_pose = n.subscribe("turtle1/pose", 1, callback_pose);

  ros::ServiceServer srv_reset = n.advertiseService("reset_traj", callback_reset_traj);

  // Retrieve params from Parameter Server
  int width, height, trans_vel, rot_vel, frequency;

  n.getParam("x", g_x);
  n.getParam("y", g_y);
  n.getParam("width", width);
  n.getParam("height", height);
  n.getParam("trans_vel", trans_vel);
  n.getParam("rot_vel", rot_vel);
  n.getParam("frequency", frequency);

  //Print params from the Server.
  ROS_INFO_STREAM("Got x param: " << g_x);
  ROS_INFO_STREAM("Got y param: " << g_y);
  ROS_INFO_STREAM("Got width param: " << width);
  ROS_INFO_STREAM("Got height param: " << height);
  ROS_INFO_STREAM("Got trans_vel param: " << trans_vel);
  ROS_INFO_STREAM("Got rot_vel param: " << rot_vel);
  ROS_INFO_STREAM("Got frequency param: " << frequency);

  // Variables to hold the number of cycles to travel the desired disatnce at the specified velocities
  int hor_cycles, ver_cycles, turn_cycles;
  int  direction; // represents the direction of the turtles motion, will either be 1 or -1
  PoseVals change;
  geometry_msgs::Twist v_command;

  // Thresholds to determine if the turtle is close enough to the starting point after teleporting
  double lin_thresh = 0.05;
  double ang_thresh = 0.05;

  direction = 1;

  v_command.linear.x = 0;
  v_command.angular.z = 0;

  // Calculate the number of cycles to traverse each segment and turn
  hor_cycles = (float) (width/trans_vel) * frequency;
  ver_cycles = (float) (height/trans_vel) * frequency;
  turn_cycles = (float) (rigid2d::PI/rot_vel) * frequency;
  ROS_INFO("Cycles for the horz, vertical, and turn movements respectively:");
  ROS_INFO("%d %d %d", hor_cycles, ver_cycles, turn_cycles);

  //Set the loop rate
  ros::Rate r(frequency);

  while(ros::ok())
  {
    // Use Feedforward control to move around the rectangle

    // State Machine
    switch(g_turtle_state)
    {
      case -1:
      // Reset State

      teleport_turtle();

      ros::Duration(0.5).sleep();

      calc_error();

      //Determine if the turtle is close enough to the starting point to begin the trajectory
      if(error_pose.x < lin_thresh || error_pose.y < lin_thresh || error_pose.ang < ang_thresh)
      {
        g_turtle_state = 0;
        g_cycle_cnt = -1;
      }

      break;
      case 0:
      // Horizontal Move
      v_command.linear.x = trans_vel;
      v_command.angular.z = 0;

      // Calculate the expected positional change
      change.x = (float) direction * trans_vel * 1/frequency;
      change.y = 0;
      change.ang = 0;

      //Proceed to next state segment it completed
      if(g_cycle_cnt >= hor_cycles-1)
      {
        g_cycle_cnt = -1;
        g_turtle_state++;
      }

      break;
      case 1:
      // Turn
      v_command.linear.x = 0;
      v_command.angular.z = rot_vel;

      // Calculate the expected positional change
      change.x = 0;
      change.y = 0;
      change.ang = (float) rot_vel * 1/frequency;

      //Proceed to next state segment it completed
      if(g_cycle_cnt >= turn_cycles-1)
      {
        g_cycle_cnt = -1;
        g_turtle_state++;
      }

      break;
      case 2:
      // Vertical Move
      v_command.linear.x = trans_vel;
      v_command.angular.z = 0;

      // Calculate the expected positional change
      change.x = 0;
      change.y = (float) direction * trans_vel * 1/frequency;
      change.ang = 0;

      //Proceed to next state segment it completed
      if(g_cycle_cnt >= ver_cycles-1)
      {
        g_cycle_cnt = -1;
        g_turtle_state++;
      }

      break;
      case 3:
      // Turn
      v_command.linear.x = 0;
      v_command.angular.z = rot_vel;

      // Calculate the expected positional change
      change.x = 0;
      change.y = 0;
      change.ang = (float) rot_vel * 1/frequency;

      //Proceed to next state segment it completed
      if(g_cycle_cnt >= turn_cycles-1)
      {
        g_cycle_cnt = -1;
        g_turtle_state = 0;
        direction *= -1;
      }

      break;
    }

    // Publish velocity command
    pub_vel.publish(v_command);

    // Adjust for turtlesim angle wrapping
    if(expected_pose.ang >= rigid2d::PI)
    {
      expected_pose.ang = -rigid2d::PI+0.1;
    }

    // Update the expected position
    expected_pose.x += change.x;
    expected_pose.y += change.y;
    expected_pose.ang += change.ang;

    calc_error();

    //publish error
    pub_error.publish(error);
    // ROS_INFO("state: %d \t x: %f \t y: %f \t ang: %f", g_turtle_state, error_pose.x, error_pose.y, error_pose.ang);

    g_cycle_cnt++;
    ros::spinOnce();
    r.sleep();

  }

  return 0;
}
