/// \file
/// \brief A brief description of what the file does
///
/// PARAMETERS:
///     parameter_name (parameter_type): description of the parameter
/// PUBLISHES:
///     topic_name (topic_type): description of topic
/// SUBSCRIBES:
///     topic_name (topic_type): description of the topic
/// SERVICES:
///     service_name (service_type): description of the service

#include "ros/ros.h"
#include "std_srvs/Empty.h"

#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "turtlesim/Pose.h"

#include "geometry_msgs/Twist.h"

#include "tsim/PoseError.h"
// Struct to hold three related x,y,ang values
struct PoseVals
{
  double x;
  double y;
  double ang;
};

int g_turtle_state = -1;
int g_cycle_cnt = 0;
int g_x, g_y;

PoseVals current_pose;
PoseVals expected_pose;
PoseVals error_pose;
tsim::PoseError error;

void teleport_turtle()
{

  ros::NodeHandle temp_n;

  ros::ServiceClient temp_client_telabs = temp_n.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
  ros::ServiceClient temp_client_pen = temp_n.serviceClient<turtlesim::SetPen>("turtle1/set_pen");

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

  g_turtle_state = -1;
  g_cycle_cnt = 0;

  expected_pose.x = g_x;
  expected_pose.y = g_y;
  expected_pose.ang = 0;

}

void calc_error()
{
  error_pose.x = std::fabs(current_pose.x - expected_pose.x);
  error_pose.y = std::fabs(current_pose.y - expected_pose.y);
  error_pose.ang = std::fabs(current_pose.ang - expected_pose.ang);

  error.x_error = error_pose.x;
  error.y_error = error_pose.y;
  error.theta_error = error_pose.ang;
}

bool callback_reset_traj(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  teleport_turtle();

  return 1;
}

void callback_pose(const turtlesim::Pose & msg)
{
  current_pose.x = msg.x;
  current_pose.y = msg.y;
  current_pose.ang = msg.theta;
}


int main(int argc, char **argv)
{

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
  ROS_INFO("Got x param: %d", g_x);
  ROS_INFO("Got y param: %d", g_y);
  ROS_INFO("Got width param: %d", width);
  ROS_INFO("Got height param: %d", height);
  ROS_INFO("Got trans_vel param: %d", trans_vel);
  ROS_INFO("Got rot_vel param: %d", rot_vel);
  ROS_INFO("Got frequency param: %d", frequency);

  // Variables to hold the number of cycles to travel the desired disatnce at the specified velocities
  int hor_cycles, ver_cycles, turn_cycles, direction;
  PoseVals change;

  double lin_thresh = 0.05;
  double ang_thresh = 0.05;

  direction = 1;

  hor_cycles = (float) (width/trans_vel) * frequency;
  ver_cycles = (float) (height/trans_vel) * frequency;
  turn_cycles = (float) (1.57/rot_vel) * frequency;
  ROS_INFO("Cycles for the horz, vertical, and turn movements respectively:");
  ROS_INFO("%d %d %d", hor_cycles, ver_cycles, turn_cycles);

  ros::Rate r(frequency);

  geometry_msgs::Twist v_command;

  v_command.linear.x = 0;
  v_command.angular.z = 0;

  while(ros::ok())
  {
    // Use Feedforward control to move around the rectangle

    switch(g_turtle_state)
    {
      case -1:
      // Reset State
      // Wait for turtlesim services and teleport turtle to the starting position
      teleport_turtle();

      ros::Duration(1.0).sleep();

      calc_error();

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

      change.x = (float) direction * trans_vel * 1/frequency;
      change.y = 0;
      change.ang = 0;

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

      change.x = 0;
      change.y = 0;
      change.ang = (float) rot_vel * 1/frequency;

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

      change.x = 0;
      change.y = (float) direction * trans_vel * 1/frequency;
      change.ang = 0;

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

      change.x = 0;
      change.y = 0;
      change.ang = (float) rot_vel * 1/frequency;

      if(g_cycle_cnt >= turn_cycles-1)
      {
        g_cycle_cnt = -1;
        g_turtle_state = 0;
        direction *= -1;
      }

      break;
    }

    pub_vel.publish(v_command);

    if(expected_pose.ang >= 3.14)
    {
      expected_pose.ang = -3.13;
    }

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
