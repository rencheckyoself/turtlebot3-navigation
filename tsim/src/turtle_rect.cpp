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

#include "geometry_msgs/Twist.h"

int g_turtle_state = 0;
int g_cycle_cnt = 0;
int g_x, g_y;


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

  g_turtle_state = 0;
  g_cycle_cnt = 0;

}

bool callback_reset_traj(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  teleport_turtle();

  return 1;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "turtle_rect");

  ros::NodeHandle g_n;

  ros::Publisher pub_vel = g_n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 2);

  ros::ServiceServer srv_reset = g_n.advertiseService("reset_traj", callback_reset_traj);

  // Retrieve params from Parameter Server
  int width, height, trans_vel, rot_vel, frequency;

  g_n.getParam("x", g_x);
  g_n.getParam("y", g_y);
  g_n.getParam("width", width);
  g_n.getParam("height", height);
  g_n.getParam("trans_vel", trans_vel);
  g_n.getParam("rot_vel", rot_vel);
  g_n.getParam("frequency", frequency);

  //Print params from the Server.
  ROS_INFO("Got Param: %d", g_x);
  ROS_INFO("Got Param: %d", g_y);
  ROS_INFO("Got Param: %d", width);
  ROS_INFO("Got Param: %d", height);
  ROS_INFO("Got Param: %d", trans_vel);
  ROS_INFO("Got Param: %d", rot_vel);
  ROS_INFO("Got Param: %d", frequency);

  // Wait for turtlesim services and teleport turtle to the starting position
  teleport_turtle();

  // Variables to hold the number of cycles to travel the desired disatnce at the specified velocities
  int hor_cycles, ver_cycles, turn_cycles;

  // Variable to track the turtle state
  g_turtle_state = 0;
  g_cycle_cnt = 0;

  hor_cycles = (float) (width/trans_vel) * frequency;
  ver_cycles = (float) (height/trans_vel) * frequency;
  turn_cycles = (float) (1.57/rot_vel) * frequency;
  ROS_INFO("Cycles for the horz, vertical, and turn movements respectively:");
  ROS_INFO("%d %d %d", hor_cycles, ver_cycles, turn_cycles);

  ros::Rate r(frequency);

  geometry_msgs::Twist v_command;

  while(ros::ok())
  {
    // Use Feedforward control to move around the rectangle

    switch(g_turtle_state)
    {
      case 0:
      // Horizontal Move
      v_command.linear.x = trans_vel;
      v_command.angular.z = 0;

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

      if(g_cycle_cnt >= turn_cycles-1)
      {
        g_cycle_cnt = -1;
        g_turtle_state = 0;
      }

      break;
    }

    pub_vel.publish(v_command);

    g_cycle_cnt++;
    ros::spinOnce();
    r.sleep();

  }

  return 0;
}
