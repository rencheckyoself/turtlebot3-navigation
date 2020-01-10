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

// Read in yaml file and print via ros info

void callback_reset_traj()
{

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "turtle_rect");
  ros::NodeHandle n;

  ros::ServiceClient client_telabs = n.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
  ros::ServiceClient client_pen = n.serviceClient<turtlesim::SetPen>("turtle1/set_pen");

  ros::ServiceServer reset = n.advertiseService("reset_traj", callback_reset_traj);

  ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 2);


  // Retrieve params from Parameter Server
  int x, y, width, height, trans_vel, rot_vel, frequency;

  n.getParam("x", x);
  n.getParam("y", y);
  n.getParam("width", width);
  n.getParam("height", height);
  n.getParam("trans_vel", trans_vel);
  n.getParam("rot_vel", rot_vel);
  n.getParam("frequency", frequency);

  //Print params from the Server.
  ROS_INFO("Got Param: %d", x);
  ROS_INFO("Got Param: %d", y);
  ROS_INFO("Got Param: %d", width);
  ROS_INFO("Got Param: %d", height);
  ROS_INFO("Got Param: %d", trans_vel);
  ROS_INFO("Got Param: %d", rot_vel);
  ROS_INFO("Got Param: %d", frequency);

  // Wait for turtlesim services
  ros::service::waitForService("turtle1/teleport_absolute");
  ros::service::waitForService("turtle1/set_pen");

  // Create service client variables
  turtlesim::SetPen pen;
  turtlesim::TeleportAbsolute tel;

  // Turn the pen off
  pen.request.off = 1;
  client_pen.call(pen);

  // teleport to lower left corner
  tel.request.x = x;
  tel.request.y = y;
  tel.request.theta = 0;
  client_telabs.call(tel);

  // Turn the pen on
  pen.request.off = 0;
  client_pen.call(pen);


  // Variables to hold the number of cycles to travel the desired disatnce at the specified velocities
  int hor_cycles, ver_cycles, turn_cycles;

  // Variable to track the turtle state
  int turtle_state, cycle_cnt;

  turtle_state = 0;
  cycle_cnt = 0;

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

    switch(turtle_state)
    {
      case 0:
      // Horizontal Move
      v_command.linear.x = trans_vel;
      v_command.angular.z = 0;

      if(cycle_cnt >= hor_cycles-1)
      {
        cycle_cnt = -1;
        turtle_state++;
      }

      break;
      case 1:
      // Turn
      v_command.linear.x = 0;
      v_command.angular.z = rot_vel;

      if(cycle_cnt >= turn_cycles-1)
      {
        cycle_cnt = -1;
        turtle_state++;
      }

      break;
      case 2:
      // Vertical Move
      v_command.linear.x = trans_vel;
      v_command.angular.z = 0;

      if(cycle_cnt >= ver_cycles-1)
      {
        cycle_cnt = -1;
        turtle_state++;
      }

      break;
      case 3:
      // Turn
      v_command.linear.x = 0;
      v_command.angular.z = rot_vel;

      if(cycle_cnt >= turn_cycles-1)
      {
        cycle_cnt = -1;
        turtle_state = 0;
      }

      break;
    }

    pub_vel.publish(v_command);

    cycle_cnt++;
    r.sleep();

  }

  return 0;
}



// publish velocity commands to turtle node
