/// \file
/// \brief This node interfaces between a main computer and the raspberri pi on the turtlebot
///
/// PARAMETERS:
///     /rotation/frac_val: proportion of max speed to use
///     /avel_lim: max robot angular velocity
/// PUBLISHES:
///     /cmd_vel (geometry_msgs/Twist): the twist command
/// SERIVCES:
///     /start (std_srvs/SetBool): Starts the movement of the robot given an initial direction. 0 is CCW/Forwards, 1 is CW/Backwards
///

#include <ros/ros.h>
#include "rigid2d/rigid2d.hpp"

#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include "rigid2d/SetPose.h"
#include "rigid2d/Pose.h"

static ros::Publisher pub_cmd;
static double frac_val = 0;
static int cnt = 0;
static int robot_state = 0;
static int cycles_per_move, cycles_per_wait;
static double robot_vel = 0;
static int total_moves = 0;
static int dir = 1;
static int move_limit = 0;
static int motion_type = 0;

bool callback_start(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response&)
{

  ros::NodeHandle temp_n;
  ros::ServiceClient temp_pose = temp_n.serviceClient<rigid2d::SetPose>("set_pose");

  ros::service::waitForService("set_pose");

  rigid2d::SetPose ps;
  ps.request.pose.x = 0;
  ps.request.pose.y = 0;
  ps.request.pose.ang = 0;

  temp_pose.call(ps);

  if(req.data == 0)
  {
    // CCW/Forwards
    dir = -1;
  }
  else if(req.data == 1)
  {
    // CW/Backwards
    dir = 1;
  }
  else
  {
    ROS_WARN_STREAM("Direction must be 1 or 0, defaulting to 1");
  }

  // ROS_INFO_STREAM("Robot Started with direction " << dir);

  robot_state = 1;
  total_moves = 0;
  cnt = 0;
  robot_vel *= dir;

  return 1;
}

void callback_timer(const ros::TimerEvent&)
{

  geometry_msgs::Twist speed_cmd;
  double cmd_val = 0;

  switch (robot_state)
  {
    case 0:
      // ROS_INFO_STREAM("Standby...");
      cmd_val  = 0;
      break;

    case 1:
      // ROS_INFO_STREAM("Moving...");
      cmd_val = robot_vel;
      if(cnt >= cycles_per_move)
      {
        robot_state = 2;
        cnt = 0;
        // ROS_INFO_STREAM("Waiting...");
      }
      break;

    case 2:

      cmd_val = 0;
      if(cnt >= cycles_per_wait)
      {
        robot_state = 1;
        cnt = 0;
        total_moves++;
        // ROS_INFO_STREAM("Number of revs: " << total_moves);
        // ROS_INFO_STREAM("Moving...");
      }

      if(total_moves >= move_limit)
      {
        robot_state = 0;
        // ROS_INFO_STREAM("Done. Going to Standy...");
      }
      break;
  }

  cnt++;
  if(motion_type == 0)
  {
    speed_cmd.angular.z = cmd_val;
  }
  else if(motion_type == 1)
  {
    speed_cmd.linear.x = cmd_val;
  }
  else
  {
    ROS_WARN_STREAM("Should not be here...something changes the motion types variable.");
  }

  pub_cmd.publish(speed_cmd);
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "rotation");
  ros::NodeHandle pn("~");
  ros::NodeHandle n;

  double avel_lim = 0;
  double tvel_lim = 0;
  double frequency = 0;

  pn.getParam("frac_val", frac_val);
  pn.getParam("motion_type", motion_type);
  n.getParam("avel_lim", avel_lim);
  n.getParam("tvel_lim", tvel_lim);
  n.getParam("frequency", frequency);

  ROS_INFO_STREAM("ROTATION: Got Frac Val: " << frac_val);
  ROS_INFO_STREAM("ROTATION: Got Lin. Vel Limit: " << tvel_lim);
  ROS_INFO_STREAM("ROTATION: Got Ang. Vel Limit: " << avel_lim);
  ROS_INFO_STREAM("ROTATION: Frequency: " << frequency);
  ROS_INFO_STREAM("ROTATION: Motion Type: " << motion_type);

  double sec_per_path = 0;

  if(motion_type != 0 && motion_type != 1)
  {
    ROS_WARN_STREAM("Motion type must be either 1 or 0. defaulting to 0, rotational motion.");
    motion_type = 0;
  }

  if(motion_type == 1)
  {
    // Linear Motion Vel Cmd
    robot_vel = tvel_lim * frac_val;

    // seconds to traverse 0.2m
    sec_per_path = 0.2 / robot_vel;

    // Cycles to move 0.2m
    cycles_per_move = static_cast<int>(std::round(sec_per_path*frequency));

    // Cycles to pause
    cycles_per_wait = cycles_per_move/20;

    // Set motion distance to 2m
    move_limit = 10;
  }
  else if(motion_type == 0)
  {
    // Rotational Motion Vel Cmd
    robot_vel = avel_lim * frac_val;

    // Seconds to complete a rotation
    sec_per_path = (2.0*rigid2d::PI) / robot_vel;

    // Cycles to move one rotation
    cycles_per_move = static_cast<int>(std::round(sec_per_path*frequency));

    // Cycles to wait 1/20th of a rotation
    cycles_per_wait = cycles_per_move/20;

    // Set motion distance to 20 revs
    move_limit = 20;
  }

  ROS_INFO_STREAM("ROTATION: Nominal robot velocity, " << robot_vel);
  // ROS_INFO_STREAM("ROTATION: Secs Per Motion, " << sec_per_path);
  // ROS_INFO_STREAM("ROTATION: Move Cycles, " << cycles_per_move);
  // ROS_INFO_STREAM("ROTATION: Wait Cycles, " << cycles_per_wait);

  ROS_INFO_STREAM("Turtlebot Standing by...");

  ros::ServiceServer client_start = n.advertiseService("start", callback_start);
  ros::Timer timer_cmd = n.createTimer(ros::Duration(1.0/frequency), callback_timer);
  pub_cmd = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::spin();
}
