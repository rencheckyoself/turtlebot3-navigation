/// \file
/// \brief This node interfaces between a main computer and the raspberri pi on the turtlebot
///
/// PARAMETERS:
///     /rotation/frac_val: proportion of max speed to use
///     /avel_lim: max robot angular velocity
/// PUBLISHES:
///     /cmd_vel (geometry_msgs/Twist): the twist command
/// SERIVCES:
///     /start (std_srvs/SetBool): Starts the movement of the robot given an initial direction. 0 is CCC, 1 is CW
///

#include <ros/ros.h>
#include "rigid2d/rigid2d.hpp"

#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include "rigid2d/SetPose.h"
#include "rigid2d/Pose.h"

static double frac_val = 0;
static int cnt = 0;
static int robot_state = 0;
static int cycles_per_rev, cycles_per_wait;
static double robot_vel = 0;
static int total_revs =0;
static bool dir = 1;
static ros::Publisher pub_cmd;

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
    // CCW
    dir = -1;
  }
  else if(req.data == 1)
  {
    // CW
    dir = 1;
  }
  else
  {
    ROS_WARN_STREAM("Direction must be 1 or 0, defaulting to 1");
  }

  ROS_INFO_STREAM("Robot Started");

  robot_state = 1;
  total_revs = 0;
  cnt = 0;

  return 1;
}

void callback_timer(const ros::TimerEvent&)
{

  geometry_msgs::Twist speed_cmd;

  switch (robot_state)
  {
    case 0:
      // ROS_INFO_STREAM("Standby...");
      speed_cmd.angular.z = 0;
      break;

    case 1:
      // ROS_INFO_STREAM("Moving...");
      speed_cmd.angular.z = robot_vel;
      if(cnt >= cycles_per_rev)
      {
        robot_state = 2;
        cnt = 0;
        ROS_INFO_STREAM("Waiting...");
      }
      break;

    case 2:

      speed_cmd.angular.z = 0;
      if(cnt >= cycles_per_wait)
      {
        robot_state = 1;
        cnt = 0;
        total_revs++;
        ROS_INFO_STREAM("Number of revs: " << total_revs);
        ROS_INFO_STREAM("Moving...");
      }

      if(total_revs >= 20)
      {
        robot_state = 0;
        ROS_INFO_STREAM("Done. Going to Standy...");
      }
      break;
  }

  cnt++;
  pub_cmd.publish(speed_cmd);
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "rotation");
  ros::NodeHandle pn("~");
  ros::NodeHandle n;

  double avel_lim = 0;

  pn.getParam("frac_val", frac_val);
  n.getParam("avel_lim", avel_lim);

  ROS_INFO_STREAM("ROTATION: " << frac_val << " " << avel_lim);

  double frequency = 105.0;
  double sec_per_rev = 0;

  robot_vel = avel_lim * frac_val;

  ROS_INFO_STREAM("ROTATION: Robot velocity, " << robot_vel);

  sec_per_rev = (2.0*rigid2d::PI) / robot_vel;

  ROS_INFO_STREAM("ROTATION: Secs Per Rotation, " << sec_per_rev);

  cycles_per_rev = static_cast<int>(std::round(sec_per_rev*frequency));
  cycles_per_wait = cycles_per_rev/20;

  ROS_INFO_STREAM("ROTATION: Move Cycles, " << cycles_per_rev);
  ROS_INFO_STREAM("ROTATION: Wait Cycles, " << cycles_per_wait);

  ROS_INFO_STREAM("Turtlebot Standing by...");
  robot_vel *= dir;
  ros::ServiceServer client_start = n.advertiseService("start", callback_start);
  ros::Timer timer_cmd = n.createTimer(ros::Duration(1.0/frequency), callback_timer);
  pub_cmd = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::spin();
}
