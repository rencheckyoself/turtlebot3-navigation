/// \file
/// \brief This node interfaces between a main computer and the raspberri pi on the turtlebot
///
/// PARAMETERS:
/// PUBLISHES:
///     /wheel_cmd (nutrutlebot/WheelCommands): a command to control the motors on the turtlebot
///     /joint_states (sensor_msgs/JointState): the wheel position and velocities of the turtlebot
/// SUBSCRIBES:
///     /cmd_vel (geometry_msgs/Twist): the twist command from the turtlesim package
///     /sensor_data (nuturtlebot/SensorData): retrives sensor info from the turtlebot

#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include "nuturtlebot/SensorData.h"
#include "nuturtlebot/WheelCommands.h"

#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"

// Global Variables
static nuturtlebot::WheelCommands bot_cmd;
static geometry_msgs::Twist twist_cmd;
static rigid2d::DiffDrive robot;

static std::string left_wheel_joint, right_wheel_joint;
static double tvel_lim, avel_lim, motor_lim;
static int frequency = 0;
static int encoder_ticks_per_rev = 0;
static double motor_power = 0;
static ros::Publisher pub_wheels, pub_encs;

nuturtlebot::WheelCommands getWheelCommands()
{
  rigid2d::Twist2D tw;
  rigid2d::WheelVelocities wv;

  nuturtlebot::WheelCommands whl_cmd;

  // convert to twist format
  tw = rigid2d::GeoTwisttoTwist2D(twist_cmd);

  // get wheel velocities
  wv = robot.twistToWheels(tw);

  // compare to motor max speed
  if(wv.ul > motor_lim)
  {
    wv.ul = motor_lim;
  }
  else if(wv.ur > motor_lim)
  {
    wv.ur = motor_lim;
  }

  double m_lim[2] = {-motor_lim, motor_lim};
  double cmd_lim[2] = {-motor_power, motor_power};

  whl_cmd.left_velocity = rigid2d::linInterp(wv.ul, m_lim, cmd_lim);
  whl_cmd.right_velocity = rigid2d::linInterp(wv.ur, m_lim, cmd_lim);

  pub_wheels.publish(whl_cmd);

  return whl_cmd;
}

/// \brief callback funtion for the /turtle1/cmd_vel subscriber
///
void callback_twist(geometry_msgs::Twist::ConstPtr data)
{

  twist_cmd = *data;

  // Check for velocity limit violations
  if(twist_cmd.linear.x > tvel_lim)
  {
    twist_cmd.linear.x = tvel_lim;
  }

  if(twist_cmd.angular.z > avel_lim)
  {
    twist_cmd.angular.z = avel_lim;
  }

  bot_cmd = getWheelCommands();
}

void callback_sensors(nuturtlebot::SensorData::ConstPtr data)
{
  rigid2d::WheelVelocities data_conv, wheel_vels, abs_enc;
  sensor_msgs::JointState js;

  data_conv.ul = static_cast<double>(data->left_encoder)/encoder_ticks_per_rev * 2.0 * rigid2d::PI;
  data_conv.ur = static_cast<double>(data->right_encoder)/encoder_ticks_per_rev * 2.0 * rigid2d::PI;

  wheel_vels = robot.updateOdometry(data_conv.ul, data_conv.ur);
  abs_enc = robot.getEncoders();

  // Create the joint state message & publish
  js.header.stamp = ros::Time::now();
  js.name = {left_wheel_joint, right_wheel_joint};
  js.position = {abs_enc.ul, abs_enc.ur};
  js.velocity = {wheel_vels.ul, wheel_vels.ur};

  pub_encs.publish(js);
}


/// \brief Main function to create the fake_diff_encoders node
///
int main(int argc, char** argv)
{
    // ros initializations
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle n;

    ros::Subscriber twist_sub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, callback_twist);
    pub_wheels = n.advertise<nuturtlebot::WheelCommands>("wheel_cmd", 1);

    ros::Subscriber sensor_sub = n.subscribe("sensor_data", 1, callback_sensors);
    pub_encs = n.advertise<sensor_msgs::JointState>("joint_states", 1);

    // Get parameters from the parameter server
    double wheel_base = 0, wheel_radius = 0;

    n.getParam("left_wheel_joint", left_wheel_joint);
    n.getParam("right_wheel_joint", right_wheel_joint);

    n.getParam("wheel_radius", wheel_radius);
    n.getParam("wheel_base", wheel_base);
    n.getParam("frequency", frequency);

    n.getParam("tvel_lim", tvel_lim);
    n.getParam("avel_lim", avel_lim);
    n.getParam("motor_lim", motor_lim);
    n.getParam("encoder_ticks_per_rev", encoder_ticks_per_rev);
    n.getParam("motor_power", motor_power);

    ROS_INFO_STREAM("T_INT: Got left wheel joint name: " << left_wheel_joint);
    ROS_INFO_STREAM("T_INT: Got right wheel joint name: " << right_wheel_joint);

    ROS_INFO_STREAM("T_INT: Got wheel base param: " << wheel_base);
    ROS_INFO_STREAM("T_INT: Got wheel radius param: " << wheel_radius);
    ROS_INFO_STREAM("T_INT: Got frequency param: " << frequency);

    ROS_INFO_STREAM("T_INT: Got trans vel limit param: " << tvel_lim);
    ROS_INFO_STREAM("T_INT: Got ang. vel limit param: " << avel_lim);
    ROS_INFO_STREAM("T_INT: Got motor limit param: " << motor_lim);

    ROS_INFO_STREAM("T_INT: Got motor power param: " << motor_power);
    ROS_INFO_STREAM("T_INT: Got encoder spec param: " << encoder_ticks_per_rev);

    // Create diff drive object to track the robot simulation
    rigid2d::Pose2D pos(0,0,0);
    rigid2d::DiffDrive robot_buf(pos, wheel_base, wheel_radius);

    robot = robot_buf;

    ros::spin();
}
