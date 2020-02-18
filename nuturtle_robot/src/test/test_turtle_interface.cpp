/// \file
/// \brief This node tests the turtle interface node
///
/// PUBLISHES:
///     /cmd_vel (geometry_msgs/Twist): the twist command from the turtlesim package
///     /sensor_data (nuturtlebot/SensorData): retrives sensor info from the turtlebot
/// SUBSCRIBES:
///     /wheel_cmd (nutrutlebot/WheelCommands): a command to control the motors on the turtlebot
///     /joint_states (sensor_msgs/JointState): the wheel position and velocities of the turtlebot

#include <gtest/gtest.h>
#include <sstream>
#include <ros/ros.h>

#include "rigid2d/rigid2d.hpp"
#include "geometry_msgs/Twist.h"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "sensor_msgs/JointState.h"

static char got_wheel_data = 0;
static char got_sensor_data = 0;

static nuturtlebot::WheelCommands wheel_cmd;
static sensor_msgs::JointState js_data;

static ros::Subscriber sub_wheelcmd, sub_joints;
static ros::Publisher pub_cmd_vel, pub_sensors;

/// \breif Callback for wheel_cmd subscriber
void callback_wheels(nuturtlebot::WheelCommands::ConstPtr data)
{
  wheel_cmd = *data;
  got_wheel_data = 1;
}

/// \breif Callback for joint_states subscriber
void callback_joints(sensor_msgs::JointState::ConstPtr data)
{
  js_data = *data;
  got_sensor_data = 1;
}

TEST(TurtleInterface, TransOnly)
{
  // Proper result for a cmd_vel message with no rotational component
  geometry_msgs::Twist tw;
  tw.linear.x = 0.1;

  pub_cmd_vel.publish(tw);

  got_wheel_data = 0;
  while(got_wheel_data == 0)
  {
      ros::spinOnce();
      // wait for data to be recieved...
  }

  ASSERT_NEAR(wheel_cmd.left_velocity, 126, 1e-4);
  ASSERT_NEAR(wheel_cmd.right_velocity, 126, 1e-4);
}

TEST(TurtleInterface, RotOnly)
{
  // Proper result for a cmd_vel message with no translational component

  geometry_msgs::Twist tw;
  tw.angular.z = 1;

  pub_cmd_vel.publish(tw);

  got_wheel_data = 0;
  while(got_wheel_data == 0)
  {
      ros::spinOnce();
      // wait for data to be recieved...
  }

  ASSERT_NEAR(wheel_cmd.left_velocity, -101, 1e-4);
  ASSERT_NEAR(wheel_cmd.right_velocity, 101, 1e-4);
}

TEST(TurtleInterface, RotAndTrans)
{
  // Proper result for a typical cmd_vel message
  geometry_msgs::Twist tw;
  tw.linear.x = 0.1;
  tw.angular.z = 1;

  pub_cmd_vel.publish(tw);

  got_wheel_data = 0;
  while(got_wheel_data == 0)
  {
      ros::spinOnce();
      // wait for data to be recieved...
  }

  ASSERT_NEAR(wheel_cmd.left_velocity, 25, 1e-4);
  ASSERT_NEAR(wheel_cmd.right_velocity, 227, 1e-4);
}

TEST(TurtleInterface, ValidEncs)
{
  // Proper result for a sensor message to joint_states conversion
  nuturtlebot::SensorData enc_vals;

  enc_vals.left_encoder = 4096/4;
  enc_vals.right_encoder = 4096/4;

  pub_sensors.publish(enc_vals);

  got_sensor_data = 0;
  while(got_sensor_data == 0)
  {
      ros::spinOnce();
      // wait for data to be recieved...
  }

  ASSERT_NEAR(js_data.position.at(0), rigid2d::PI/2, 1e-4);
  ASSERT_NEAR(js_data.position.at(1), rigid2d::PI/2, 1e-4);
  ASSERT_NEAR(js_data.velocity.at(0), rigid2d::PI/2, 1e-4);
  ASSERT_NEAR(js_data.velocity.at(1), rigid2d::PI/2, 1e-4);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "turtle_interface_test");
  ros::NodeHandle n;

  pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  sub_wheelcmd = n.subscribe<nuturtlebot::WheelCommands>("wheel_cmd", 1, callback_wheels);

  pub_sensors = n.advertise<nuturtlebot::SensorData>("sensor_data", 1, true);
  sub_joints = n.subscribe("/joint_states", 1, callback_joints);

  return RUN_ALL_TESTS();
}
