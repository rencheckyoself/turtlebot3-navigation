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

#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"

#include "rigid2d/diff_drive.hpp"

// Global Variables
static geometry_msgs::Twist twist_cmd;
static rigid2d::WheelVelocities wheel_pos;

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
}

void callback_sensors(nuturtlebot::SensorData::ConstPtr data)
{
  wheel_pos.ul = data.left_encoder;
  wheel_pos.ur = data.right_encoder;
}

nuturtlebot::WheelCommands getWheelCommands(rigid2d::DiffDrive &bot)
{
  rigid2d::Twist2D tw;
  rigid2d::WheelVelocities wv;

  nuturtlebot::WheelCommands whl_cmd;

  // convert to twist format
  tw = GeoTwisttoTwist2D(twist_cmd);

  // get wheel velocities
  wv = bot.twistToWheels(tw)

  // compare to motor max speed
  if(wv.ul > motor_lim)
  {
    wv.ul = motor_lim;
  }
  else if(wv.ur > motor_lim)
  {
    wv.ur = motor_lim;
  }

  whl_cmd.left_velocity = rigid2d::linInterp(wv.ul, [-motor_lim, motor_lim], [-44, 44]);
  whl_cmd.right_velocity = rigid2d::linInterp(wv.ur, [-motor_lim, motor_lim], [-44, 44]);

  return whl_cmd;
}

/// \brief Main function to create the fake_diff_encoders node
///
int main(int argc, char** argv)
{
    // ros initializations
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle n;
    ros::NodeHandle np("~odometer");

    ros::Subscriber twist_sub = n.subscribe("turtle1/cmd_vel", 1, callback_twist);
    ros::Publisher pub_wheels = n.advertise("/wheel_cmd", 1);

    ros::Subscriber sensor_sub = n.subscribe("sensor_data", 1, callback_sensors);
    ros::Publisher pub_encs = n.advertise("/joint_states", 1);

    // Get parameters from the parameter server
    std::string left_wheel_joint, right_wheel_joint;
    double wheel_base = 0, wheel_radius = 0, frequency = 0;
    double tvel_lim = 0, avel_lim = 0, motor_lim = 0;

    np.getParam("/odometer/left_wheel_joint", left_wheel_joint);
    np.getParam("/odometer/right_wheel_joint", right_wheel_joint);

    n.getParam("wheel_radius", wheel_radius);
    n.getParam("wheel_base", wheel_base);
    n.getParam("frequency", frequency);

    n.getParam(("tvel_lim", tvel_lim);
    n.getParam(("avel_lim", avel_lim);
    n.getParam(("motor_lim", motor_lim);

    ROS_INFO_STREAM("Got left wheel joint name: " << left_wheel_joint);
    ROS_INFO_STREAM("Got right wheel joint name: " << right_wheel_joint);

    ROS_INFO_STREAM("Got wheel base param: " << wheel_base);
    ROS_INFO_STREAM("Got wheel radius param: " << wheel_radius);
    ROS_INFO_STREAM("Got frequency param: " << frequency);

    ROS_INFO_STREAM("Got trans vel limit param: " << tvel_lim);
    ROS_INFO_STREAM("Got ang. vel limit param: " << avel_lim);
    ROS_INFO_STREAM("Got motor limit param: " << motor_lim);

    // Create diff drive object to track the robot simulation
    rigid2d::Pose2D pos(0,0,0);
    rigid2d::DiffDrive robot(pos, wheel_base, wheel_radius);

    sensor_msgs::JointState js;
    nuturtlebot::WheelCommands bot_cmd;

    double dt = 1.0/frequency;

    ros::Rate r(frequency);

    while(ros::ok())
    {
      // Use the scaled twist to propigate the robot
      wheel_vels = robot.updateOdometry(wheel_pos);

      // Create the joint state message & publish
      js.header.stamp = ros::Time::now();
      js.name = {left_wheel_joint, right_wheel_joint};
      js.position = {abs_enc.ul, abs_enc.ur};
      js.velocity = {wheel_vels.ul, wheel_vels.ur};

      bot_cmd = getWheelCommands(&robot);

      pub_wheels.publish(bot_cmd)
      pub_joint_state.publish(js);

      ros::spinOnce();
      r.sleep();
    }
}
