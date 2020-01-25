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
///     /turtle1/cmd_vel (geometry_msgs/Twist): Retrieves the current twist of the turtle

#include <iostream>

#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include <sensor_msgs/JointState.h>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

// Global Variables
ros::Publisher pub_joint_state;

std::string left_wheel_joint, right_wheel_joint;

double wheel_base, wheel_radius;

rigid2d::Pose2D pos(0,0,0);

rigid2d::DiffDrive OnlyForFunction;

ros::Time cur_t, last_t;

// Callbacks
void callback_twist(geometry_msgs::Twist::ConstPtr data)
{

  cur_t = ros::Time::now();

  rigid2d::Twist2D twist_cmd;

  sensor_msgs::JointState js;

  double dt;

  dt = (cur_t - last_t).toSec();

  twist_cmd.wz = (data->angular.z) * dt;
  twist_cmd.vx = data->linear.x * dt;
  twist_cmd.vy = data->linear.y * dt;

  rigid2d::WheelVelocities wheel_vels = OnlyForFunction.twistToWheels(twist_cmd);

  js.name = {left_wheel_joint, right_wheel_joint};
  js.position = {wheel_vels.ul*dt, wheel_vels.ur*dt};
  js.velocity = {wheel_vels.ul, wheel_vels.ur};

  pub_joint_state.publish(js);
  cur_t = last_t;
}

int main(int argc, char** argv)
{
    // ros initializations
    ros::init(argc, argv, "fake_encoders");
    ros::NodeHandle n;

    n.getParam("~odometer/left_wheel_joint", left_wheel_joint);
    n.getParam("~odometer/right_wheel_joint", right_wheel_joint);

    n.getParam("wheel_radius", wheel_radius);
    n.getParam("wheel_base", wheel_base);

    ROS_INFO("Got wheel base param: %f", wheel_base);
    ROS_INFO("Got wheel radius param: %f", wheel_radius);
    ROS_INFO("Got left wheel joint name: %s", left_wheel_joint.c_str());
    ROS_INFO("Got right wheel joint name: %s", right_wheel_joint.c_str());

    OnlyForFunction.setRadius(wheel_radius);
    OnlyForFunction.setBase(wheel_base);

    cur_t = ros::Time::now();
    last_t = ros::Time::now();

    ros::Subscriber twist_sub = n.subscribe("turtle1/cmd_vel", 1, callback_twist);
    ros::Publisher pub_joint_state = n.advertise<sensor_msgs::JointState>("odom", 10);

    ros::spin();
}
