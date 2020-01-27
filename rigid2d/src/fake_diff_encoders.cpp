/// \file
/// \brief This file contains the source code for the turtle_rect node. It pulls in parameters set from the yaml file and uses feed forward control to drive the turtle in a rectangle. It also calculates the positional error over time.
///
/// PARAMETERS:
///     left_wheel_joint (std::string) the name of the left wheel joint
///     right_wheel_joint (std::string) the name of the right wheel joint
///     wheel_base (double) the distance between the two wheels of the diff drive robot
///     wheel_radius (double) the radius of the wheels
/// PUBLISHES:
///     /joint_states (sensor_msgs/JointState) publishs the scaled wheel velocities and the resulting distance of rotation for wheels moving at that velocity
/// SUBSCRIBES:
///     /turtle1/cmd_vel (geometry_msgs/Twist): Retrieves the current twist of the turtle

#include <iostream>

#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include <sensor_msgs/JointState.h>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"

// Global Variables
geometry_msgs::Twist twist_cmd;
rigid2d::Twist2D twist_rg;

// Callbacks
void callback_twist(geometry_msgs::Twist::ConstPtr data)
{
  twist_cmd = *data;
  twist_rg = rigid2d::GeoTwisttoTwist2D(twist_cmd);
}

int main(int argc, char** argv)
{
    // ros initializations
    ros::init(argc, argv, "fake_encoders");
    ros::NodeHandle n;
    ros::NodeHandle np("~odometer");

    std::string left_wheel_joint, right_wheel_joint;
    double wheel_base, wheel_radius, frequency;

    np.getParam("/odometer/left_wheel_joint", left_wheel_joint);
    np.getParam("/odometer/right_wheel_joint", right_wheel_joint);
    n.getParam("wheel_radius", wheel_radius);
    n.getParam("wheel_base", wheel_base);
    n.getParam("frequency", frequency);

    ROS_INFO_STREAM("Got wheel base param: " << wheel_base);
    ROS_INFO_STREAM("Got wheel radius param: " << wheel_radius);
    ROS_INFO_STREAM("Got left wheel joint name: " << left_wheel_joint);
    ROS_INFO_STREAM("Got right wheel joint name: " << right_wheel_joint);

    rigid2d::Pose2D pos(0,0,0);
    rigid2d::DiffDrive robot(pos, wheel_base, wheel_radius);

    ros::Subscriber twist_sub = n.subscribe("turtle1/cmd_vel", 1, callback_twist);
    ros::Publisher pub_joint_state = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

    sensor_msgs::JointState js;
    double dt = 1.0/frequency;

    ros::Rate r(frequency);

    while(ros::ok())
    {

      robot.feedforward(twist_rg.scaleTwist(dt));

      rigid2d::WheelVelocities abs_enc = robot.getEncoders();

      js.header.stamp = ros::Time::now();
      js.name = {left_wheel_joint, right_wheel_joint};
      js.position = {abs_enc.ul, abs_enc.ur};

      pub_joint_state.publish(js);

      ros::spinOnce();
      r.sleep();
    }
}
