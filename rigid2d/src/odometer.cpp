/// \file
/// \brief This node publishes the Odometry messages per ROS standard format
///
/// PARAMETERS:
///     odom_frame_id (std::string) the name of the odometer frame
///     base_frame_id (std::string) the name of the base frame
///     left_wheel_joint (std::string) the name of the left wheel joint
///     right_wheel_joint (std::string) the name of the right wheel joint
///     wheel_base (double) the distance between the two wheels of the diff drive robot
///     wheel_radius (double) the radius of the wheels
///     frequency (double) the frequency to publish joint states at
/// PUBLISHES:
///     /odom (nav_msgs/Odometry): The calculated odometry of the diff drive robot
/// SUBSCRIBES:
///     /joint_states (sensor_msgs/JointState): Retrieves the calculated wheel velocities and change in wheel position
/// SERVICES:
///     /set_odom (rigid2d/SetOdom): Sets the odometry of the robot to the provided values

#include <iostream>

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#include "rigid2d/SetPose.srv"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

//Global Variables
static sensor_msgs::JointState cur_js;
static int got_data = 0;

/// \brief Use to search through the all joint names and return the index of the desired joint
/// \param joints - a vector of all the joint names
/// \param target - the desire joint name to find
/// \return the index of the desired joint name
int findJointIndex(std::vector<std::string> joints, std::string target)
{
    std::vector<std::string>::iterator find_joint;
    find_joint = std::find(joints.begin(), joints.end(), target);

    return std::distance(joints.begin(), find_joint);
}

/// \brief Callback for the joint state subscriber
void callback_joints(const sensor_msgs::JointState::ConstPtr data)
{
    cur_js = *data;
    got_data = 1;
}

/// \brief Callback for the set_pose service
void callback_set_pose(rigid2d::SetPose::Request &req, rigid2d::SetPose::Response &res, DiffDrive &robot_obj)
{

  rigid2d::Pose2D buf(rigid2d::deg2rad(req.th), req.x, req.y)
  robot_obj.reset(buf)
}

/// \brief Main function for the odometer node
///
int main(int argc, char** argv)
{
    // ros initializations
    ros::init(argc, argv, "odometer");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    ros::Subscriber joint_sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, callback_joints);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);


    tf2_ros::TransformBroadcaster odom_br;

    std::string odom_frame_id, base_frame_id, left_wheel_joint, right_wheel_joint;;
    double frequency;
    double wheel_base, wheel_radius;

    // Get private parameters
    pn.getParam("odom_frame_id", odom_frame_id);
    pn.getParam("base_frame_id", base_frame_id);
    pn.getParam("left_wheel_joint", left_wheel_joint);
    pn.getParam("right_wheel_joint", right_wheel_joint);
    n.getParam("frequency", frequency);
    n.getParam("wheel_radius", wheel_radius);
    n.getParam("wheel_base", wheel_base);

    ROS_INFO_STREAM("Odom Got odom frame id: " << odom_frame_id);
    ROS_INFO_STREAM("Odom Got base frame id: " << base_frame_id);
    ROS_INFO_STREAM("Odom Got left wheel joint name: " << left_wheel_joint);
    ROS_INFO_STREAM("Odom Got right wheel joint name: " << right_wheel_joint);
    ROS_INFO_STREAM("Odom Got wheel base param: " << wheel_base);
    ROS_INFO_STREAM("Odom Got wheel radius param: " << wheel_radius);
    ROS_INFO_STREAM("Odom Got frequency param: " << frequency);

    // Create diff drive object to simuate the robot
    rigid2d::Pose2D pos;
    rigid2d::DiffDrive bot(pos, wheel_base, wheel_radius);

    ros::ServiceServer srv_set_pose = n.advertiseService("set_pose", boost::bind(callback_set_pose, _1, _2, bot);

    ros::Rate r(frequency);

    std::vector<std::string> joints;
    int lw_i, rw_i;

    while(ros::ok())
    {

      ros::spinOnce();

      if(got_data == 1)
      {

      // get the index of each wheel
      joints = cur_js.name;
      lw_i = findJointIndex(joints, left_wheel_joint);
      rw_i = findJointIndex(joints, right_wheel_joint);

      // Update the odometer of the robot using the new wheel positions
      rigid2d::WheelVelocities cmd = (bot.updateOdometry(cur_js.position[lw_i], cur_js.position[rw_i]));
      rigid2d::Twist2D tw = bot.wheelsToTwist(cmd);

      // Get info to fill out the message and transform
      rigid2d::Pose2D pos = bot.pose();
      tf2::Quaternion q;
      q.setRPY(0, 0, pos.th);

      // Publish an odometry message
      nav_msgs::Odometry odom;
      geometry_msgs::Quaternion q_geo = tf2::toMsg(q);
      odom.header.stamp = ros::Time::now();
      odom.header.frame_id = odom_frame_id;

      odom.pose.pose.position.x = pos.x;
      odom.pose.pose.position.y = pos.y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = q_geo;

      odom.child_frame_id = base_frame_id;
      odom.twist.twist.linear.x = tw.vx;
      odom.twist.twist.linear.y = tw.vy;
      odom.twist.twist.angular.z = tw.wz;

      odom_pub.publish(odom);

      // Broadcast the transform
      geometry_msgs::TransformStamped T_ob;

      T_ob.header.stamp = ros::Time::now();
      T_ob.header.frame_id = odom_frame_id;

      T_ob.child_frame_id = base_frame_id;

      T_ob.transform.translation.x = pos.x;
      T_ob.transform.translation.y = pos.y;
      T_ob.transform.translation.z = 0.0;

      T_ob.transform.rotation.x = q.x();
      T_ob.transform.rotation.y = q.y();
      T_ob.transform.rotation.z = q.z();
      T_ob.transform.rotation.w = q.w();

      odom_br.sendTransform(T_ob);

      got_data = 0;
    }
  }
}
