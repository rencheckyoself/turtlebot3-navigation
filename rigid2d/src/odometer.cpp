/// \file
/// \brief This node publishes the Odometry messages per ROS standard format
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
///     /turtle1/pose (turtlesim/Pose): Retrieves the current pose of the turtle

#include <iostream>

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#include <boost/ref.hpp>
#include <boost/bind.hpp>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

using std::cout;
using std::cin;

int findJointIndex(std::vector<std::string> joints, std::string target)
{
    std::vector<std::string>::iterator find_joint;

    find_joint = std::find(joints.begin(), joints.end(), target);

    return std::distance(joints.begin(), find_joint);
}

void callback_joints(const sensor_msgs::JointState::ConstPtr & data, rigid2d::DiffDrive & robot, ros::Publisher pub, tf2_ros::TransformBroadcaster br)
{
    // ros::NodeHandle n;
    //
    // std::string odom_frame_id, base_frame_id;
    //
    // n.getParam("~odom_frame_id", odom_frame_id);
    // n.getParam("~base_frame_id", base_frame_id);

    // Update internal odometry state of the robot

    std::vector<std::string> joints = data->name;
    int lw_i, rw_i;

    lw_i = findJointIndex(joints, "left_wheel_axel");
    rw_i = findJointIndex(joints, "right_wheel_axel");

    robot.updateOdometry(data->position[lw_i], data->position[rw_i]);

    rigid2d::WheelVelocities cmd(data->velocity[lw_i], data->velocity[rw_i]);
    rigid2d::Twist2D tw = robot.wheelsToTwist(cmd);

    rigid2d::Pose2D pos = robot.pose();

    tf2::Quaternion q;
    q.setRPY(0, 0, pos.th);

    // Publish an odometry message
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion q_geo = tf2::toMsg(q);
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = pos.x;
    odom.pose.pose.position.y = pos.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = q_geo;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = tw.vx;
    odom.twist.twist.linear.y = tw.vy;
    odom.twist.twist.angular.z = tw.wz;

    pub.publish(odom);

    // Broadcast the transform
    geometry_msgs::TransformStamped T_ob;

    T_ob.header.stamp = ros::Time::now();
    T_ob.header.frame_id = "odom";

    T_ob.child_frame_id = "base_link";

    T_ob.transform.translation.x = pos.x;
    T_ob.transform.translation.y = pos.y;
    T_ob.transform.translation.z = 0.0;

    T_ob.transform.rotation.x = q.x();
    T_ob.transform.rotation.y = q.y();
    T_ob.transform.rotation.z = q.z();
    T_ob.transform.rotation.w = q.w();

    br.sendTransform(T_ob);
}

int main(int argc, char** argv)
{
    // ros initializations
    ros::init(argc, argv, "odometer");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    tf2_ros::TransformBroadcaster odom_br;

    // Set private parameters for the node
    pn.setParam("odom_frame_id", "odom");
    pn.setParam("body_frame_id", "base_link");
    pn.setParam("left_wheel_joint", "left_wheel_axel");
    pn.setParam("right_wheel_joint", "right_wheel_axel");

    // Get parameters from server
    double wheel_base, wheel_radius;

    n.getParam("wheel_radius", wheel_radius);
    n.getParam("wheel_base", wheel_base);

    ROS_INFO("Got wheel base param: %f", wheel_base);
    ROS_INFO("Got wheel radius param: %f", wheel_radius);

    rigid2d::Pose2D pos;
    rigid2d::DiffDrive bot(pos, wheel_base, wheel_radius);

    ros::Subscriber joint_sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, boost::bind(callback_joints, _1, boost::ref(bot), odom_pub, odom_br));

    ros::spin();
}
