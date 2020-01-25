/// \file
/// \brief This node publishes the Odometry messages per ROS standard format
///

#include <iostream>

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#include <boost/ref.hpp>
#include <boost/bind.hpp>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"

using std::cout;
using std::cin;

void callback_joints(const sensor_msgs::JointStates & data, DiffDrive & robot, ros::Publisher pub)
{
    // ros::NodeHandle n;
    //
    // String odom_frame_id, base_frame_id;
    //
    // n.getParam("~odom_frame_id", odom_frame_id);
    // n.getParam("~base_frame_id", base_frame_id);

    // Update internal odometry state of the robot

    // the incoming data is the state of the left and right joints
    // convert the wheel displacement/wheel velocities in an updated position
    // in the DiffDrive class



    // Publish an odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    pub.publish(odom);

    // Broadcast the transform
    geometry_msgs::TransformStamped T_ob;

    T_ob.header.stamp = ros::Time::now();
    T_ob.header.frame_id = "odom";

    T_ob.child_frame_id = "base_link";

    T_ob.transform.translation.x = msg->x;
    T_ob.transform.translation.y = msg->y;
    T_ob.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
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

    Pose2D pos;

    DiffDrive bot(pos, wheel_base, wheel_radius);

    boost::reference_wrapper<DiffDrive>(bot);



    ros::Subscriber joint_sub = n.subscribe("/joint_states", 1, boost::bind(callback_joints, _1, boost::ref(bot), odom_pub);
}
