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
///     /set_pose (rigid2d/SetPose): Sets the pose of the robot to the provided values

#include <iostream>

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#include "nuslam/TurtleMap.h"

#include "rigid2d/SetPose.h"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

#include "nuslam/ekf_slam.hpp"

//Global Variables
static sensor_msgs::JointState cur_js;
static nuslam::TurtleMap cur_landmarks;
static int got_odom_data = 0;
static int got_slam_data = 0;
static rigid2d::DiffDrive bot;

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
///
void callback_joints(const sensor_msgs::JointState::ConstPtr data)
{
    cur_js = *data;
    got_odom_data = 1;
}

/// \brief Callback for the set_pose service
bool callback_set_pose(rigid2d::SetPose::Request &req, rigid2d::SetPose::Response &)
{
  rigid2d::Pose2D buf(rigid2d::deg2rad(req.pose.ang), req.pose.x, req.pose.y);
  bot.reset(buf);
  return 1;
}

/// \brief Callback for the landmark data
///
void callback_landmarks(const sensor_msgs::JointState::ConstPtr data)
{
    cur_landmarks = *data;
    got_slam_data = 1;
}

/// \brief Main function for the odometer node
///
int main(int argc, char** argv)
{
    // ros initializations
    ros::init(argc, argv, "slam");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

// ODOMETRY INITIALIZAIONS /////////////////////////////////////////////////////
    ros::Subscriber joint_sub = n.subscribe<sensor_msgs::JointState>("joint_states", 1, callback_joints);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
    ros::Publisher odom_path_pub = n.advertise<nav_msgs::Path>("odom_path", 1);

    ros::ServiceServer srv_set_pose = n.advertiseService("set_pose", callback_set_pose);

    tf2_ros::TransformBroadcaster odom_br;

    std::string odom_frame_id, base_frame_id, left_wheel_joint, right_wheel_joint;;
    double frequency;
    double wheel_base, wheel_radius;

    // Get private parameters
    pn.getParam("odom_frame_id", odom_frame_id);
    pn.getParam("base_frame_id", base_frame_id);
    pn.getParam("left_wheel_joint", left_wheel_joint);
    pn.getParam("right_wheel_joint", right_wheel_joint);
    n.getParam("/frequency", frequency);
    n.getParam("/wheel_radius", wheel_radius);
    n.getParam("/wheel_base", wheel_base);

    ROS_INFO_STREAM("ODOM: Got odom frame id: " << odom_frame_id);
    ROS_INFO_STREAM("ODOM: Got base frame id: " << base_frame_id);
    ROS_INFO_STREAM("ODOM: Got left wheel joint name: " << left_wheel_joint);
    ROS_INFO_STREAM("ODOM: Got right wheel joint name: " << right_wheel_joint);
    ROS_INFO_STREAM("ODOM: Got wheel base param: " << wheel_base);
    ROS_INFO_STREAM("ODOM: Got wheel radius param: " << wheel_radius);
    ROS_INFO_STREAM("ODOM: Got frequency param: " << frequency);

    // Create diff drive object to simuate the robot
    rigid2d::Pose2D pos;
    rigid2d::DiffDrive bufbot(pos, wheel_base, wheel_radius);

    bot = bufbot;

    ros::Rate r(frequency);

    std::vector<std::string> joints;
    int lw_i, rw_i;

    rigid2d::Twist2D tw;
    tf2::Quaternion q;
    rigid2d::WheelVelocities cmd;
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion q_geo;
    geometry_msgs::TransformStamped T_ob;

    geometry_msgs::PoseStamped odom_point;
    std::vector<geometry_msgs::PoseStamped> odom_points;
    nav_msgs::Path odom_path;

// SLAM INITIALIZAIONS /////////////////////////////////////////////////////////
    ros::Subscriber landmark_sub = n.subscribe<nuslam::TurtleMap>("landmarks", 1, callback_landmarks);
    ros::Publisher slam_path_pub = n.advertise<nav_msgs::Path>("slam_path", 1);

    int num_landmarks = 0;
    std::string map_frame_id;

    pn.getParam("num_landmarks", num_landmarks);
    pn.getParam("map_frame_id", map_frame_id)

    Eigen::Matrix3d Qnoise << 1, 0, 0,
                              0, 1, 0,
                              0, 0, 1;

    Eigen::Matrix3d Rnoise << 1, 0, 0,
                              0, 1, 0,
                              0, 0, 1;

    ROS_INFO_STREAM("SLAM: Got number of landmarks: " << num_landmarks);
    ROS_INFO_STREAM("SLAM: Got map frame id: " << map_frame_id);

    ekf_slam::Slam robot(num_landmarks, Qnoise, Rnoise);

    std::vector<double> slam_pose;

    geometry_msgs::PoseStamped slam_point;
    std::vector<geometry_msgs::PoseStamped> slam_points;
    nav_msgs::Path slam_path;



    while(ros::ok())
    {

      ros::spinOnce();

/////// ODOMETRY CALCULATIONS //////////////////////////////////////////////////
      if(got_odom_data == 1)
      {

        // get the index of each wheel
        joints = cur_js.name;
        lw_i = findJointIndex(joints, left_wheel_joint);
        rw_i = findJointIndex(joints, right_wheel_joint);

        // Update the odometer of the robot using the new wheel positions
        cmd = (bot.updateOdometry(cur_js.position[lw_i], cur_js.position[rw_i]));
        tw = bot.wheelsToTwist(cmd);

        // Get info to fill out the message and transform
        pos = bot.pose();
        q.setRPY(0, 0, pos.th);

        // Publish an odometry message
        q_geo = tf2::toMsg(q);
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
        odom_point.header.frame_id = odom_frame_id;
        odom_point.header.stamp = ros::Time::now();

        odom_point.pose  = odom.pose;

        odom_points.push_back(odom_point);

        odom_path.header.frame_id = odom_frame_id;
        odom_path.header.stamp = ros::Time::now();

        odom_path_pub.publish(odom_path);

        // Broadcast the transform
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

        got_odom_data = 0;
      }
/////// SLAM CALCULATIONS //////////////////////////////////////////////////////
      if(got_slam_data == 1)
      {

        robot.MotionModelUpdate(tw);
        robot.MeasurmentModelUpdate(cur_landmarks);

        // Braodcast Map to Odom Frame


        // Publish Path Message
        slam_pose = robot.getRobotState();

        slam_point.header.frame_id = map_frame_id
        slam_point.header.stamp = ros::Time::now();

        slam_point.pose.position.x = slam_pose.at(1);
        slam_point.pose.position.y = slam_pose.at(2);

        q.setRPY(0, 0, slam_pose.at(0));
        q_geo = tf2::toMsg(q);

        slam_point.pose.orientation = q_geo;

        slam_points.push_back(slam_point);

        slam_path.header.stamp = ros::Time::now();
        slam_path.header.frame_id = map_frame_id;

        slam_path.poses = slam_points;

        slam_path_pub.publish(slam_path);


        got_slam_data = 0;
      }
  }
}
