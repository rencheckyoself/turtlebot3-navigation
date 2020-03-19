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
#include <nav_msgs/Path.h>
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

static double ekf_enc_l = 0.0, ekf_enc_r = 0.0;

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

/// \brief Callback for the landmark data
///
void callback_landmarks(const nuslam::TurtleMap::ConstPtr data)
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
    // ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
    ros::Publisher odom_path_pub = n.advertise<nav_msgs::Path>("odom_path", 1);

    std::string odom_frame_id, base_frame_id, left_wheel_joint, right_wheel_joint;
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
    rigid2d::Twist2D tw;
    rigid2d::WheelVelocities cmd;
    rigid2d::DiffDrive bufbot(pos, wheel_base, wheel_radius);

    bot = bufbot;

    ros::Rate r(frequency);

    std::vector<std::string> joints;
    int lw_i, rw_i;

    tf2::Quaternion q;
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion q_geo;
    geometry_msgs::TransformStamped T_ob;

    geometry_msgs::PoseStamped odom_point;
    std::vector<geometry_msgs::PoseStamped> odom_points;
    nav_msgs::Path odom_path;

// SLAM INITIALIZAIONS /////////////////////////////////////////////////////////
    ros::Subscriber landmark_sub = n.subscribe<nuslam::TurtleMap>("landmark_data", 1, callback_landmarks);

    ros::Publisher slam_path_pub = n.advertise<nav_msgs::Path>("slam_path", 1);
    ros::Publisher slam_landmark_pub = n.advertise<nuslam::TurtleMap>("slam_landmark_data", 1);

    tf2_ros::TransformBroadcaster Tmo_br;

    int num_landmarks = 0;
    std::string map_frame_id;

    pn.getParam("num_landmarks", num_landmarks);
    pn.getParam("map_frame_id", map_frame_id);

    Eigen::Matrix3d Qnoise;

    Qnoise << 1e-7, 0, 0,
              0, 1e-7, 0,
              0, 0, 1e-7;

    Eigen::Matrix2d Rnoise;
    Rnoise << 1e-4, 0,
              0, 1e-4;

    ROS_INFO_STREAM("SLAM: Got number of landmarks: " << num_landmarks);
    ROS_INFO_STREAM("SLAM: Got map frame id: " << map_frame_id);

    ekf_slam::Slam robot(num_landmarks, Qnoise, Rnoise);

    rigid2d::DiffDrive ekf_bot(rigid2d::Pose2D(0,0,0), wheel_base, wheel_radius);
    rigid2d::WheelVelocities ekf_cmd;
    rigid2d::Twist2D ekf_tw;

    std::vector<double> slam_pose;
    rigid2d::Pose2D slam_pose2d(0, 0, 0);

    geometry_msgs::PoseStamped slam_point;
    std::vector<geometry_msgs::PoseStamped> slam_points;
    nav_msgs::Path slam_path;

    nuslam::TurtleMap est_landmarks;

    std::vector<double> radii(num_landmarks, 0.01);

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

        // track encoder position for ekf update
        ekf_enc_l += cur_js.position[lw_i];
        ekf_enc_r += cur_js.position[rw_i];

        // Update the odometer of the robot using the new wheel positions
        cmd = (bot.updateOdometry(cur_js.position[lw_i], cur_js.position[rw_i]));
        tw = bot.wheelsToTwist(cmd);

        // Get info to fill out the message and transform
        pos = bot.pose();
        q.setRPY(0, 0, pos.th);
        q_geo = tf2::toMsg(q);

        odom_point.header.frame_id = map_frame_id;
        odom_point.header.stamp = ros::Time::now();

        odom_point.pose.position.x = pos.x;
        odom_point.pose.position.y = pos.y;
        odom_point.pose.position.z = 0;
        odom_point.pose.orientation = q_geo;

        odom_points.push_back(odom_point);

        odom_path.header.frame_id = map_frame_id;
        odom_path.header.stamp = ros::Time::now();

        odom_path.poses = odom_points;

        odom_path_pub.publish(odom_path);

/////// SLAM CALCULATIONS //////////////////////////////////////////////////////
        if(got_slam_data == 1)
        {
          // Get twist from the last SLAM update til now
          rigid2d::WheelVelocities ekf_cmd = ekf_bot.updateOdometry(cur_js.position[lw_i], cur_js.position[rw_i]);
          rigid2d::Twist2D ekf_tw = ekf_bot.wheelsToTwist(ekf_cmd);

          // update SLAM state
          robot.MotionModelUpdate(ekf_tw);
          robot.MeasurmentModelUpdate(cur_landmarks);

          // Publish SLAM Path Message
          slam_pose = robot.getRobotState(); // returns robot state vector in (th, x, y) syntax

          slam_pose2d.x = slam_pose.at(1);
          slam_pose2d.y = slam_pose.at(2);
          slam_pose2d.th = slam_pose.at(0);

          slam_point.header.frame_id = map_frame_id;
          slam_point.header.stamp = ros::Time::now();

          slam_point.pose.position.x = slam_pose.at(1);
          slam_point.pose.position.y = slam_pose.at(2);
          slam_point.pose.position.z = 0;

          q.setRPY(0, 0, slam_pose.at(0));
          q_geo = tf2::toMsg(q);

          slam_point.pose.orientation = q_geo;

          slam_points.push_back(slam_point);

          slam_path.header.stamp = ros::Time::now();
          slam_path.header.frame_id = map_frame_id;

          slam_path.poses = slam_points;

          slam_path_pub.publish(slam_path);

          est_landmarks.header.stamp = ros::Time::now();
          est_landmarks.header.frame_id = "map";


          est_landmarks.centers = robot.getLandmarkStates();
          est_landmarks.radii = radii;

          slam_landmark_pub.publish(est_landmarks);

          got_slam_data = 0;
        }

        // Broadcast Map to Odom Frame

        rigid2d::Transform2D T_or(pos);
        rigid2d::Transform2D T_mr(slam_pose2d);

        rigid2d::Transform2D T_mo = T_mr * T_or.inv();

        tf2::Quaternion q;
        q.setRPY(0,0,T_mo.displacementRad().th);

        geometry_msgs::TransformStamped T_map_odom;

        T_map_odom.header.stamp = ros::Time::now();
        T_map_odom.header.frame_id = map_frame_id;

        T_map_odom.child_frame_id = odom_frame_id;

        T_map_odom.transform.translation.x = T_mo.displacement().x;
        T_map_odom.transform.translation.y = T_mo.displacement().y;
        T_map_odom.transform.translation.z = 0.0;

        T_map_odom.transform.rotation.x = q.x();
        T_map_odom.transform.rotation.y = q.y();
        T_map_odom.transform.rotation.z = q.z();
        T_map_odom.transform.rotation.w = q.w();

        Tmo_br.sendTransform(T_map_odom);

        got_odom_data = 0;
      }
   }
}
