/// \file
/// \brief This node takes a list of centers and radii to draw the coresponding map
///
/// PARAMETERS:
/// PUBLISHES:
///     /markers....
/// SUBSCRIBES:
///     /landmark_data: (nuslam/TurtleMap) a list of centers and radii for cylindrical landmarks
/// SERIVCES:

#include <ros/ros.h>

#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "nuslam/TurtleMap.h"
#include "rigid2d/rigid2d.hpp"

static std::vector<geometry_msgs::Point> centroids;
static std::vector<double> radii;
static std::string frame_id = "base_scan";

/// \brief callback fuction for to save the landmark data
void callback_landmark_data(nuslam::TurtleMap::ConstPtr data)
{
  centroids = data->centers;
  radii = data->radii;
  // frame_id = data->header.frame_id;
}
/// \brief main function to create the real_waypoints node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_map");
  ros::NodeHandle n;

  ros::NodeHandle pn("~");
  ros::Subscriber sub_landmarks = n.subscribe<nuslam::TurtleMap>("landmark_data", 1, callback_landmark_data);
  ros::Publisher pub_markers = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  // Publish the markers

  std::string ns = "landmarks";
  float r = 1.0, g = 1.0, b = 1.0;

  pn.getParam("frame_id", frame_id);
  pn.getParam("ns", ns);
  pn.getParam("r", r);
  pn.getParam("g", g);
  pn.getParam("b", b);

  ROS_INFO_STREAM("MAP: frame_id: " << frame_id);
  ROS_INFO_STREAM("MAP: namespace: " << ns);
  ROS_INFO_STREAM("MAP: r: " << r);
  ROS_INFO_STREAM("MAP: g: " << g);
  ROS_INFO_STREAM("MAP: b: " << b);


  visualization_msgs::MarkerArray pub_marks;
  std::vector<visualization_msgs::Marker> markers;

  visualization_msgs::Marker marker;

  while(ros::ok())
  {
    for(unsigned int i = 0; i < centroids.size(); i++)
    {
      marker.header.frame_id = frame_id;
      marker.header.stamp = ros::Time::now();

      marker.ns = ns;
      marker.id = i;

      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = centroids.at(i).x;
      marker.pose.position.y = centroids.at(i).y;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 1;

      marker.scale.x = radii.at(i) * 2.0;
      marker.scale.y = radii.at(i) * 2.0;
      marker.scale.z = .1;

      marker.color.r = r;
      marker.color.b = b;
      marker.color.g = g;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration(1); //5Hz to match the sensor publishing freq

      markers.push_back(marker);
      // r.sleep();
    }

    pub_marks.markers = markers;
    pub_markers.publish(pub_marks);

    markers.clear();

    ros::spinOnce();
  }
}
