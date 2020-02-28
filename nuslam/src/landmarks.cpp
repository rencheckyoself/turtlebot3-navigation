/// \file
/// \brief This node clusters laser scan data points and fits each cluster to a circular landmark
///
/// PARAMETERS:
/// PUBLISHES:
///     /landmark_data: (nuslam/TurtleMap) a list of centers and radii for cylindrical landmarks
/// SUBSCRIBES:
///     /scan: (sensor_msgs/LaserScan) the raw laser data from the turtlebot
/// SERIVCES:

#include <vector>

#include "ros/ros.h"

#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"
#include "nuslam/TurtleMap.h"
#include "sensor_msgs/LaserScan.h"

#include "rigid2d/rigid2d.hpp"

static double distance_threshold = 0;

// Struct to describe a cluster
// typedef struct clusters
// {
//   std::vector<rigid2d::Vector2D> points;
//   int id = 0;
//   geometry_msgs::Point center;
//   double radius = 0;
//
//   clusters(){};
//
//   clusters(int num)
//   {
//     id = num;
//   }
//
// } cluster;


/// \brief converts a point in polar coordinates into cartesian coordinates
/// \param r: the distange to the point
/// \param theta: the angle to the point
/// \return (rigid2d::Vector2D) x,y coordinates of the point
rigid2d::Vector2D polar2cart(double r, double theta)
{
  return rigid2d::Vector2D(r*std::cos(theta), r*std::sin(theta));
}


/// \brief Callback function for the sensor subscriber
void callback_robotScan(sensor_msgs::LaserScan::ConstPtr data)
{
  std::vector<rigid2d::Vector2D> temp_points; // Temporary cluster of points
  std::vector<std::vector<rigid2d::Vector2D>> buf_points_list; // list of all point clusters
  std::vector<std::vector<rigid2d::Vector2D>> points_list; // list of all point clusters

  double cur_range = 0;
  double cur_theta = 0;

  double max_range = data->range_max;
  double min_range = data->range_min;


  // Loop through the rest of the data points in the returned array
  for(unsigned int i = 0; i < data->ranges.size(); i++)
  {
    cur_range = data->ranges.at(i);
    cur_theta = data->angle_min + (data->angle_increment * i);

    // check if the point is valid valid point
    if(cur_range < max_range && cur_range > min_range)
    {

      // if the current cluster is still empty, add point to the list
      if(temp_points.empty())
      {
        temp_points.push_back(polar2cart(cur_range, cur_theta));
      }

      // if the current point is within the distance threshold, add to the
      // current point list
      else if(std::fabs(cur_range - data->ranges.at(i-1)) <= distance_threshold)
      {
        temp_points.push_back(polar2cart(cur_range, cur_theta));
      }

      // if the current point is outside of the distance threshold, add current
      // point a new cluster
      else if(std::fabs(cur_range - data->ranges.at(i-1)) > distance_threshold)
      {
        buf_points_list.push_back(temp_points);
        temp_points.clear();
        temp_points.push_back(polar2cart(cur_range,cur_theta));
      }
      // Error Catching
      else
      {
        ROS_ERROR_STREAM("Valid Point not assigned. Point ID " << i << " Point range " << cur_range << " Point bearing " << cur_theta);
      }
    }
  }


  // If last cluster merges with the first cluster, then combine the two point lists
  if(std::fabs(data->ranges.at(0) - data->ranges.at(data->ranges.size()-1)) <= distance_threshold)
  {
    buf_points_list.at(0).insert(buf_points_list.at(0).end(), temp_points.begin(), temp_points.end());
  }

  // Save final cluster
  else
  {
    buf_points_list.push_back(temp_points);
  }

  // Prune clusters with less than 3 points
  for(unsigned int j = 0; j < buf_points_list.size(); j++)
  {
    temp_points.clear();
    temp_points = buf_points_list.back();
    buf_points_list.pop_back();

    if(temp_points.size() >= 3)
    {
      points_list.push_back(temp_points);
    }
  }

  // Circle Algorithm


}

/// \brief main function to create the real_waypoints node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "landmarks");
  ros::NodeHandle n;

  ros::Subscriber sub_scan = n.subscribe("scan", 1, callback_robotScan);
  ros::Publisher pub_cmd = n.advertise<nuslam::TurtleMap>("landmark_data", 1);

  n.getParam("distance_threshold", distance_threshold);

  ROS_INFO_STREAM("LANDMARKS: Distance Threshold " << distance_threshold);

  ros::spin();
}
