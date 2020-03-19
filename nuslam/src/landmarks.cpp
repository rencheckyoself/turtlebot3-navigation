/// \file
/// \brief This node clusters laser scan data points and fits each cluster to a circular landmark
///
/// PARAMETERS:
///   distance_threshold: (double) Threshold to determine if a point is in a cluster
///   radius_threshold: (double) Threshold to determine if a cirlce fit is valid for an obstacle
///   frame_id: (string) The frame the Laser scan data is relative to
/// PUBLISHES:
///     /landmark_data: (nuslam/TurtleMap) a list of centers and radii for cylindrical landmarks
/// SUBSCRIBES:
///     /scan: (sensor_msgs/LaserScan) the raw laser data from the turtlebot
/// SERIVCES:

#include <vector>
#include <Eigen/Dense>

#include "ros/ros.h"

#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "nuslam/TurtleMap.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"

#include "rigid2d/rigid2d.hpp"
#include "nuslam/cylinder_detect.hpp"

static double distance_threshold = 0;
static double radius_threshold = 0;
static std::string frame_id = "Did not Fill";
static ros::Publisher pub_cmd, pub_pc;


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
  ros::NodeHandle temp_n("~");
  int plot_cluster = 0;

  temp_n.getParam("plot_cluster", plot_cluster);

  std::vector<rigid2d::Vector2D> temp_points; // Temporary cluster of points
  std::vector<std::vector<rigid2d::Vector2D>> buf_points_list; // list of all point clusters
  std::vector<std::vector<rigid2d::Vector2D>> points_list; // list of all point clusters
  std::vector<geometry_msgs::Point32> pc_points;
  geometry_msgs::Point32 pc_point;

  sensor_msgs::PointCloud pointcloud;

  double cur_range = 0;
  double cur_theta = 0;

  double max_range = data->range_max;
  double min_range = data->range_min;

  // Loop through the data points in the returned array to find clusters

  // ROS_INFO_STREAM("Total scan points: " << data->ranges.size());

  for(unsigned int i = 0; i < data->ranges.size(); i++)
  {
    cur_range = data->ranges.at(i);
    cur_theta = data->angle_min + (data->angle_increment * i);


    // check if the point is valid valid point
    if(cur_range < max_range && cur_range > min_range)
    {

      // if the current cluster is empty, add point to the list
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
  else // Save final cluster
  {
    buf_points_list.push_back(temp_points);
  }

  // Plot one cluster of data
  for(unsigned int buf_ctr = 0; buf_ctr < buf_points_list.at(plot_cluster).size()-1; buf_ctr++)
  {
    pc_point.x = buf_points_list.at(plot_cluster).at(buf_ctr).x;
    pc_point.y = buf_points_list.at(plot_cluster).at(buf_ctr).y;
    pc_point.z = 0.1;
    pc_points.push_back(pc_point);
  }

  pointcloud.points = pc_points;
  pointcloud.header.frame_id = frame_id;
  pointcloud.header.stamp = ros::Time::now();
  pub_pc.publish(pointcloud);
  pc_points.clear();

  int total_clusters = buf_points_list.size();

  // Prune clusters with less than 3 points
  for(int j = 0; j < total_clusters; j++)
  {
    temp_points.clear();
    temp_points = buf_points_list.back();
    buf_points_list.pop_back();

    if(temp_points.size() > 3)
    {
      points_list.push_back(temp_points);
    }
  }

  // Circle Fitting Algorithm
  // For more details see: A. Al-Sharadqah and N. Chernov, Error Analysis for
  // Circle Fitting Algorithms, Electronic Journal of Statistics (2009), Volume 3 p 886-911
  // https://projecteuclid.org/download/pdfview_1/euclid.ejs/1251119958
  temp_points.clear();

  std::vector<double> circle_param;
  geometry_msgs::Point center_point;
  std::vector<geometry_msgs::Point> center_points;
  std::vector<double> radii;
  nuslam::TurtleMap cluster_data;

  for(auto cluster : points_list)
  {
    // Fit circle to a cluster
    circle_param = cylinder::fit_circles(cluster);

    // eliminate if radius is unreasonably large
    if(circle_param.at(2) < radius_threshold)
    {
      center_point.x = circle_param.at(0);
      center_point.y = circle_param.at(1);

      center_points.push_back(center_point);
      radii.push_back(circle_param.at(2));

      // ROS_INFO_STREAM("STEP 10 Complete");
    }
    else
    {
      // ROS_INFO_STREAM("Cluster eliminated due to a large radius size.");
    }
  }

  // ROS_INFO_STREAM("Init Clusters: " << points_list.size() << " Fin Clusters: " << center_points.size());

  // Assemble variable to publish
  cluster_data.header.frame_id = frame_id;
  cluster_data.centers = center_points;
  cluster_data.radii = radii;

  pub_cmd.publish(cluster_data);
}



/// \brief main function to create the real_waypoints node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "landmarks");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  ros::Subscriber sub_scan = n.subscribe("scan", 1, callback_robotScan);
  pub_cmd = n.advertise<nuslam::TurtleMap>("landmark_data", 1);
  pub_pc = n.advertise<sensor_msgs::PointCloud>("pointcloud_data", 12);

  pn.getParam("distance_threshold", distance_threshold);
  pn.getParam("radius_threshold", radius_threshold);
  pn.getParam("frame_id", frame_id);

  ROS_INFO_STREAM("LANDMARKS: Distance Threshold " << distance_threshold);
  ROS_INFO_STREAM("LANDMARKS: Radius Threshold " << radius_threshold);
  ROS_INFO_STREAM("LANDMARKS: Frame ID " << frame_id);

  ros::spin();
}
