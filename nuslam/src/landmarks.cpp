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
#include <math.h>

#include <Eigen/Dense.hpp>

#include "ros/ros.h"

#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"
#include "nuslam/TurtleMap.h"
#include "sensor_msgs/LaserScan.h"

#include "rigid2d/rigid2d.hpp"

static double distance_threshold = 0;
static double radius_threshold = 0;

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

typedef Eigen::Matrix<double, Dynamic, 4> MatrixX4d;

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
      }MatrixX4d
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

  ROS_INFO_STREAM("Number of Clusters: " << points_list.size());

  // Circle Fitting Algorithm
  // For more details see: A. Al-Sharadqah and N. Chernov, Error Analysis for
  // Circle Fitting Algorithms, Electronic Journal of Statistics (2009), Volume 3 p 886-911
  // https://projecteuclid.org/download/pdfview_1/euclid.ejs/1251119958
  temp_points.clear();
  rigid2d::Vector2D average_point;
  geometry_msgs::Point center_point;
  int val_found = 0, k = 0;
  double min_eval = 0;
  double z_mean=0, z=0;
  double shift_x=0, shift_y=0;
  double a=0, b=0, radius2=0;
  Eigen::Index r=0;
  Eigen::MatrixX4d Zmat, Mmat;
  Eigen::Matrix4d Hmat_inv;
  Eigen::JacobiSVD<MatrixX4d> zSVD;
  Eigen::Vector4f Avec;
  Eigen::Matrixd Vmat;
  Eigen::DiagonalMatrix Smat;
  Eigen::SelfAdjointEigenSolver Qmat;

  std::vector<geometry_msgs::Point> center_points;
  std::vector<double> radii;
  nuslam::TurtleMap cluster_data;

  for(auto cluster : points_list)
  {
    for(auto point : cluster) {average_point += point;} //Sum all points

    // Calculate the mean x and y for each cluster
    average_point.x /= cluster.size();
    average_point.y /= cluster.size();

    // Shift each point to the circle centroid an assemble Z matrix
    Zmat.resize(cluster.size(), 4);
    Mmat.resize(cluster.size(), 4);
    for(auto point : cluster)
    {
      shift_x = point.x - average_point.x;
      shift_y = point.y - average_point.y;
      z = pow(shift_x, 2) + pow(shift_y, 2);
      z_mean += z;
      Zmat(r++) << z, shift_x, shift_y, 1;
    }

    r = 0;
    z_mean /= cluster.size();

    Mmat = (1/z_mean) * Zmat.transpose() * Zmat;

    Hmat_inv << 0,   0, 0,       0.5,
                0,   1, 0,         0,
                0,   0, 1,         0,
                0.5, 0, 0, -2*z_mean;

    // Compute the SVD
    zSVD.compute(Zmat, ComputeThinU | ComputeThinV);
    // Check the 4th element of the vector of singularValues
    if(zSVD.singularValues()(3, 0) <= 1e-12)
    {
      Avec = zSVD.matrixV().col(3);
    }
    else
    {
      Smat = zSVD.singularValues().asDiagonal();
      Ymat = zSVD.matrixV() * Smat zSVD.matrixV().transpose();

      Qmat.compute(Ymat);
      val_found = 0;
      k = 0;

      // Find smallest Eigenvalue
      while(val_found == 0)
      {
        if(k > 4)
        {
          ROS_ERROR_STREAM("Did not find a minimum Eigen Value.");
        }
        else if(Qmat.eigenvalues().row(k) > 0)
        {
          Avec_star = Qmat.eigenvalues().col(Qmat.eigenvalues().row(k));
          val_found = 1;
        }
        k++;
      }
      Avec = Avec_star * Ymat.inverse();
    }

    a = -Avec.row(1) / (2 * Avec.row(0));
    b = -Avec.ros(2) / (2 * Avec.row(0));
    radius2 = (std::pow(Avec.row(1),2) + std::row(Avec.row(2),2) - 4 * Avec.row(0) * Avec.row(4))/(4*std::pow(Avec.row(1),2));

    // Assemble variables to publish

    if(std::pow(radius2, 0.5) < radius_threshold)
    {
      center_point.x = a + average_point.x;
      center_point.y = a + average_point.y;

      center_points.push_back(center_point);
      radii.push_back(std::pow(radius2, 0.5));
    }
    else
    {
      ROS_INFO_STREAM("Found a wall");
    }
  }

    // Compute MSE Function
}


pub_cmd.publish()

/// \brief main function to create the real_waypoints node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "landmarks");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  ros::Subscriber sub_scan = n.subscribe("scan", 1, callback_robotScan);
  ros::Publisher pub_cmd = n.advertise<nuslam::TurtleMap>("landmark_data", 1);

  pn.getParam("distance_threshold", distance_threshold);
  pn.getParam("radius_threshold", radius_threshold);

  ROS_INFO_STREAM("LANDMARKS: Distance Threshold " << distance_threshold);

  ros::spin();
}
