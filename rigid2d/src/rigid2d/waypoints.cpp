/// \file
/// \brief Library for calculating the velocities to move between waypoints

#include <iostream>
#include <cmath>

#include "geometry_msgs/Twist.h"

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"

namespace rigid2d
{
  geometry_msgs::Twist Twist2DtoGeoTwist(Twist2D tw)
  {
    geometry_msgs::Twist gtw;
    gtw.linear.x = tw.vx;
    gtw.linear.y = tw.vy;
    gtw.angular.z = tw.wz;

    return gtw;
  }

  Twist2D GeoTwisttoTwist2D(geometry_msgs::Twist gtw)
  {
    Twist2D tw;
    tw.vx = gtw.linear.x;
    tw.vy = gtw.linear.y;
    tw.wz = gtw.angular.z;

    return tw;
  }

  Waypoints::Waypoints()
  {
    rate = 60;
    linv_lim = 0.5;
    angv_lim = 0.5;

    dist_thresh = 0.01;
    ang_thresh = 0.01;
    kp = 0.5;
    num_cycles = -1;

    point_list.at(0) = Vector2D (1,1);
    point_list.at(1) = Vector2D (5,1);
    point_list.at(2) = Vector2D (5,5);
    point_list.at(3) = Vector2D (3,6);
    point_list.at(4) = Vector2D (1,5);

    init_target = point_list.at(1);
    setTarget();
  }

  Waypoints::Waypoints(std::vector<Vector2D> points, int r, double vlim, double alim)
  {
    rate = r;
    linv_lim = vlim;
    angv_lim = alim;

    dist_thresh = 0.01;
    ang_thresh = 0.01;
    kp = 0.5;
    num_cycles = -1;

    point_list = points;

    init_target = point_list.at(1);
    setTarget();
  }

  void Waypoints::setTarget()
  {
    target = point_list.at(0);
    point_list.erase(point_list.begin());
    point_list.push_back(target);


    if(target.x == init_target.x && target.y == init_target.y)
    {
      num_cycles++;
    }

    std::cout << "New target set to: " << target << "\n";
    std::cout << "Num Cycles: " << num_cycles << "\n"; 
  }

  Vector2D Waypoints::getTarget()
  {
    return target;
  }

  Twist2D Waypoints::nextWaypoint(Pose2D pos)
  {
    Twist2D tw;
    double x_dist, y_dist;
    double calc_dist, calc_heading;
    double err_heading;

    tw.vy = 0;

    x_dist = target.x - pos.x;
    y_dist = target.y - pos.y;

    // calculate the linear distance between the robot and target
    calc_dist = std::hypot(x_dist, y_dist);

    // calculate the angular distance between the robot and target
    calc_heading = std::atan2(y_dist, x_dist);
    err_heading = calc_heading - pos.th;

    if(err_heading > rigid2d::PI)
    {
      err_heading = (err_heading - 2*rigid2d::PI);
    }

    else if(err_heading < -rigid2d::PI)
    {
      err_heading = (err_heading + 2*rigid2d::PI);
    }

    // update target if nessissary
    if(calc_dist < dist_thresh)
    {
      setTarget();

      tw.wz = 0;
      tw.vx = 0;
    }

    else if(std::fabs(err_heading) < ang_thresh)
    {
      // move straight
      tw.wz = 0;
      tw.vx = std::clamp(kp * calc_dist, -linv_lim, linv_lim);
    }

    else
    {
      // turn
      tw.wz = std::clamp(kp * err_heading, -angv_lim, angv_lim);
      tw.vx = 0;

    }

    // return the twist
    return tw;
  }

  void Waypoints::setVlims(double lin, double ang)
  {
    linv_lim = lin;
    angv_lim = ang;
  }

  void Waypoints::setThresholds(double alim, double tlim)
  {
    dist_thresh = tlim;
    ang_thresh = alim;
  }

  void Waypoints::setGains(double p)
  {
    kp = p;
  }

  void Waypoints::setRate(int r)
  {
    rate = r;
  }

  int Waypoints::getCycles()
  {
    return num_cycles;
  }
}
