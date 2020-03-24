#ifndef WAYPOINTS_INCLUDE_GUARD_HPP
#define WAYPOINTS_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for calculating the velocities to move between waypoints

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <vector>
#include "geometry_msgs/Twist.h"

namespace rigid2d
{
  /// \brief convert a Twist2D into a geometry_msgs::Twist
  /// \param tw - a Twist2D to convert
  /// \return the equivalent geometry_msgs::Twist
  geometry_msgs::Twist Twist2DtoGeoTwist(Twist2D tw);

  /// \brief convert a geometry_msgs::Twist into a Twist2D
  /// \param gtw - a geometry_msgs::Twist to convert
  /// \return the equivalent Twist2D
  Twist2D GeoTwisttoTwist2D(geometry_msgs::Twist gtw);

  class Waypoints
  {
  public:

    /// \brief Use defualt values for parameters
    Waypoints();

    /// \brief Use a custom waypoint string
    Waypoints(std::vector<Vector2D> points, int r, double vlim, double alim);

    /// \brief Method to calculate the next velocity command given the current target waypoint
    /// \param pos - the current pose of the robot
    /// \return The velocity command to reach the next waypoint
    Twist2D nextWaypoint(Pose2D pos);

    /// \brief set the internal velocity limits
    /// \param lin - the linear velocity limit
    /// \param ang - the angular velocity limit
    void setVlims(double lin, double ang);

    /// \brief set the internal rate
    /// \param r - the desired rate
    void setRate(int r);

    /// \brief set the angular and distance thresholds
    /// \param alim - the angular position threshold
    /// \param tlim - the translation position threshold
    void setThresholds(double alim, double tlim);

    /// \brief set the controls gains
    /// \param p - the proportional control gain
    void setGains(double p);

    /// \brief get the current target
    /// \return the current target
    Vector2D getTarget();

    /// \brief get the number of completed cycles
    /// \return the number of completed cycles
    int getCycles();

  private:

    double rate; //rate to send commands
    double linv_lim; // linear velocity limits
    double angv_lim; // angular velocity limits
    std::vector<Vector2D> point_list; // list of waypoint to travel between
    Vector2D target; // The current target waypoint to drive to
    Vector2D init_target; // The first waypoint of the path
    double kp; //proportional control gain
    double dist_thresh; //angular threshold setting
    double ang_thresh; //angular threshold setting
    int num_cycles; //the number of cycles through the waypoints

    /// \brief Retrieve the next waypoint to move to
    void setTarget();
  };

}
#endif
