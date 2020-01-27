#ifndef WAYPOINTS_INCLUDE_GUARD_HPP
#define WAYPOINTS_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for calculating the velocities to move between waypoints

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <vector>

namespace rigid2d
{
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
    void setVlims(double lin, double ang);

    /// \brief set the internal rate
    void setRate(int r);

    /// \brief get the current target
    /// \return the current target
    Vector2D getTarget();

    /// \brief convert a Twist2D into a geometry_msgs::Twist
    /// \param tw - a Twist2D to convert
    /// \return the equivalent geometry_msgs::Twist
    geometry_msgs::Twist Twist2DtoGeoTwist(Twist2D tw);

  private:

    double rate; //rate to send commands
    double linv_lim; // linear velocity limits
    double angv_lim; // angular velocity limits
    std::vector<Vector2D> point_list; // list of waypoint to travel between
    Vector2D target; // The current target waypoint to drive to

    /// \brief Retrieve the next waypoint to move to
    void setTarget();
  };

}
#endif
