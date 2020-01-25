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
    Waypoints(std::vector<Vector2D> points);

    /// \brief Method to calculate the next velocity command given the current target waypoint
    /// \param pos - the current pose of the robot
    /// \return The velocity command to reach the next waypoint
    Twist2D nextWaypoint(Pose2D pos);

    /// \brief Retrieve the next waypoint to move to
    void getTarget();

    /// \brief set the internal velocity limits
    void setVlims(double lin, double ang);

    /// \brief set the internal rate
    void setRate(int r);

  private:

    double rate; //rate to send commands
    double linv_lim; // linear velocity limits
    double angv_lim; // angular velocity limits
    std::vector<Vector2D> point_list; // list of waypoint to travel between
    Vector2D target; // The current target waypoint to drive to
  };

}
#endif
