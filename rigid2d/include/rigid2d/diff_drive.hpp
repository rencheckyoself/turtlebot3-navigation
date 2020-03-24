#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for tracking the state of a diff drive robot.

#include "rigid2d/rigid2d.hpp"

namespace rigid2d
{

  /// \brief Wheel velocities for a diff drive robot.
  struct WheelVelocities
  {
      double ur = 0; ///< right wheel vel
      double ul = 0; ///< left wheel vel

      /// \brief sets wheel velocities to zero
      WheelVelocities();

      /// \brief sets wheel velocity parameters to specified values
      WheelVelocities(double left, double right);
  };

  class DiffDrive
  {
  public:
      /// \brief the default constructor creates a robot at (0,0,0), with a default wheel base and wheel radius
      ///
      DiffDrive();

      /// \brief create a DiffDrive model by specifying the pose, and geometry
      ///
      /// \param pose - the current position of the robot
      /// \param wheel_base - the distance between the wheel centers
      /// \param wheel_radius - the raidus of the wheels
      DiffDrive(Pose2D pose, double wheel_base, double wheel_radius);

      /// \brief determine the wheel velocities required to make the robot
      ///        move with the desired linear and angular velocities. See doc directory for derivation.
      /// \param twist - the desired twist in the body frame of the robot
      /// \returns - the wheel velocities to use
      /// \throws std::exception when yb != 0
      WheelVelocities twistToWheels(Twist2D twist);

      /// \brief determine the body twist of the robot from its wheel velocities. See doc directory for derivation.
      /// \param vel - the velocities of the wheels, assumed to be held constant
      ///  for one time unit
      /// \returns twist in the original body frame of the robot
      Twist2D wheelsToTwist(WheelVelocities vel) const;

      /// \brief Update the robot's odometry based on the current encoder readings
      /// \param left - the left encoder angle (in radians)
      /// \param right - the right encoder angle (in radians)
      /// \return the velocities of each wheel, assuming that they have been
      /// constant since the last call to updateOdometry
      WheelVelocities updateOdometry(double left, double right);

      /// \brief update the odometry of the diff drive robot, assuming that
      /// it follows the given body twist for one time  unit
      /// \param cmd - the twist command to send to the robot
      void feedforward(Twist2D cmd);

      /// \brief Sets the wheel radius
      /// \param radius - value to update the class parameter to
      void setRadius(double radius);

      /// \brief Sets the wheel base
      /// \param b - value to update the class parameter to
      void setBase(double b);

      /// \brief get the current pose of the robot
      /// \returns the current pose of the robot
      Pose2D pose() const;

      /// \brief get the current absolute encoder position
      /// \return the encoder position
      WheelVelocities getEncoders() const;

      /// \brief get the wheel speeds, based on the last encoder update
      /// \returns the velocity of the wheels, which is equivalent to
      ///          displacement because Delta_T = 1
      WheelVelocities wheelVelocities() const;

      /// \brief reset the robot to the given position/orientation
      /// \param ps - the desired position/orientation to place the robot
      void reset(Pose2D ps);

  private:
      Pose2D pos; // position of the robot
      double r; // wheel radius of the robot
      double base; // distance between the wheel centers
      WheelVelocities w_vels; // velocities of the two wheels
      WheelVelocities prev_enc; // Previous encoder values
      Transform2D T_wb; // Transforms to the base and wheels

      /// \brief sets the transform to the body frame of the robot using the existing pos values.
      void setTransform();

  };
}
#endif
