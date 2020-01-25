/// \file
/// \brief Source file for Diff Dirve Robot library
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <iostream>

namespace rigid2d
{

  WheelVelocities::WheelVelocities()
  {
    ul = 0;
    ur = 0;
  }

  WheelVelocities::WheelVelocities(double left, double right)
  {
    ul = left;
    ur = right;
  }

  DiffDrive::DiffDrive()
  {
    pos.x = 0;
    pos.y = 0;
    pos.th = 0;
    r = 0.1;
    base = 0.5;

    Pose2D bufl(0, -base/2, 0);
    Pose2D bufr(0, base/2, 0);

    Transform2D buf_base(pos);
    Transform2D buf_left(bufl);
    Transform2D buf_right(bufr);

    T_wb = buf_base;
    T_bl = buf_left;
    T_br = buf_right;
  }

  DiffDrive::DiffDrive(Pose2D pose, double wheel_base, double wheel_radius)
  {
    pos = pose;
    base = wheel_base;
    r = wheel_radius;

    Pose2D bufl(0, -base/2, 0);
    Pose2D bufr(0, base/2, 0);

    Transform2D buf_base(pos);
    Transform2D buf_left(bufl);
    Transform2D buf_right(bufr);

    T_wb = buf_base;
    T_bl = buf_left;
    T_br = buf_right;
  }

  WheelVelocities DiffDrive::twistToWheels(Twist2D twist)
  {


    if(twist.vy == 0)
    {
      // These Equations were derived by:
      // 1) Converting the body twist into the frame of each wheel (eq 1 & 2)
      // 2) Identify the velocity characteristics of a conventional wheel (eq 3)
      // 3) Solve for the wheel velocities using eq 1-3 (eq 4)

      double d = base/2;
      w_vels.ur = (1/r) * (d*twist.wz + twist.vx);
      w_vels.ul = (1/r) * (-d*twist.wz + twist.vx);
    }

    else
    {
      throw std::invalid_argument("The body twist cannot have a vy component that is non zero");
    }

    return w_vels;
  }

  Twist2D DiffDrive::wheelsToTwist(WheelVelocities vel) const
  {

    // These Equations were derived by:
    // Using eq 4, take the pseudoinverse of the H matrix (eq 5) and then solve

    double d = 1 /(base);
    Twist2D tw((r)*(d*vel.ur - d*vel.ul), (r)*(.5*vel.ur + .5*vel.ul), 0);
    return tw;
  }

  void DiffDrive::updateOdometry(double left, double right)
  {
    WheelVelocities move;
    move.ul = left - prev_enc.ul;
    move.ur = right - prev_enc.ur;

    feedforward(wheelsToTwist(move));
  }

  void DiffDrive::feedforward(Twist2D cmd)
  {
    T_wb = T_wb.integrateTwist(cmd);
    pos = T_wb.displacement();
  }

  Pose2D DiffDrive::pose() const
  {
    return pos;
  }

  WheelVelocities DiffDrive::wheelVelocities() const
  {
    return w_vels;
  }

  void DiffDrive::reset(Pose2D ps)
  {
    pos = ps;
  }

}
