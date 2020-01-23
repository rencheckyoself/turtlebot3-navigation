/// \file
/// \brief Source file for Diff Dirve Robot library
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <iostream>

namespace rigid2d
{

  DiffDrive::DiffDrive()
  {
    pos.x = 0;
    pos.y = 0;
    pos.th = 0;
    r = 0.1;
    base = .3;

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
  }

  WheelVelocities DiffDrive::twistToWheels(Twist2D twist)
  {
    // Convert the given body twist into a twist at each wheel frame
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
