/// \file
/// \brief Source file for rigid2D library
#include "rigid2d/rigid2d.hpp"
#include <iostream>

namespace rigid2d
{
  std::ostream & operator<<(std::ostream & os, const Vector2D & v)
  {
    os << "2D Vector, [" << v.x << ", " << v.y << "]\n";
    return os;
  }

  std::ostream & operator<<(std::ostream & os, const Twist2D & tw)
  {
    os << "2D Twist, [" << tw.wz << ", " << tw.vx << ", " << tw.vy << "]\n";
    return os;
  }

  std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
  {
    os << "2D Transform, theta(degrees): " << rad2deg(tf.theta) << " x: " << tf.x << " y: " << tf.y << "\n";
    return os;
  }

  std::istream & operator>>(std::istream & is, Vector2D & v)
  {
    std::cout << "Enter x value of vector: ";
    is >> v.x;
    std::cout << "Enter y value of vector: ";
    is >> v.y;
    return is;
  }

  std::istream & operator>>(std::istream & is, Twist2D & tw)
  {
    std::cout << "Enter wz value of twist: ";
    is >> tw.wz;
    std::cout << "Enter vx value of twist: ";
    is >> tw.vx;
    std::cout << "Enter vy value of twist: ";
    is >> tw.vy;
    return is;
  }

  std::istream & operator>>(std::istream & is, Transform2D & tf)
  {
    Vector2D vec;
    double ang;

    std::cout << "Enter an angle (degrees): ";
    is >> ang;

    std::cout << "Enter x value: ";
    is >> vec.x;

    std::cout << "Enter y value: ";
    is >> vec.y;

    Transform2D temp(vec, deg2rad(ang));

    tf = temp;

    return is;
  }

  Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
  {
    lhs *= rhs ;
    return lhs;
  }

  Vector2D::Vector2D()
  {
    x = 0.0;
    y = 0.0;
  }

  Vector2D::Vector2D(double xcomp, double ycomp)
  {
    x = xcomp;
    y = ycomp;
  }

  Vector2D Vector2D::normalize() const
  {
    float mag;
    Vector2D unit_vec;

    mag = pow(x*x + y*y, 0.5);

    unit_vec.x = x/mag;
    unit_vec.y = y/mag;

    return unit_vec;
  }

  Twist2D::Twist2D()
  {
    wz = 0.0;
    vx = 0.0;
    vy = 0.0;
  }

  Twist2D::Twist2D(double ang, double linx, double liny)
  {
    wz = ang;
    vx = linx;
    vy = liny;
  }

  // Public
  Transform2D::Transform2D()
  {
    x = 0;
    y = 0;
    theta = 0;
    ctheta = 1;
    stheta = 0;
  }

  Transform2D::Transform2D(const Vector2D & trans)
  {
    ctheta = 1;
    stheta = 0;
    theta = 0;
    x = trans.x;
    y = trans.y;
  }

  Transform2D::Transform2D(double radians)
  {
    theta = radians;
    ctheta = std::cos(radians);
    stheta = std::sin(radians);
    x = 0;
    y = 0;
  }

  Transform2D::Transform2D(const Vector2D & trans, double radians)
  {
    theta = radians;
    ctheta = std::cos(radians);
    stheta = std::sin(radians);
    x = trans.x;
    y = trans.y;
  }

  Vector2D Transform2D::operator()(Vector2D v) const
  {
    Vector2D v_prime;

    v_prime.x = ctheta * v.x - stheta * v.y + x;
    v_prime.y = stheta * v.x + ctheta * v.y + y;

    return v_prime;
  }

  Twist2D Transform2D::operator()(Twist2D tw) const
  {
    Twist2D tw_prime;

    tw_prime.wz = tw.wz;
    tw_prime.vx = tw.vx * ctheta - tw.vy * stheta + tw.wz * y;
    tw_prime.vy = tw.vx * stheta + tw.vy * ctheta - tw.wz * x;

    return tw_prime;
  }

  Transform2D Transform2D::inv() const
  {
    Transform2D inv_trans(theta, ctheta, -stheta, -x * ctheta - y * stheta, x * stheta - y * ctheta);
    return inv_trans;
  }

  Twist2D Transform2D::displacement() const
  {
    Twist2D disp(rad2deg(theta), x, y);
    return disp;
  }

  Transform2D & Transform2D::operator*=(const Transform2D & rhs)
  {
    double x_buf, y_buf, cth_buf, sth_buf;

    cth_buf = ctheta*rhs.ctheta - stheta*rhs.stheta;
    sth_buf = stheta*rhs.ctheta + ctheta*rhs.stheta;

    x_buf = ctheta*rhs.x - stheta*rhs.y + x;
    y_buf = stheta*rhs.x - ctheta*rhs.y + y;

    theta = std::acos(cth_buf);
    x = x_buf;
    y = y_buf;
    ctheta = cth_buf;
    stheta = sth_buf;

    return *this;
  }

  // Private
  Transform2D::Transform2D(double theta, double ctheta, double stheta, double x, double y)
  {
    this->theta = theta;
    this->ctheta = ctheta;
    this->stheta = stheta;
    this->x = x;
    this->y = y;
  }
}