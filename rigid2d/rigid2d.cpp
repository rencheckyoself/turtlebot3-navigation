
#include "rigid2d.hpp"
#include <iostream>


namespace rigid2d
{
  std::ostream & operator<<(std::ostream & os, const Vector2D & v)
  {
    os << "2D Vector, x portion= " << v.x << " and y portion= " << v.y << "\n";
    return os;
  }

  std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
  {
    os << "2D Transform, dtheta(degrees): " << rad2deg(tf.theta) << " x: " << tf.x << " y: " << tf.y << "\n";
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
    lhs.operator*=(rhs);
    return lhs;
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
    v_prime.y = stheta * v.x - stheta * v.y + y;

    return v_prime;
  }

  Transform2D Transform2D::inv() const
  {

    Transform2D inv_trans;

    inv_trans.theta = theta;
    inv_trans.ctheta = ctheta;
    inv_trans.stheta = -stheta;
    inv_trans.x = x * ctheta + y * stheta;
    inv_trans.y = -x * stheta + y * ctheta;

    return inv_trans;
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
