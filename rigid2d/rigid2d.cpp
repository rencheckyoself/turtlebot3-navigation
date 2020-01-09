
#include "rigid2d.hpp"
#include <iostream>

std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Vector2D & v)
{
  os << "2D Vector, x portion= " << v.x << " and y portion: " << v.y << "\n";
  return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Vector2D & v)
{
  std::cout << "Enter x value of vector: ";
  is >> v.x;
  std::cout << "\nEnter y value of vector: ";
  is >> v.y;
  return is;
}

/*
/// \brief create a transformation that is a pure translation
/// \param trans - the vector by which to translate
explicit Transform2D(const Vector2D & trans);

/// \brief create a pure rotation
/// \param radians - angle of the rotation, in radians
explicit Transform2D(double radians);

/// \brief Create a transformation with a translational and rotational
/// component
/// \param trans - the translation
/// \param rot - the rotation, in radians
Transform2D(const Vector2D & trans, double radians);

/// \brief apply a transformation to a Vector2D
/// \param v - the vector to transform
/// \return a vector in the new coordinate system
Vector2D operator()(Vector2D v) const;

/// \brief invert the transformation
/// \return the inverse transformation.
Transform2D Transform2D::inv() const
{

}

/// \brief compose this transform with another and store the result
/// in this object
/// \param rhs - the first transform to apply
/// \returns a reference to the newly transformed operator
Transform2D & operator*=(const Transform2D & rhs);

/// \brief \see operator<<(...) (declared outside this class)
/// for a description
friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
*/
