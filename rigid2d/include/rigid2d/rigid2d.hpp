#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath> // standard math functions

namespace rigid2d
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
    /// be useful here
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
      return std::fabs(d1 - d2) < epsilon ? true : false;
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr double deg2rad(double deg)
    {
      return deg * (PI/180.0);
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return rad * (180.0/PI);
    }

    /// \brief maps an angle to the range [-pi, pi)
    /// \param rad - any angle
    /// \return angle mapped to the range [-pi, pi)
    constexpr double normalize_angle(double rad)
    {
      double ang = 0;
      double i = 0;

      i = std::floor((rad + PI)/(2.0 * PI));
      ang = rad + PI - (i * 2.0 * PI);
      if(ang < 0)
      {
        ang += 2.0*PI;
      }
      return ang - PI;
    }

    /// \brief Linear Interpolation function
    /// \param x value to interpolate from xlims to ylims
    /// \param xlims limits for the input range
    /// \param ylims limits for the output range
    /// \return angle mapped to the range [-pi, pi)
    constexpr double linInterp(double x, const double xlims[], const double ylims[])
    {
      double y = 0;
      y = ylims[0] + (x - xlims[0]) * ((ylims[1] - ylims[0])/(xlims[1] - xlims[0]));
      return y;
    }

    /// static_assertions test compile time assumptions.
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(0.001, 0.005, 1.0e-1), "is_zero failed");
    static_assert(almost_equal(-1, -1.0001, 1.0e-3), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(deg2rad(180), PI), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg failed");

    static_assert(almost_equal(rad2deg(PI), 180), "rad2deg failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    static_assert(almost_equal(normalize_angle(0),0), "normalized angle falied");
    static_assert(almost_equal(normalize_angle(PI),-PI, 1e-4), "normalized angle falied");
    static_assert(almost_equal(normalize_angle(-PI),-PI, 1e-4), "normalized angle falied");
    static_assert(almost_equal(normalize_angle(1.5*PI),-PI/2, 1e-4), "normalized angle falied");
    static_assert(almost_equal(normalize_angle(2.5*PI),PI/2, 1e-4), "normalized angle falied");
    static_assert(almost_equal(normalize_angle(-1.5*PI),PI/2, 1e-4), "normalized angle falied");
    static_assert(almost_equal(normalize_angle(-2.5*PI),-PI/2, 1e-4), "normalized angle falied");
    static_assert(almost_equal(normalize_angle(deg2rad(370)), deg2rad(10), 1e-4), "normalized angle falied");
    static_assert(almost_equal(normalize_angle(deg2rad(-190)),deg2rad(170), 1e-4), "normalized angle falied");

    // lin interpolate assert input array
    static constexpr double x_test[2] = {-6.35492,6.35492};
    // lin interpolate assert output array
    static constexpr double y_test[2] = {-265,265};

    static_assert(almost_equal(linInterp(-6.35492, x_test, y_test), -265), "Linear Interpolation Failed");

    // static_assert(almost_equal(linInterp(1, x_test, y_test), 2), "Linear Interpolation Failed");
    // static_assert(almost_equal(linInterp(0, x_test, y_test), 1), "Linear Interpolation Failed");
    // static_assert(almost_equal(linInterp(2, x_test, y_test), 3), "Linear Interpolation Failed");

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        double x; ///< x val of vector
        double y; ///< y val of vector

        /// \brief create an empty vector
        Vector2D();

        /// \brief create a vector
        /// \param xcomp - the x vector component
        /// \param ycomp - the y vector component
        Vector2D(double xcomp, double ycomp);

        /// \brief output a unit vector corresponding to the [xcomponent ycomponent]
        /// \return the unit vector corresponding to the current x and y values of the Vector2D
        Vector2D normalize() const;

        /// \brief calculate the length of the vector based on the components
        /// \return the length of this vector
        double length() const;

        /// \brief calculate the angle of the vector based on the components
        /// \return the angle of this vector in degrees
        double angle() const;

        /// \brief calculate the dot product between this vector and another
        /// \vec another Vector2D
        /// \returns the dot product
        double dot(Vector2D vec) const;

        /// \brief calculate the distance between this vector and another
        /// \param vec - vector to find the distance to
        /// \return the distance between the two vectors
        double distance(Vector2D vec) const;

        /// \brief add this vector with another and store the result
        /// in this object
        /// \param rhs - the vector to add by
        /// \returns a reference to the newly transformed operator
        Vector2D & operator+=(const Vector2D & rhs);

        /// \brief subtract this vector with another and store the result
        /// in this object
        /// \param rhs - the vector to subtract by
        /// \returns a reference to the newly transformed operator
        Vector2D & operator-=(const Vector2D & rhs);

        /// \brief multiply(scalar) this vector with another and store the result
        /// in this object
        /// \param rhs - the vector to multiply by
        /// \returns a reference to the newly transformed operator
        Vector2D & operator*=(const Vector2D & rhs);

        /// \brief evaluate if two vectors are the same
        /// \param rhs - the vector to compare with
        /// \returns True if both x and y are the same
        bool operator==(const Vector2D & rhs);

        /// \brief evaluate if two vectors are the not the same
        /// \param rhs - the vector to compare with
        /// \returns True if both x and y are the same
        bool operator!=(const Vector2D & rhs);
    };

    /// \brief add two vectors together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two vectors
    Vector2D operator+(Vector2D lhs, const Vector2D & rhs);

    /// \brief subtract two vectors together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two vectors
    Vector2D operator-(Vector2D lhs, const Vector2D & rhs);

    /// \brief multiply(scalar) two vectors together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two vectors
    Vector2D operator*(Vector2D lhs, const Vector2D & rhs);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as two numbers
    ///   separated by a newline or a space, or entered as [xcomponent ycomponent]
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief A 2-Dimensional Twist
    struct Twist2D
    {
    public:

      // angular z, linear x and y
      double wz = 0.0; ///< rotational velocity
      double vx = 0.0; ///< x translational velocity
      double vy = 0.0; ///< y translational velocity

      /// \brief create an zero twist
      ///
      Twist2D();

      /// \brief create a twist
      /// \param ang - the angular component
      /// \param linx - the x velocity component
      /// \param liny - the y velocity component
      Twist2D(double ang, double linx, double liny);

      /// \brief scale this twist based on the current unit time
      /// \param dt - the unit time
      /// \returns this twist scaled by dt to
      Twist2D scaleTwist(double dt);
    };

    /// \brief A 2-Dimensional Pose
    struct Pose2D
    {
    public:

      // heading, x, and y
      double th = 0.0; ///< heading angle
      double x = 0.0; ///< x position
      double y = 0.0; ///< y position

      /// \brief create an empty pose
      Pose2D();

      /// \brief create a pose
      /// \param ang - the angular heading
      /// \param xpos - the x position
      /// \param ypos - the y position
      Pose2D(double ang, double xpos, double ypos);
    };

    /// \brief output a 2 dimensional twist as [wx wy vx vy]
    /// os - stream to output to
    /// tw - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw);

    /// \brief input a 2 dimensional twist
    /// tw [out] - output twist
    std::istream & operator>>(std::istream & is, Twist2D & tw);

    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(const Vector2D & trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param radians - the rotation, in radians
        Transform2D(const Vector2D & trans, double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param pose - the translation and rotation
        Transform2D(const Pose2D);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief apply a transformation to a Twist2D
        /// \param tw - the twist to transform
        /// \return a twist in the new frame
        Twist2D operator()(Twist2D tw) const;

        /// \brief invert the transformation
        /// \return the inverse transformation.
        Transform2D inv() const;

        /// \brief retrieve information about the transform
        /// \return the angle (in degs) and translation of the transform
        Pose2D displacement() const;

        /// \brief retrieve information about the transform
        /// \return the angle (in rads) and translation of the transform
        Pose2D displacementRad() const;

        /// \brief advance the current transform by a twist for one time unit
        /// \param tw - the twist to follow
        /// \return The transform relative to the world after following the given twist
        Transform2D integrateTwist(const Twist2D tw) const;

        /// \brief compose this transform with another and store the result
        /// in this object
        /// \param rhs - the first transform to apply
        /// \returns a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    private:
        /// directly initialize, useful for forming the inverse
        Transform2D(double theta, double ctheta, double stheta, double x, double y);
        double theta, ctheta, stheta, x, y; // angle, sin, cos, x, and y
    };


    /// \brief Compute the transform relative to the initial position after following the given twist
    /// \param tw - the twist to follow
    /// \returns the transform relative to the initial position after following the given twist
    Transform2D transformFromTwist(Twist2D tw);

    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

}
#endif
