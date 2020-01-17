#include <gtest/gtest.h>
#include "rigid2d/rigid2d.hpp"
#include <sstream>


TEST(rigid2dLibrary, VectorIO)
{

  std::string input("3 3");
  std::string output("2D Vector, [3, 3]\n");

  std::stringstream ss_in(input);
  std::stringstream ss_out;

  rigid2d::Vector2D vec;

  ss_in >> vec;
  ss_out << vec;

  ASSERT_EQ(ss_out.str(),output);
}

TEST(rigid2dLibrary, TwistIO)
{
  std::string input("1 2 2");
  std::string output("2D Twist, [1, 2, 2]\n");

  std::stringstream ss_in(input);
  std::stringstream ss_out;

  rigid2d::Twist2D tw;

  ss_in >> tw;
  ss_out << tw;

  ASSERT_EQ(ss_out.str(),output);
}

TEST(rigid2dLibrary, TransformIO)
{

  std::string input("90 1 1");
  std::string output("2D Transform, theta(degrees): 90 x: 1 y: 1\n");

  std::stringstream ss_in(input);
  std::stringstream ss_out;

  rigid2d::Transform2D tf;

  ss_in >> tf;
  ss_out << tf;

  ASSERT_EQ(ss_out.str(),output);
}

TEST(rigid2dLibrary, VectorNormalization)
{
  rigid2d::Vector2D vec(1.0, 1.0), unit_vec;

  unit_vec = vec.normalize();

  ASSERT_PRED3(rigid2d::almost_equal, unit_vec.x, 0.707107, 1e-5);
  ASSERT_PRED3(rigid2d::almost_equal, unit_vec.y, 0.707107, 1e-5);
}

TEST(rigid2dLibrary, VectorTransform)
{
  rigid2d::Vector2D vec1, vec2(3.0, 3.0), vec2p;

  rigid2d::Transform2D tf(vec1, rigid2d::PI);

  vec2p = tf(vec2);

  ASSERT_PRED3(rigid2d::almost_equal, vec2p.x, -3, 1e-5);
  ASSERT_PRED3(rigid2d::almost_equal, vec2p.y, -3, 1e-5);
}

TEST(rigid2dLibrary, TwistTransform)
{
  rigid2d::Twist2D test(1.0, 1.0, 1.0), test_prime;
  rigid2d::Vector2D vec(1.0, 1.0);
  rigid2d::Transform2D tf(vec, rigid2d::PI);

  test_prime = tf(test);

  ASSERT_PRED3(rigid2d::almost_equal, test_prime.wz, 1, 1e-5);
  ASSERT_PRED3(rigid2d::almost_equal, test_prime.vx, 0, 1e-5);
  ASSERT_PRED3(rigid2d::almost_equal, test_prime.vy, -2, 1e-5);
}

TEST(rigid2dLibrary, TransformInverse)
{
  rigid2d::Vector2D vec(1.0, 1.0), vec2(3.0, 3.0), vec2p;
  rigid2d::Transform2D tf(vec, rigid2d::PI/2), inv_tf;

  inv_tf = tf.inv();

  vec2p = inv_tf(vec2);

  ASSERT_EQ(vec2p.x, 2);
  ASSERT_EQ(vec2p.y, -2);
}

TEST(rigid2dLibrary, TransformDisplacement)
{
  rigid2d::Twist2D tw;
  rigid2d::Vector2D vec(1,1);
  rigid2d::Transform2D tf(vec, rigid2d::PI/2);

  tw = tf.displacement();

  ASSERT_EQ(tw.wz, 90);
  ASSERT_EQ(tw.vx, vec.x);
  ASSERT_EQ(tw.vy, vec.y);
}

TEST(rigid2dLibrary, IntegrateTwist)
{
  rigid2d::Twist2D tw(1,1,1);
  rigid2d::Vector2D vec(1,1);
  rigid2d::Transform2D tf(vec, rigid2d::PI/2);

  std::stringstream ss_out;
  std::string output("2D Transform, theta(degrees): 147.296 x: 2 y: 2\n");

  tf.integrateTwist(tw);

  ss_out << tf;
  ASSERT_EQ(ss_out.str(), output);
}

TEST(rigid2dLibrary, operator_Star)
{
  rigid2d::Vector2D va(1.0, 1.0), vb(2.0, 2.0);
  rigid2d::Transform2D tab(va, rigid2d::PI/2), tbc(vb, 0);

  std::stringstream ss_out;
  std::string output("2D Transform, theta(degrees): 90 x: -1 y: 3\n");

  tab *= tbc;

  ss_out << tab;

  ASSERT_EQ(ss_out.str(), output);
}

TEST(rigid2dLibrary, operator_StarEquals)
{
  rigid2d::Vector2D va(1.0, 1.0), vb(2.0, 2.0);
  rigid2d::Transform2D tab(va, rigid2d::PI/2), tbc(vb, 0), tac;

  std::stringstream ss_out1, ss_out2;
  std::string output1("2D Transform, theta(degrees): 90 x: -1 y: 3\n");
  std::string output2("2D Transform, theta(degrees): 90 x: 1 y: 1\n");

  tac = tab * tbc;

  ss_out1 << tac;
  ASSERT_EQ(ss_out1.str(), output1);

  ss_out2 << tab;
  EXPECT_EQ(ss_out2.str(), output2);
}
