/// \file
/// \brief This node tests the landmark node circle fitting functions

#include <gtest/gtest.h>
#include <sstream>
#include <vector>

#include "rigid2d/rigid2d.hpp"
#include "nuslam/cylinder_detect.hpp"

TEST(Landmark, CircleTest1)
{
  std::vector<rigid2d::Vector2D> input_cluster = {rigid2d::Vector2D(1,7), rigid2d::Vector2D(2,6), rigid2d::Vector2D(5,8), rigid2d::Vector2D(7,7), rigid2d::Vector2D(9,5), rigid2d::Vector2D(3,7)};

  std::vector<double> circle_vals = cylinder::fit_circles(input_cluster);

  ASSERT_NEAR(circle_vals.at(0), 4.615482, 1e-4);
  ASSERT_NEAR(circle_vals.at(1), 2.807354, 1e-4);
  ASSERT_NEAR(circle_vals.at(2), 4.8275, 1e-4);

}
//
// TEST(Landmark, CircleTest2)
// {
//
// }
