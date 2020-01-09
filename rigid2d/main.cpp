
#include <iostream>
#include "rigid2d.hpp"

using std::cout;
using std::cin;
using namespace rigid2d;

int main(void)
{

  Vector2D vec;

  vec.x = 1;
  vec.y = 2;

  cout << almost_equal(1.1, 1.0) << "\n";
  cout << deg2rad(180) << "\n";
  cout << rad2deg(PI) << "\n";
  cout << vec.x << "\n";
  cout << vec.y << "\n";

  operator>>(cin, vec);
  operator<<(cout, vec);
}

// Get input from user, T_ab and T_bc

// Compute various transforms

// enter a vector and the frame

// Compute the vector in all frames.
