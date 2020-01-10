
#include <iostream>
#include "rigid2d.hpp"

using std::cout;
using std::cin;
using namespace rigid2d;

int main(void)
{

  // Vector2D vec;
  //
  // vec.x = 1;
  // vec.y = 2;
  //
  // cout << almost_equal(1.1, 1.0) << "\n";
  // cout << deg2rad(180) << "\n";
  // cout << rad2deg(PI) << "\n";
  // cout << vec.x << "\n";
  // cout << vec.y << "\n";
  //
  // operator>>(cin, vec);
  // operator<<(cout, vec);
  //
  // Transform2D testing1;
  // Transform2D testing2;

  Transform2D t_ab;
  Transform2D t_bc;
  Transform2D t_ac;

  cout << "Enter data for T_ab: \n";
  operator>>(cin, t_ab);
  cout << "\n" << "Enter data for T_bc: \n";
  operator>>(cin, t_bc);

  cout << "\n" << "Transform T_ab: \n";
  operator<<(cout, t_ab);

  cout << "\n" << "Transform T_ba: \n";
  operator<<(cout, t_ab.inv());

  cout << "\n" << "Transform T_bc: \n";
  operator<<(cout, t_bc);

  cout << "\n" << "Transform T_cb: \n";
  operator<<(cout, t_bc.inv());

  t_ac = operator*(t_ab, t_bc);

  cout << "\n" << "Transform T_ac: \n";
  operator<<(cout, t_ac);

  cout << "\n" << "Transform T_ca: \n";
  operator<<(cout, t_ac.inv());

  Vector2D vec;
  char frame;

  cout << "\n";
  operator>>(cin, vec);

  cout << "Enter a frame for the vector: ";
  cin >> frame;

  switch (frame)
  {
    case 'a':
      cout << "\n" << "Frame a: ";
      operator<<(cout, vec);
      cout << "Frame b: ";
      operator<<(cout, t_ab.inv().operator()(vec));
      cout << "Frame c: ";
      operator<<(cout, t_ac.inv().operator()(vec));

    break;
    case 'b':
      cout << "\n" << "Frame a: ";
      operator<<(cout, t_ab.operator()(vec));
      cout << "Frame b: ";
      operator<<(cout, vec);
      cout << "Frame c: ";
      operator<<(cout, t_bc.inv().operator()(vec));
    break;

    case 'c':
      cout << "\n" << "Frame a: ";
      operator<<(cout, t_ac.operator()(vec));
      cout << "Frame b: ";
      operator<<(cout, t_bc.operator()(vec));
      cout << "Frame c: ";
      operator<<(cout, vec);
    break;
    default:
      cout << frame << " is not a valid frame, please enter a, b, or c.\n";
  }

}

// Get input from user, T_ab and T_bc



// Compute various transforms

// enter a vector and the frame

// Compute the vector in all frames.
