/// \file
/// \brief Manipulates 2D Transforms, Vectors, and Twists to test the ridig2d files.
///

#include <iostream>
#include "rigid2d.hpp"

using std::cout;
using std::cin;
using namespace rigid2d;

int main(void)
{

  Transform2D t_ab;
  Transform2D t_bc;
  Transform2D t_ac;

  // User inputs T_ab and T_bc
  cout << "Enter data for T_ab: \n";
  operator>>(cin, t_ab);
  cout << "\n" << "Enter data for T_bc: \n";
  operator>>(cin, t_bc);


  // Print all Transforms between the three frames
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
  Twist2D tws;
  char frame;

  // Get vector, twist, and frame from the user
  cout << "\n";
  operator>>(cin, vec);
  cout << "\n";
  operator>>(cin, tws);

  cout << "Enter a frame for the vector and twist: ";
  cin >> frame;

  // Print the vector & twist in each frame
  switch (frame)
  {
    case 'a':
      cout << "\n" << "Frame a: \n";
      operator<<(cout, vec);
      operator<<(cout, tws);
      cout << "Frame b: \n";
      operator<<(cout, t_ab.inv().operator()(vec));
      operator<<(cout, t_ab.inv().operator()(tws));
      cout << "Frame c: \n";
      operator<<(cout, t_ac.inv().operator()(vec));
      operator<<(cout, t_ac.inv().operator()(tws));

    break;
    case 'b':
      cout << "\n" << "Frame a: \n";
      operator<<(cout, t_ab.operator()(vec));
      operator<<(cout, t_ab.operator()(tws));
      cout << "Frame b: \n";
      operator<<(cout, vec);
      operator<<(cout, tws);
      cout << "Frame c: \n";
      operator<<(cout, t_bc.inv().operator()(vec));
      operator<<(cout, t_bc.inv().operator()(tws));
    break;

    case 'c':
      cout << "\n" << "Frame a: \n";
      operator<<(cout, t_ac.operator()(vec));
      operator<<(cout, t_ac.operator()(tws));
      cout << "Frame b: \n";
      operator<<(cout, t_bc.operator()(vec));
      operator<<(cout, t_bc.operator()(tws));
      cout << "Frame c: \n";
      operator<<(cout, vec);
      operator<<(cout, tws);
    break;
    default:
      cout << frame << " is not a valid frame, please enter a, b, or c.\n";
  }
}
