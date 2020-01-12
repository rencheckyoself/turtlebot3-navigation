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
  cin >> t_ab;
  cout << "\n" << "Enter data for T_bc: \n";
  cin >> t_bc;


  // Print all Transforms between the three frames
  cout << "\n" << "Transform T_ab: \n";
  cout << t_ab;

  cout << "\n" << "Transform T_ba: \n";
  cout << t_ab.inv();

  cout << "\n" << "Transform T_bc: \n";
  cout << t_bc;

  cout << "\n" << "Transform T_cb: \n";
  cout << t_bc.inv();

  t_ac = operator*(t_ab, t_bc);

  cout << "\n" << "Transform T_ac: \n";
  cout << t_ac;

  cout << "\n" << "Transform T_ca: \n";
  cout << t_ac.inv();

  Vector2D vec;
  Twist2D tws;
  char frame;

  // Get vector, twist, and frame from the user
  cout << "\n";
  cin >> vec;
  cout << "\n";
  cin >> tws;

  cout << "Enter a frame for the vector and twist: ";
  cin >> frame;

  // Print the vector & twist in each frame
  switch (frame)
  {
    case 'A':
    case 'a':
      cout << "\n" << "Frame a: \n";
      cout << vec;
      cout << tws;
      cout << "Frame b: \n";
      cout << t_ab.inv()(vec);
      cout << t_ab.inv()(tws);
      cout << "Frame c: \n";
      cout << t_ac.inv()(vec);
      cout << t_ac.inv()(tws);
    break;

    case 'B':
    case 'b':
      cout << "\n" << "Frame a: \n";
      cout << t_ab(vec);
      cout << t_ab(tws);
      cout << "Frame b: \n";
      cout << vec;
      cout << tws;
      cout << "Frame c: \n";
      cout << t_bc.inv()(vec);
      cout << t_bc.inv()(tws);
    break;

    case 'C':
    case 'c':
      cout << "\n" << "Frame a: \n";
      cout << t_ac(vec);
      cout << t_ac(tws);
      cout << "Frame b: \n";
      cout << t_bc(vec);
      cout << t_bc(tws);
      cout << "Frame c: \n";
      cout << vec;
      cout << tws;
    break;

    default:
      cout << frame << " is not a valid frame, please enter a, b, or c.\n";
  }
}
