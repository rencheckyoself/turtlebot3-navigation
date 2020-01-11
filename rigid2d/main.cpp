
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
  char frame;

  // Get vector and frame from the user
  cout << "\n";
  operator>>(cin, vec);

  cout << "Enter a frame for the vector: ";
  cin >> frame;

  // Print the vector in each frame
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
