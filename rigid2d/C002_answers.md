1. What is the difference between a class and a struct in C++?

  - The class keys struct and class are indistinguishable in C++, except that the default access mode and default inheritance mode are public if class declaration uses the struct class-key and private if the class declaration uses the class class-key.


2. Why is Vector2D a struct and Transform2D Class? Refer to specific C++ Core guidelines (Classes and Class Hierarchies) in your answer. (You should refer to at least 2 guidelines).

  - The x and y values of a vector can vary independently of one another, therefore it should be a struct. [(C.2)](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#c2-use-class-if-the-class-has-an-invariant-use-struct-if-the-data-members-can-vary-independently)
  - The contents of the vector should ideally be public, therefore it should be a struct [(C.8)](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#c8-use-class-rather-than-struct-if-any-member-is-non-public)


3. Why are some of the constructors in Transform2D explicit? Refer to specific C++ Core guidelines (Constructors, Assignments, and Destructors) in your answer

  - Constructors should be specified as explicit when the constructor only takes 1 argument. [(C.46)](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#c46-by-default-declare-single-argument-constructors-explicit)

4. We need to be able to normalize Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
Propose three different designs for implementing the normalize functionality
Discuss the pros and cons of each proposed method, in light of The C++ Core guidelines (Classes and Class Hierarchies). Which of the methods would you implement and why?

  1. Change the Vector2D from a struct to a class and upon creating the vector, use the input to calculate and store the matching unit vector.
  2. Add a public method to the Vector2D struct that returns the unit vector from the values currently stored in the struct.
  3. Write a helper function that returns a unit vector given a Vector2D.</br>
  </br>
  Based on the Core Guidelines, related data should be kept together. Method 3 does not conform to this as the unit vector data is unrelated to the vector data and the helper function is also unrelated to the Vector2D struct. Looking at Method 1 and 2, both are feasible for different scenarios. Method 2 has the benefit of always returning correct information as the values of the Vector2D struct change, the downside is that the unit vector is always being recalculated each time the function is called. The benefit of Method 1 is that it keeps all related variables and functions together, however the downside is that the unit vector data members must be intentionally be updated each time the vector is updated. This could lead to errors if the some is not aware these variables need to be updated. Given these pros and cons, I am opting to go with Method 2 because, based on our use case, the vector will be changing a lot and this method will always ensure the correct unit vector is returned when requested.


5. Implement the normalize functionality using the method you chose.

  - See the `normalize()` method inside of the Vector2D struct.


6. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
Refer to C++ Core Guidelines (Constants and Immutability) in your answer

  - Class members that do not modify any of the contents should be marked const. `inv()` does not modify the contents whereas `operator*=()` does. [(Con.2)](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con2-by-default-make-member-functions-const)
