1. What is the difference between a class and a struct in C++?

  - The class keys struct and class are indistinguishable in C++, except that the default access mode and default inheritance mode are public if class declaration uses the struct class-key and private if the class declaration uses the class class-key.


2. Why is Vector2D a struct and Transform2D Class? Refer to specific C++ Core guidelines (Classes and Class Hierarchies) in your answer. (You should refer to at least 2 guidelines).

  - The x and y values of a vector can vary independently of one another, therefore it should be a struct. (C.2)
  - The contents of the vector should ideally be public, therefore it should be a struct (C.8)


3. Why are some of the constructors in Transform2D explicit? Refer to specific C++ Core guidelines (Constructors, Assignments, and Destructors) in your answer

  - Constructors should be specified as explicit when the constructor only takes 1 argument. (C.46)

4. We need to be able to normalize Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
Propose three different designs for implementing the normalize functionality
Discuss the pros and cons of each proposed method, in light of The C++ Core guidelines (Classes and Class Hierarchies). Which of the methods would you implement and why?

  - Method 1: Normalize the Vector2D by dividing each component by the magnitude of the vector: `u = v / |v|`
  - Method 2: Normalize the Vector2D by calculating the angle of the vector and using that to calculate the x and y of a unit vector along the same angle. `u = [cos(th), sin(th)]`
  - Method 3: Normalize the Vector2D by adding another vector to it. If |v| > 1, the second vector will be


  - Method 1: Normalize the Vector2D by dividing each component by the magnitude of the vector: `u = v / |v|`
  - Method 2: Normalize the Vector2D by calculating the angle of the vector and using that to calculate the x and y of a unit vector along the same angle. `u = [cos(th), sin(th)]`
  - Method 3: Normalize the Vector2D by adding another vector to it. If |v| > 1, the second vector will be pointed in the opposite direction and have a length of |v| - 1. If |v| < 1, the second vector will be pointed in the same direction and have a length of 1 - |v|.

  A pro for method 1 and 2, is both can be computed using the existing information within the Vector2D struct. However, method 1 can be easily generalized to n dimensions, whereas method 2 is not intuitively as simple.




Implement the normalize functionality using the method you chose.
Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
Refer to C++ Core Guidelines (Constants and Immutability) in your answer
