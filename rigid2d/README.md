1. What is the difference between a class and a struct in C++?

  The class keys struct and class are indistinguishable in C++, except that the default access mode and default inheritance mode are public if class declaration uses the struct class-key and private if the class declaration uses the class class-key.

2. Why is Vector2D a struct and Transform2DClass? Refer to specific C++ Core guidelines (Classes and Class Hierarchies) in your answer. (You should refer to at least 2 guidelines).

3. Why are some of the constructors in Transform2D explicit? Refer to specific C++ Core guidelines (Constructors, Assignments, and Destructors) in your answer

4. We need to be able to normalize Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
Propose three different designs for implementing the normalize functionality
Discuss the pros and cons of each proposed method, in light of The C++ Core guidelines (Classes and Class Hierarchies)
Which of the methods would you implement and why?
Implement the normalize functionality using the method you chose.
Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
Refer to C++ Core Guidelines (Constants and Immutability) in your answer
