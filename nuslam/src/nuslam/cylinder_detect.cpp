
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

#include "rigid2d/rigid2d.hpp"
#include "nuslam/cylinder_detect.hpp"

namespace cylinder
{
  typedef Eigen::Matrix<double, Eigen::Dynamic, 4> MatrixX4d;

  std::vector<double> fit_circles(std::vector<rigid2d::Vector2D> cluster)
  {
    rigid2d::Vector2D average_point;
    int val_found = 0, k = 0;
    double z_mean=0, z=0;
    double shift_x=0, shift_y=0;
    double a=0, b=0, radius2=0;
    int r=0;
    Eigen::MatrixX4d Zmat, Mmat, Hmat_inv;
    Eigen::MatrixX4d Vmat, Ymat, Smat, Qmat;
    Eigen::JacobiSVD<MatrixX4d> zSVD;
    Eigen::Vector4d Avec, Avec_star;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixX4d> EigSol;

    // STEP 1: Compute the average x and y coordinate for a cluster
    for(auto point : cluster) {average_point += point;}

    average_point.x /= static_cast<double>(cluster.size());
    average_point.y /= static_cast<double>(cluster.size());

    // std::cout << "Average Point: " << average_point;

    // Initialize matricies
    Zmat = MatrixX4d::Zero(cluster.size(), 4);
    Mmat = MatrixX4d::Zero(cluster.size(), 4);
    Hmat_inv = MatrixX4d::Zero(4, 4);

    // std::cout << "STEP 1 Complete \n";

    r = 0;
    for(auto point : cluster)
    {
      // STEP 2: Shift each point so the center is near the origin
      shift_x = point.x - average_point.x;
      shift_y = point.y - average_point.y;

      // STEP 3: Compute z for each point in the cluster
      z = pow(shift_x, 2) + pow(shift_y, 2);
      z_mean += z;

      // STEP 4: Assemble Z matrix
      Zmat.row(r) << z, shift_x, shift_y, 1;
      r++;
    }

    // std::cout << "Z matrix" << '\n';
    // std::cout << Zmat << '\n';
    //
    // std::cout << "STEP 2, 3, & 4 Complete \n";


    // STEP 5: Compute average z value for the cluster
    z_mean /= static_cast<double>(cluster.size());

    // std::cout << "Z mean" << '\n';
    // std::cout << z_mean << '\n';
    //
    // std::cout << "STEP 5 Complete \n";

    // Assemble the data martix
    Mmat = (1/static_cast<double>(cluster.size())) * Zmat.transpose() * Zmat;

    // STEP 6: Assemble the inverse of constraint matrix for the "Hyperaccurate algebraic fit"
    Hmat_inv << 0.0, 0.0, 0.0, 0.5,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.5, 0.0, 0.0, -2.0*z_mean;

    // std::cout << "Hinv matrix" << '\n';
    // std::cout << Hmat_inv << '\n';
    //
    // std::cout << "STEP 6 Complete \n";

    // STEP 7: Compute the singular value decompistion for the Z matrix
    zSVD.compute(Zmat, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // std::cout << "STEP 7 Complete" << '\n';

    // Check the 4th element of the vector of singularValues
    if(zSVD.singularValues()(3) <= 1e-12)
    {
      // STEP 8: if the last singular value is less than 10^-12, A vector is the
      // last column of the V matrix.
      Avec = zSVD.matrixV().col(3);

      // std::cout << "A vec" << '\n';
      // std::cout << Avec << '\n';
      //
      // std::cout << "STEP 8 < Complete \n";
    }
    else
    {
      // STEP 8: if the last singular value is less than 10^-12, A vector is
      // calculated by finding the eigenvalues and eigenvectors of Y
      Smat = zSVD.singularValues().asDiagonal();

      // std::cout << "Sigma" << '\n';
      // std::cout << Smat << '\n';

      Ymat = zSVD.matrixV() * Smat * zSVD.matrixV().transpose();

      // std::cout << "Ymat" << '\n';
      // std::cout << Ymat << '\n';

      Qmat = Ymat * Hmat_inv * Ymat;

      // std::cout << "Qmat" << '\n';
      // std::cout << Qmat << '\n';

      EigSol.compute(Qmat);

      val_found = 0;
      k = 0;
      // Find smallest Eigenvalue

      // std::cout << "Eigenvalues: \n";
      // std::cout << EigSol.eigenvalues() << "\n";

      // std::cout << "Eigenvectors: \n";
      // std::cout << EigSol.eigenvectors() << "\n";

      while(val_found == 0)
      {
        if(k > 3)
        {
          // ROS_ERROR_STREAM("Did not find a minimum Eigen Value.");
        }
        else if(EigSol.eigenvalues()(k) > 0)
        {
          Avec_star = EigSol.eigenvectors().col(k);
          val_found = 1;
        }
        k++;
      }

      // std::cout << "A*: " << '\n';
      // std::cout << Avec_star << '\n';

      Avec = Ymat.completeOrthogonalDecomposition().solve(Avec_star);

      // std::cout << "A: " << '\n';
      // std::cout << Avec << '\n';
      //
      // std::cout << "STEP 8 > Complete \n";
    }

    // STEP 9: Compute the a, b, and R^2 to define the equation of a circle
    // R^2 = (x+a)^2 + (y+b)^2

    a = -Avec(1) / (2.0 * Avec(0));
    b = -Avec(2) / (2.0 * Avec(0));
    radius2 = (pow(Avec(1), 2) + pow(Avec(2), 2) - (4.0 * Avec(0) * Avec(3))) / (4.0 * pow(Avec(0), 2));

    // std::cout << "a, b, R2" << '\n';
    // std::cout << a << " " << b << " " << radius2 << '\n';
    //
    // std::cout << "STEP 9 Complete \n";

    return {a + average_point.x, b + average_point.y, std::sqrt(radius2)};
  }

  // Compute MSE Function
}
