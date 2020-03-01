
#include <eigen3/Eigen/Dense>
#include <vector>

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
    Eigen::MatrixX4d Vmat, Ymat, Smat;
    Eigen::JacobiSVD<MatrixX4d> zSVD;
    Eigen::Vector4d Avec, Avec_star;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixX4d> Qmat;

    // STEP 1: Compute the average x and y coordinate for a cluster
    for(auto point : cluster) {average_point += point;}

    average_point.x /= cluster.size();
    average_point.y /= cluster.size();

    // Initialize matricies
    Zmat = MatrixX4d::Zero(cluster.size(), 4);
    Mmat = MatrixX4d::Zero(cluster.size(), 4);
    Hmat_inv = MatrixX4d::Zero(4, 4);

    // ROS_INFO_STREAM("STEP 1 Complete");
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
      // Zmat(r, 0) = z;
      // Zmat(r, 1) = shift_x;
      // Zmat(r, 2) = shift_y;
      // Zmat(r, 3) = 1;
      r++;
    }
    // ROS_INFO_STREAM("STEP 2, 3, & 4 Complete");


    // STEP 5: Compute average z value for the cluster
    z_mean /= cluster.size();

    // ROS_INFO_STREAM("STEP 5 Complete");

    // Assemble the data martix
    Mmat = (1/z_mean) * Zmat.transpose() * Zmat;

    // STEP 6: Assemble the inverse of constraint matrix for the "Hyperaccurate algebraic fit"
    Hmat_inv << 0,   0, 0,       0.5,
                0,   1, 0,         0,
                0,   0, 1,         0,
                0.5, 0, 0, -2*z_mean;

    // ROS_INFO_STREAM("STEP 6 Complete");

    // STEP 7: Compute the singular value decompistion for the Z matrix
    zSVD.compute(Zmat, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // ROS_INFO_STREAM("STEP 7 Complete");

    // ROS_INFO_STREAM("Singular Values Size: " << zSVD.singularValues().size());

    // Check the 4th element of the vector of singularValues
    if(zSVD.singularValues()(3) <= 1e-12)
    {
      // STEP 8: if the last singular value is less than 10^-12, A vector is the
      // last column of the V matrix.
      Avec = zSVD.matrixV().col(3);
      // ROS_INFO_STREAM("STEP 8 < Complete");
    }
    else
    {
      // STEP 8: if the last singular value is less than 10^-12, A vector is
      // calculated by finding the eigenvalues and eigenvectors of Y
      Smat = zSVD.singularValues().asDiagonal();
      Ymat = zSVD.matrixV() * Smat * zSVD.matrixV().transpose();

       Qmat.compute(Ymat);

      val_found = 0;
      k = 0;
      // Find smallest Eigenvalue
      while(val_found == 0)
      {
        if(k > 3)
        {
          // ROS_ERROR_STREAM("Did not find a minimum Eigen Value.");
        }
        else if(Qmat.eigenvalues()(k) > 0)
        {
          Avec_star = Qmat.eigenvectors().col(k);
          val_found = 1;
        }
        k++;
      }
      Avec = Ymat.inverse() * Avec_star;
      // ROS_INFO_STREAM("STEP 8 > Complete");
    }

    // STEP 9: Compute the a, b, and R^2 to define the equation of a circle
    // R^2 = (x+a)^2 + (y+b)^2

    // ROS_INFO_STREAM(" Avec size: " << Avec.size());

    a = -Avec(1) / (2 * Avec(0));
    b = -Avec(2) / (2 * Avec(0));
    radius2 = (pow(Avec(1), 2) + pow(Avec(2), 2) - (4 * Avec(0) * Avec(3))) / (4 * pow(Avec(1), 2));

    // ROS_INFO_STREAM("STEP 9 Complete");

    return {a + average_point.x, b + average_point.y, pow(radius2, 0.5)};
  }

    // Compute MSE Function
}
