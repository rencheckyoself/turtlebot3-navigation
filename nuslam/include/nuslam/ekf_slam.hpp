#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

#include "rigid2d/rigid2d.hpp"

namespace ekf_slam
{

  std::mt19937 & get_random();
  double sampleNormalDistribution();

  class Slam
  {
  public:
    /// \brief Initialize an instance of EKF Slam. Initializes the covarience matrix.
    /// \param num_landmarks the number of landmarks in the map
    Slam(int num_landmarks, Eigen::Matrix3d q_var);


    /// \brief Predict the current state of the robot using the motion model.
    ///
    void MotionModelUpdate(rigid2d::Twist2D tw, int num_landmarks);

    Eigen::VectorXd getStateNoise();

    void updateCovarPrediction(Eigen::Vector3d dupdate);

  private:

    double normalDistributionVal();

    Eigen::MatrixXd sigma;
    Eigen::VectorXd prev_state;

    int state_size=0;

    Eigen::Matrix3d Qnoise;
    Eigen::MatrixXd Hnoise;
  };

}
