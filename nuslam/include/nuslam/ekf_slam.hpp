#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

#include "geometry_msgs/Point.h"

#include "rigid2d/rigid2d.hpp"

namespace ekf_slam
{

  // std::mt19937 & get_random();
  double sampleNormalDistribution();

  class Slam
  {
  public:
    /// \brief Initialize an instance of EKF Slam. Initializes the covarience matrix.
    /// \param num_landmarks the number of landmarks in the map
    Slam(int num_landmarks, Eigen::Matrix3d q_var, Eigen::Matrix2d r_var);

    /// \brief Predict the current state of the robot using the motion model.
    /// \param tw a twist command the robot will follow
    void MotionModelUpdate(rigid2d::Twist2D tw);

    /// \brief Incorperate sensor information into the prediction from the motion model
    /// \param &map_data a reference to the landmarks observed by the robot
    void MeasurmentModelUpdate(nuslam::TurtleMap map_data);

    /// \brief Extract the robot state
    /// \returns a vector of the robot state (th, x, y)
    std::vector<double> getRobotState();

    /// \brief Extract the landmark states
    /// \returns a vector of the points
    std::vector<geometry_msgs::Point> getLandmarkStates();

  private:
    /// \brief Update the Covar based on the the motion model prediction
    /// \param dupdate a vetor containing the elements for the derivative of the motion model
    void updateCovarPrediction(Eigen::Vector3d dupdate);

    /// \brief Generate a random noise based on the process varience
    /// \returns a vector of noise for the state of the robot
    Eigen::VectorXd getStateNoise();

    /// \brief Generate a random noise based on the measurment varience
    /// \returns a vector of noise for the measurement of a landmark
    Eigen::VectorXd getMeasurementNoise();

    /// \brief Compute the range and bearing to a landmark given the robot state
    /// \param x landmark x position
    /// \param y landmark y position
    /// \returns a vector containing the range and bearing
    Eigen::Vector2d sensorModel(double x, double y, Eigen::VectorXd noise);

    /// \brief Assemble measurement model derivative matrix
    /// \param x x distance between the landmark and robot
    /// \param y y distance between the landmark and robot
    /// \param d squared distance between the landmark and robot
    /// \param id landmark id
    /// \returns a 2xstate_size matrix, H
    Eigen::MatrixXd getHMatrix(double x, double y, double d, int id);

    /// \brief Get a random number from a normal distribution
    /// \returns a random number
    double normalDistributionVal();

    Eigen::MatrixXd sigma, sigma_bar;
    Eigen::VectorXd prev_state;

    int state_size=0;

    Eigen::Matrix3d Qnoise;
    Eigen::Matrix2d Rnoise;
  };

}
