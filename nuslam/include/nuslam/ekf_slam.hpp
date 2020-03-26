#ifndef LANDMARK_INCLUDE_GUARD_HPP
#define LANDMARK_INCLUDE_GUARD_HPP
/// \file
/// \brief Library to contain SLAM class and supporting functions

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

#include "geometry_msgs/Point.h"

#include "rigid2d/rigid2d.hpp"


namespace ekf_slam
{

  /// \brief Draw a random value from a normal distribution
  ///
  double sampleNormalDistribution();

  class Slam
  {
  public:
    /// \brief Initialize an instance of EKF Slam. Initializes the covarience matrix.
    /// \param num_landmarks the number of landmarks in the map
    /// \param q_var the process noise
    /// \param r_var the sensor noise
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
    /// \param id landmark id
    /// \returns a 2xstate_size matrix, H
    Eigen::MatrixXd getHMatrix(int id);

    /// \brief associate incoming data to features stored in the state matrix
    /// \param x the measured x location of a landmark
    /// \param y the measured y location of a landmark
    /// \returns the index of the matched landmark or -1 to indicate no match
    int associate_data(double x, double y);

    /// \brief caclulate the mahalonbis distance between a landmark data point and an estimated landmark state
    /// \param data_x the x value of the incoming data
    /// \param data_y the y value of the incoming data
    /// \param id the landmark index in the state vector
    /// \returns the distance between the two states
    double mahalonbis_distance(double data_x, double data_y, int id);

    /// \brief caclulate the euclidean distance between a landmark data point and an estimated landmark state
    /// \param data_x the x value of the incoming data
    /// \param data_y the y value of the incoming data
    /// \param id the landmark index in the state vector
    /// \returns the distance between the two states
    double euclidean_distance(double data_x, double data_y, int id);

    /// brief eliminate false positive landmark readings, based on the time and location last seen
    ///
    void landmark_culling();

    Eigen::MatrixXd sigma, sigma_bar; // Covarience matricies
    Eigen::VectorXd prev_state; // state vector

    Eigen::MatrixXd landmark_history; // used to track sighting information of each landmark, matrix should be X x 5, where each column is initialized, robot_x, robot_y, ros::Time(), matched

    int tot_landmarks = 0; // max allowable number of landmarks
    int created_landmarks = 0; // number of landmarks created in state vector
    int state_size = 0; // state vector size
    double deadband_min = 500.; // mah: 3000, euc: 10 cm
    double deadband_max = 10000.; // mah: 10000, euc: 20 cm

    double robot_pose_threshold = 0.1; // distance threshold for landmark culling, 10 cm
    double time_threshold = 15.0; // time threshold for landmark culling, 15 seconds

    Eigen::Matrix3d Qnoise; // motion noise model
    Eigen::Matrix2d Rnoise; // sensor noise model
  };

}
#endif
