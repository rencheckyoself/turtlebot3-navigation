#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <limits>
#include <cmath>
#include <random>

#include "nuslam/ekf_slam.hpp"
#include "rigid2d/rigid2d.hpp"

namespace ekf_slam
{
  std::mt19937 & get_random()
  {
      // static variables inside a function are created once and persist for the remainder of the program
      static std::random_device rd{};
      static std::mt19937 mt{rd()};
      // we return a reference to the pseudo-random number genrator object. This is always the
      // same object every time get_random is called
      return mt;
  }

  double sampleNormalDistribution()
  {
    std::normal_distribution<> d(0, 1);
    return d(get_random());
  }


  /////////////// Slam CLASS /////////////////////////
  Slam::Slam(int num_landmarks, Eigen::Matrix3d q_var)
  {
    Qnoise = q_var;

    state_size = 3 + 2*num_landmarks;

    prev_state.resize(state_size);
    prev_state.setZero();

    sigma.resize(state_size, state_size); // Resize init covarience

    sigma.topLeftCorner(3,3).setZero();
    sigma.topRightCorner(3, 2*num_landmarks).setZero();
    sigma.bottomLeftCorner(2*num_landmarks, 3).setZero();
    sigma.bottomRightCorner(2*num_landmarks, 2*num_landmarks) = Eigen::MatrixXd::Identity(2*num_landmarks, 2*num_landmarks) * 10000;
  }

  void Slam::MotionModelUpdate(rigid2d::Twist2D tw, int num_landmarks)
  {

    state_size = 3 + 2*num_landmarks;

    Eigen::Vector3d noise = Slam::getStateNoise();

    Eigen::Vector3d update;
    Eigen::Vector3d dupdate;

    double th = prev_state(2);

    if(tw.wz == 0)
    {
      update(0) = 0;
      update(1) = tw.vx * cos(th);
      update(2) = tw.vy * sin(th);

      dupdate(0) = 0;
      dupdate(1) = -tw.vx * sin(th);
      dupdate(2) = tw.vx * cos(th);
    }
    else
    {
      double vel_ratio = tw.vx/tw.wz;

      update(0) = tw.wz;
      update(1) = -vel_ratio * sin(th) + vel_ratio * sin(th + tw.wz);
      update(2) = vel_ratio * cos(th) - vel_ratio * cos(th + tw.wz);

      dupdate(0) = 0;
      dupdate(1) = -vel_ratio * cos(th) + vel_ratio * cos(th + tw.wz);
      dupdate(2) = -vel_ratio * sin(th) + vel_ratio * sin(th + tw.wz);
    }

    // Update -- Prediction
    prev_state(0) += update(0) + noise(0);
    prev_state(1) += update(1) + noise(1);
    prev_state(2) += update(2) + noise(2);

    // Landmarks do not move so no need to update that part of the state matrix

    Slam::updateCovarPrediction(dupdate);
  }

  Eigen::VectorXd Slam::getStateNoise()
  {
    // Cholesky Decomp
    Eigen::MatrixXd l(Qnoise.llt().matrixL());

    Eigen::VectorXd samples(3);

    samples(0) = sampleNormalDistribution();
    samples(1) = sampleNormalDistribution();
    samples(2) = sampleNormalDistribution();

    Eigen::VectorXd noise(3);

    noise(0) = l(0,0) * samples(0) + l(0,1) * samples(1) + l(0,2) * samples(2);
    noise(1) = l(1,0) * samples(0) + l(1,1) * samples(1) + l(1,2) * samples(2);
    noise(2) = l(2,0) * samples(0) + l(2,1) * samples(1) + l(2,2) * samples(2);

    return noise;
  }


  void Slam::updateCovarPrediction(Eigen::Vector3d dupdate)
  {
    Eigen::MatrixXd Gt = Eigen::MatrixXd::Zero(state_size,state_size);

    Gt(0, 0) = dupdate(0);
    Gt(1, 0) = dupdate(1);
    Gt(2, 0) = dupdate(2);

    Gt += Eigen::MatrixXd::Identity(state_size, state_size);

    Eigen::MatrixXd Qbar = Eigen::MatrixXd::Zero(state_size,state_size);

    Qbar.topLeftCorner(3, 3) = Qnoise;

    sigma = Gt * sigma * Gt.transpose() + Qbar;
  }
}
