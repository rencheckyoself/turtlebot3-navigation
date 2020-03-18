#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <limits>
#include <cmath>
#include <random>
#include <algorithm>


#include "nuslam/TurtleMap.h"
#include "geometry_msgs/Point.h"

#include "nuslam/ekf_slam.hpp"
#include "rigid2d/rigid2d.hpp"

namespace ekf_slam
{
  static std::mt19937 & get_random()
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

  Eigen::Vector2d cart2polar(double cur_x, double cur_y)
  {
    
  }

  /////////////// Slam CLASS /////////////////////////
  Slam::Slam(int num_landmarks, Eigen::Matrix3d q_var, Eigen::Matrix2d r_var)
  {
    Qnoise = q_var;
    Rnoise = r_var;

    state_size = 3 + 2*num_landmarks;

    prev_state.resize(state_size);
    prev_state.setZero();

    sigma.resize(state_size, state_size); // Resize init covarience
    sigma_bar.resize(state_size, state_size); // Resize predicted covarience
    sigma_bar.setZero();

    sigma.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3) * 1e-8;
    sigma.topRightCorner(3, 2*num_landmarks).setZero();
    sigma.bottomLeftCorner(2*num_landmarks, 3).setZero();
    sigma.bottomRightCorner(2*num_landmarks, 2*num_landmarks) = Eigen::MatrixXd::Identity(2*num_landmarks, 2*num_landmarks) * 1e4;
  }

  void Slam::MotionModelUpdate(rigid2d::Twist2D tw)
  {

    Eigen::Vector3d noise = Slam::getStateNoise();

    Eigen::Vector3d update;
    Eigen::Vector3d dupdate;

    // std::cout << "Init. State: \n" << prev_state << "\n";

    // std::cout << "Twist: " << tw;

    double th = prev_state(0);
    std::cout << "Init. State: " << prev_state(0) << "\n";

    if(rigid2d::almost_equal(tw.wz, 0.0, 1e-5))
    {
      update(0) = 0;
      update(1) = tw.vx * std::cos(th);
      update(2) = tw.vx * std::sin(th);

      dupdate(0) = 0;
      dupdate(1) = -tw.vx * std::sin(th);
      dupdate(2) = tw.vx * std::cos(th);
    }
    else
    {
      double vel_ratio = static_cast<double>(tw.vx)/tw.wz;

      update(0) = tw.wz;
      update(1) = -vel_ratio * std::sin(th) + vel_ratio * std::sin(th + tw.wz);
      update(2) = vel_ratio * std::cos(th) - vel_ratio * std::cos(th + tw.wz);

      dupdate(0) = 0;
      dupdate(1) = -vel_ratio * std::cos(th) + vel_ratio * std::cos(th + tw.wz);
      dupdate(2) = -vel_ratio * std::sin(th) + vel_ratio * std::sin(th + tw.wz);
    }


    // Update -- Prediction
    prev_state(0) += update(0) + noise(0);
    prev_state(1) += update(1) + noise(1);
    prev_state(2) += update(2) + noise(2);

    prev_state(0) = rigid2d::normalize_angle(prev_state(0));

    std::cout << "Pred. State: " << prev_state(0) << "\n";


    // std::cout << "Update: \n" << update << "\n";
    // std::cout << "Noise: \n" << noise << "\n";

    // std::cout << "Pred. State: \n" << prev_state << "\n";

    // Landmarks do not move so no need to update that part of the state matrix

    Slam::updateCovarPrediction(dupdate);
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

    sigma_bar.setZero();
    sigma_bar = Gt * sigma * Gt.transpose() + Qbar;
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

  void Slam::MeasurmentModelUpdate(nuslam::TurtleMap map_data)
  {
    Eigen::Vector2d z_expected = Eigen::Vector2d::Zero();
    Eigen::Vector2d z_actual = Eigen::Vector2d::Zero();

    Eigen::MatrixXd Ki = Eigen::MatrixXd::Zero(state_size, state_size);
    Eigen::MatrixXd Ri = Eigen::MatrixXd::Zero(state_size, state_size);
    Eigen::MatrixXd Hi = Eigen::MatrixXd::Zero(2, state_size);

    Eigen::Vector2d noise = Eigen::Vector2d::Zero();

    double cur_x = 0, cur_y = 0;
    // double cur_r = 0;
    auto landmark_index = 0;
    double del_x = 0, del_y = 0, dist = 0;

    int data_size = map_data.centers.size();

    unsigned int landmarks_to_use = 0;

    if (data_size > (state_size-3)/2)
    {
      landmarks_to_use = (state_size-3)/2;
    }
    else
    {
      landmarks_to_use = data_size;
    }

    for(unsigned int i = 0; i < landmarks_to_use; i++)
    {
      cur_x = map_data.centers.at(i).x;
      cur_y = map_data.centers.at(i).y;
      // cur_r = map_data.radii.at(i);

      landmark_index = 3 + 2*i;

      // replace this with data association later
      if(prev_state(landmark_index) == 0 && prev_state(landmark_index+1) == 0)
      {
        std::cout << "New Landmark! setting index " << landmark_index << "\n";
        prev_state(landmark_index) = cur_x;
        prev_state(landmark_index+1) = cur_y;
      }

      // std::cout << "Landmark act_x: " << cur_x << " " << "exp_x: " << prev_state(landmark_index) << "\n";
      // std::cout << "Landmark act_y: " << cur_y << " " << "exp_y: " << prev_state(landmark_index+1) << "\n";

      // Compute actual measurment
      noise = Eigen::Vector2d::Zero();
      z_actual = cart2polar(cur_x, cur_y);

      std::cout << "Z act: " << z_actual << "\n";

      // Compute the expected measurment
      del_x = prev_state(landmark_index) - prev_state(1);
      del_y = prev_state(landmark_index + 1) - prev_state(2);

      noise = Slam::getMeasurementNoise();
      z_expected = sensorModel(del_x, del_y, noise);

      std::cout << "Bearing Noise: " << noise(1) << "\n";
      std::cout << "Z exp: " << z_expected << "\n";

      dist = del_x*del_x + del_y*del_y;

      // Assemble H Matrix
      Hi = getHMatrix(del_x, del_y, dist, landmark_index);

      // Compute the Kalman Gain
      Ki = sigma_bar * Hi.transpose() * ((Hi * sigma_bar * Hi.transpose() + Rnoise).inverse());

      Eigen::Vector2d z_diff = (z_actual - z_expected);
      z_diff(1) = rigid2d::normalize_angle(z_diff(1));

      std::cout << "Z diff: " << z_diff << "\n";

      // Update the Posterior
      prev_state += Ki * z_diff;
      prev_state(0) = rigid2d::normalize_angle(prev_state(0));

      std::cout << "Bel. State: " << prev_state(0) << "\n";

      // Update the Covarience
      sigma_bar = (Eigen::MatrixXd::Identity(state_size, state_size) - Ki * Hi) * sigma_bar;
    }

    sigma = sigma_bar;
  }

  Eigen::Vector2d Slam::sensorModel(double x, double y, Eigen::VectorXd noise)
  {
    Eigen::Vector2d output;

    double del_x = prev_state(landmark_index) - prev_state(1);
    double del_y = prev_state(landmark_index + 1) - prev_state(2);

    // range calculation
    output(0) = std::sqrt(del_x*del_x + del_y*del_y) + noise(0);

    // bearing calculation
    output(1) = std::atan2(del_y, del_x) - prev_state(0) + noise(1);

    output(1) = rigid2d::normalize_angle(output(1));

    if (output(1) > rigid2d::PI || output(1) < -rigid2d::PI)
    {
      std::cout << "SENSOR MODEL FAILED TO PROPERLY NORMALIZE ANGLE \n\n";
    }

    return output;
  }

  Eigen::MatrixXd Slam::getHMatrix(double x, double y, double d, int id)
  {
    Eigen::MatrixXd Hi = Eigen::MatrixXd::Zero(2,state_size);
    double sqd = std::sqrt(d);

    Hi(0,1) = -x/sqd;
    Hi(0,2) = -y/sqd;
    Hi(0, id) = x/sqd;
    Hi(0, id + 1) = y/sqd;

    Hi(1,0) = -1;
    Hi(1,1) = y/d;
    Hi(1,2) = -x/d;
    Hi(1, id) = -y/d;
    Hi(1, id + 1) = x/d;

    return Hi;
  }

  Eigen::VectorXd Slam::getMeasurementNoise()
  {
    // Cholesky Decomp
    Eigen::MatrixXd l(Rnoise.llt().matrixL());

    Eigen::VectorXd samples(2);

    samples(0) = sampleNormalDistribution();
    samples(1) = sampleNormalDistribution();

    Eigen::VectorXd noise(2);

    noise(0) = l(0,0) * samples(0) + l(0,1) * samples(1);
    noise(1) = l(1,0) * samples(0) + l(1,1) * samples(1);

    return noise;
  }

  std::vector<double> Slam::getRobotState()
  {
    return {prev_state(0), prev_state(1), prev_state(2)};
  }

  std::vector<geometry_msgs::Point> Slam::getLandmarkStates()
  {
    geometry_msgs::Point buf;

    std::vector<geometry_msgs::Point> output;

    for(int i = 3; i < state_size - 1; i += 2)
    {
      buf.x = prev_state(i);
      buf.y = prev_state(i+1);

      output.push_back(buf);
    }

    return output;
  }
}
