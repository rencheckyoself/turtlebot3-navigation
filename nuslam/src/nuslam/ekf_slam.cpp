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

  // Convert cartesian coordintes to their polar equivalent
  static Eigen::Vector2d cart2polar(double x, double y)
  {
    Eigen::Vector2d output;
    output.setZero();

    output(0) = std::sqrt(x*x + y*y);
    output(1) = std::atan2(y,x);

    return output;
  }

  double sampleNormalDistribution()
  {
    std::normal_distribution<> d(0, 1);
    return d(get_random());
  }

  /////////////// Slam CLASS /////////////////////////
  Slam::Slam(int num_landmarks, Eigen::Matrix3d q_var, Eigen::Matrix2d r_var)
  {
    Qnoise = q_var;
    Rnoise = r_var;

    state_size = 3 + 2*num_landmarks;

    tot_landmarks = num_landmarks;

    prev_state.resize(state_size);
    prev_state.setZero();

    sigma.resize(state_size, state_size); // Resize init covarience
    sigma_bar.resize(state_size, state_size); // Resize predicted covarience
    sigma_bar.setZero();

    sigma.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3) * 1e-8;
    sigma.topRightCorner(3, 2*num_landmarks).setZero();
    sigma.bottomLeftCorner(2*num_landmarks, 3).setZero();
    sigma.bottomRightCorner(2*num_landmarks, 2*num_landmarks) = Eigen::MatrixXd::Identity(2*num_landmarks, 2*num_landmarks) * 1e6;
  }

  void Slam::MotionModelUpdate(rigid2d::Twist2D tw)
  {

    Eigen::Vector3d noise = Slam::getStateNoise();

    Eigen::Vector3d update;
    Eigen::Vector3d dupdate;

    double th = prev_state(0);

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

    // Limits the amount of data processed to either the size of the incoming data vectors
    // or the predetermined amount of landmarks
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

      // replace this with data association later also update to convert relative measurment to the map frame
      // landmark_index = -1;
      landmark_index = associate_data(cur_x, cur_y);


      // If the landmark was unmatched, outside the deadband, and if there is room
      // left in the state vector, add it.
      if(landmark_index == -1 && created_landmarks < tot_landmarks)
      {

        int next_landmark = 3 + 2*created_landmarks;

        if(prev_state(next_landmark) == 0 && prev_state(next_landmark+1) == 0)
        {
          landmark_index = next_landmark;
          std::cout << "New Landmark! Setting index " << landmark_index << "\n";
          prev_state(landmark_index) = cur_x + prev_state(1);
          prev_state(landmark_index+1) = cur_y + prev_state(2);
          created_landmarks++;
        }
        else // error catching
        {
          std::cout << "Tried to overwrite an existing landmark! \n";
        }
      }
      else if(landmark_index <= -1) // data was in the the deadband of an existing landmark
      {
        continue;
      }

      // Compute actual measurment
      z_actual = cart2polar(cur_x, cur_y);

      // Compute the expected measurment
      noise = Slam::getMeasurementNoise();
      z_expected = sensorModel(prev_state(landmark_index), prev_state(landmark_index + 1), noise);

      // Compute error
      Eigen::Vector2d z_diff = (z_actual - z_expected);
      z_diff(1) = rigid2d::normalize_angle(z_diff(1));

      // Assemble H Matrix
      del_x = prev_state(landmark_index) - prev_state(1);
      del_y = prev_state(landmark_index + 1) - prev_state(2);
      dist = del_x*del_x + del_y*del_y;
      Hi = getHMatrix(del_x, del_y, dist, landmark_index);

      // Compute the Kalman Gain
      Ki = sigma_bar * Hi.transpose() * ((Hi * sigma_bar * Hi.transpose() + Rnoise).inverse());

      // Update the Posterior
      prev_state += Ki * z_diff;
      prev_state(0) = rigid2d::normalize_angle(prev_state(0));

      // Update the Covarience
      sigma_bar = (Eigen::MatrixXd::Identity(state_size, state_size) - Ki * Hi) * sigma_bar;
    }

    // Update Covarience
    sigma = sigma_bar;
  }

  int Slam::associate_data(double x, double y)
  {

    int landmark_index = -1;
    int output_index = -1;

    // Check data point against all created landmarks
    for(int i = 0; i < created_landmarks; i++)
    {
      landmark_index = 3 + 2*i;

      Eigen::Vector2d noise;
      noise.setZero();

      Eigen::Vector2d z = sensorModel(prev_state(landmark_index), prev_state(landmark_index + 1), noise);

      double x_diff = x - z(0)*std::cos(z(1));
      double y_diff = y - z(0)*std::sin(z(1));

      double dist = std::sqrt(x_diff*x_diff + y_diff*y_diff);

      if(dist < deadband_min) // if less than the deadband, consider this a match
      {
        // std::cout << "Matched landmark data id " << i << ": " << x << " " << y << "\n with state data: " << prev_state(landmark_index) << " " << prev_state(landmark_index+1) << "\n\n";
        output_index = landmark_index;
        break;
      }
      else if(dist > deadband_max) // if greater than the deadband, consider this a potentially new landmark
      {
        output_index += 0;
      }
      else // if inside the deadband, ignore the data
      {
        // std::cout  << "Deadband hit\n";
        output_index -= 1;
      }
    }

    return output_index;
  }

  Eigen::Vector2d Slam::sensorModel(double x, double y, Eigen::VectorXd noise)
  {
    Eigen::Vector2d output;

    double del_x = x - prev_state(1);
    double del_y = y - prev_state(2);

    output = cart2polar(del_x, del_y);

    output += noise;
    output(1) -= prev_state(0);

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
      buf.x = prev_state(i) + 0.05; // added value to offset state because base_scan is offset from base_link
      buf.y = prev_state(i+1);

      output.push_back(buf);
    }

    return output;
  }
}
