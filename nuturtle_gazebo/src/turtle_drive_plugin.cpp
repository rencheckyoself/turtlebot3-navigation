#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/util/system.hh>

#include <iostream>
#include <vector>
#include <string>

#include "rigid2d/rigid2d.hpp"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"

namespace gazebo
{
  class TurtleDrivePlugin : public ModelPlugin
  {

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {

      // Model Plugin Boilerplate ================================================
      // =========================================================================
      // see http://gazebosim.org/tutorials?tut=ros_plugins Accessed 02/09/2020
      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized())
      {
        ROS_FATAL("A ROS node for Gazebo has not been initialized."
                  "Unable to load plugin. Load the Gazebo system plugin"
                  "'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
        return;
      }

      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&TurtleDrivePlugin::OnUpdate, this));
      // =========================================================================
      // =========================================================================

      // Set parameter variables from sdf attributes ===========================
      if(_sdf->HasElement("left_wheel_joint"))
      {
        this->left_wheel_joint = _sdf->GetElement("left_wheel_joint")->Get<std::string>();
      }
      else
      {
        gzerr << "left_wheel_joint is required in the sdf, but has not been set. Terminating plugin...";
        return;
      }

      if(_sdf->HasElement("right_wheel_joint"))
      {
        this->right_wheel_joint = _sdf->GetElement("right_wheel_joint")->Get<std::string>();
      }
      else
      {
        gzerr << "right_wheel_joint is required in the sdf, but has not been set. Terminating plugin...";
        return;
      }

      if(_sdf->HasElement("sensor_frequency"))
      {
        this->sensor_frequency = _sdf->GetElement("sensor_frequency")->Get<double>();
      }
      else
      {
        ROS_WARN("sensor_frequency has not been set in the sdf, using default of 200Hz.");
      }

      if(_sdf->HasElement("wheel_cmd_topic"))
      {
        this->wheel_cmd_topic = _sdf->GetElement("wheel_cmd_topic")->Get<std::string>();
      }
      else
      {
        gzerr << "wheel_cmd_topic is required in the sdf, but has not been set. Terminating plugin...";
        return;
      }

      if(_sdf->HasElement("sensor_data_topic"))
      {
        this->sensor_data_topic = _sdf->GetElement("sensor_data_topic")->Get<std::string>();
      }
      else
      {
        gzerr << "sensor_data_topic is required in the sdf, but has not been set. Terminating plugin...";
        return;
      }

      if(_sdf->HasElement("encoder_ticks_per_rev"))
      {
        this->encoder_ticks_per_rev = _sdf->GetElement("encoder_ticks_per_rev")->Get<double>();
      }
      else
      {
        gzerr << "encoder_ticks_per_rev is required in the sdf, but has not been set. Terminating plugin...";
        return;
      }

      if(_sdf->HasElement("motor_lim"))
      {
        this->max_motor_rot_vel = _sdf->GetElement("motor_lim")->Get<double>();
      }
      else
      {
        gzerr << "motor_lim is required in the sdf, but has not been set. Terminating plugin...";
        return;
      }

      if(_sdf->HasElement("motor_power"))
      {
        this->max_motor_power = _sdf->GetElement("motor_power")->Get<double>();
      }
      else
      {
        gzerr << "motor_power is required in the sdf, but has not been set. Terminating plugin...";
        return;
      }

      if(_sdf->HasElement("motor_torque"))
      {
        this->motor_torque = _sdf->GetElement("motor_torque")->Get<double>();
      }
      else
      {
        gzerr << "motor_torque is required in the sdf, but has not been set. Terminating plugin...";
        return;
      }

      // create node handle
      this->n.reset(new ros::NodeHandle());

      // Get paramerers from the server
      // this->n->getParam("encoder_ticks_per_rev", this->encoder_ticks_per_rev);
      // this->n->getParam("motor_lim", this->max_motor_rot_vel);
      // this->n->getParam("motor_power", this->max_motor_power);

      ROS_INFO_STREAM("PLUGIN: Got left wheel joint name: " << this->left_wheel_joint);
      ROS_INFO_STREAM("PLUGIN: Got right wheel joint name: " << this->right_wheel_joint);
      ROS_INFO_STREAM("PLUGIN: Got sensor frequency: " << this->sensor_frequency);
      ROS_INFO_STREAM("PLUGIN: Got wheel cmd topic name: " << this->wheel_cmd_topic);
      ROS_INFO_STREAM("PLUGIN: Got sensor data topic name: " << this->sensor_data_topic);

      ROS_INFO_STREAM("PLUGIN: Got encoder ticks param: " << this->encoder_ticks_per_rev);
      ROS_INFO_STREAM("PLUGIN: Got max motor rot vel param: " << this->max_motor_rot_vel);
      ROS_INFO_STREAM("PLUGIN: Got motor power param: " << this->max_motor_power);

      this->rad2enc = this->encoder_ticks_per_rev / (2 * rigid2d::PI);
      ROS_INFO_STREAM("PLUGIN: calced enc conversion: " << this->rad2enc);

      // initialize publishers and subscribers
      this->sensor_pub = this->n->advertise<nuturtlebot::SensorData>(this->sensor_data_topic, 1);
      this->wheel_sub = this->n->subscribe(this->wheel_cmd_topic, 1, &TurtleDrivePlugin::callback_wheel_cmd, this);

      last_update = model->GetWorld()->SimTime();

      // Set wheels to move according to 0
      this->model->GetJoint(left_wheel_joint)->SetParam("fmax", 0 , motor_torque);
      this->model->GetJoint(left_wheel_joint)->SetParam("vel", 0 , 0.0);
      this->model->GetJoint(right_wheel_joint)->SetParam("fmax", 0 , motor_torque);
      this->model->GetJoint(right_wheel_joint)->SetParam("vel", 0 , 0.0);
    }

    // Callback Method for the wheel cmd subscriber
    public: void callback_wheel_cmd(nuturtlebot::WheelCommands::ConstPtr data)
    {
      double m_lim[2] = {-this->max_motor_rot_vel, this->max_motor_rot_vel};
      double cmd_lim[2] = {-this->max_motor_power, this->max_motor_power};

      // convert from integer command to rad/s
      this->left_wheel_vel = rigid2d::linInterp(data->left_velocity, cmd_lim, m_lim);
      this->right_wheel_vel = rigid2d::linInterp(data->right_velocity, cmd_lim, m_lim);
      ROS_INFO_STREAM("Most recent Wheel Cmd: " << this->left_wheel_vel << " " << this->right_wheel_vel);
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      double period = 1/(this->sensor_frequency);

      gazebo::common::Time current_time = model->GetWorld()->SimTime();
      double timePassed = (current_time - last_update).Double();

      nuturtlebot::SensorData enc_vals;

      if(timePassed > period)
      {
        // Set wheels to move according to the most recent wheel_cmd data
        this->model->GetJoint(left_wheel_joint)->SetParam("fmax", 0 , motor_torque);
        this->model->GetJoint(left_wheel_joint)->SetParam("vel", 0 , this->left_wheel_vel);
        this->model->GetJoint(right_wheel_joint)->SetParam("fmax", 0 , motor_torque);
        this->model->GetJoint(right_wheel_joint)->SetParam("vel", 0 , this->right_wheel_vel);

        double left_pos = this->model->GetJoint(left_wheel_joint)->Position();
        double right_pos = this->model->GetJoint(right_wheel_joint)->Position();


        // Calculate corresponding encoder reading
        enc_vals.left_encoder = rad2enc * left_pos;
        enc_vals.right_encoder = rad2enc * right_pos;

        ROS_INFO_STREAM("Left Enc: " << enc_vals.left_encoder);
        ROS_INFO_STREAM("Right Enc: " << enc_vals.right_encoder);

        sensor_pub.publish(enc_vals);

        last_update = current_time;
      }
    }
    // sdf attributes
    private: std::string left_wheel_joint;
    private: std::string right_wheel_joint;
    private: std::string wheel_cmd_topic;
    private: std::string sensor_data_topic;
    private: int sensor_frequency = 200;

    // parameters from server
    private: double max_motor_rot_vel = 0.0;
    private: double max_motor_power = 0.0;
    private: double encoder_ticks_per_rev = 0.0;
    private: double motor_torque = 0.0;

    // ROS parameters
    private: ros::Subscriber wheel_sub;
    private: ros::Publisher sensor_pub;
    private: std::unique_ptr<ros::NodeHandle> n;

    // Wheel variables
    private: double left_wheel_vel;
    private: double right_wheel_vel;
    private: double rad2enc = 0.0;

    // Timing
    private: gazebo::common::Time last_update = 0.0;

    // Pointer to the model
    private: physics::ModelPtr model;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
}
GZ_REGISTER_MODEL_PLUGIN(gazebo::TurtleDrivePlugin)
