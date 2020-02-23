#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
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
          std::bind(&ModelPush::OnUpdate, this));

    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      ROS_WARN("Plugin Loaded Successfully!");
      // ROS_WARN("Plugin Loaded Successfully!");
      // Apply a small linear velocity to the model.
      // this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
