#include <gtest/gtest.h>
#include <sstream>
#include <ros/ros.h>

#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/WheelCommands.h"

static char got_wheel_data = 0;
static char got_joint_data = 0;

static nuturtlebot::WheelCommands wheel_cmd;
static sensor_msgs::JointState js_data;

void callback_wheels(nuturtlebot::WheelCommands::ConstPtr data)
{
  wheel_cmd = *data;
  got_wheel_data = 1;
}

void callback_joints(sensor_msgs::JointState::ConstPtr data)
{
  js_data = *data;
  got_sensor_data = 1;
}


TEST(TurtleInterface, TransOnly)
{
  // Proper result for a cmd_vel message with no rotational component
  ros::NodeHandle n;
  ros::Publisher pub_cmd_vel = n.advertise("turtle1/cmd_vel", 1, latch = true);
  ros::Subscriber sub_wheelcmd = n.subscribe("wheel_cmd", 1, callback_wheels);

  geometry_msgs::Twist = tw;
  tw.linear.x = 0.1; // ans 21,21

  pub_cmd_vel(tw);

  while(got_wheel_data == 0)
  {
      ros::spinOnce();
      // wait for data to pub recieved...
  }

  ASSERT_NEAR(wheel_cmd.left_velocity, 21, 1e-4);
  ASSERT_NEAR(wheel_cmd.right_velocity, 21, 1e-4);
}

TEST(TurtleInterface, RotOnly)
{
  // Proper result for a cmd_vel message with no translational component
  // 1, -17 17


}

TEST(TurtleInterface, RotAndTrans)
{
  // Proper result for a cmd_vel message
  // .1 1, 4,38
}

TEST(TurtleInterface, ValidEncs)
{
  // Proper result for a sensor message to joint_states conversion
  // ros::NodeHandle n;
  // ros::Publisher pub_sensors = n.advertise("/sensor_data");
  // ros::Subscriber sub_joints = n.subscribe("/joint_states", 1, callback_joints);

}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::NodeHandle n;

  double wheel_base = 0, wheel_radius = 0, frequency = 0;

  // n.getParam("wheel_radius", wheel_radius);
  // n.getParam("wheel_base", wheel_base);
  // n.getParam("frequency", frequency);
  //
  // rigid2d::Pose2D pos(0,0,0);
  // rigid2d::DiffDrive robot(pos, wheel_base, wheel_radius);

  return RUN_ALL_TESTS();
}
