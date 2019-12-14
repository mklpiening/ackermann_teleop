#include <ros/ros.h>

#include "ackermann_teleop/ackermann_teleop_joy.h"

AckermannTeleopJoy::AckermannTeleopJoy()
{
  std::string joy_topic;

  ros::NodeHandle nh_private("~");

  nh_private.param<std::string>("joy_topic", joy_topic, "joy");

  joy_subscriber_ = nh_.subscribe<sensor_msgs::Joy>(joy_topic, 15, &AckermannTeleopJoy::joyCallback, this);
}

void AckermannTeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  float leftTrigger = (joy->axes[5] + 1) / 2;
  float rightTrigger = (joy->axes[4] + 1) / 2;
  setSpeed(leftTrigger - rightTrigger);

  float leftStick = joy->axes[0];
  setSteeringAngle(leftStick);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ackermann_teleop_joy");

  AckermannTeleopJoy teleop;

  ros::spin();
}
