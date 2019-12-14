#include <ros/ros.h>

#include "ackermann_teleop/ackermann_teleop_joy.h"

AckermannTeleopJoy::AckermannTeleopJoy()
{
  joy_subscriber_ = nh_.subscribe<sensor_msgs::Joy>("joy", 15, &AckermannTeleopJoy::joyCallback, this);
}

void AckermannTeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  float leftTrigger = (joy->axes[5] + 1) / 2;
  float rightTrigger = (joy->axes[2] + 1) / 2;
  setSpeed(rightTrigger - leftTrigger);

  float leftStick = joy->axes[0];
  setSteeringAngle(leftStick);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ackermann_teleop_joy");

  AckermannTeleopJoy teleop;
  ROS_INFO("running!");

  ros::spin();
}
