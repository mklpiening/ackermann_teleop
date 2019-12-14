#include "ackermann_teleop/ackermann_teleop.h"

AckermannTeleop::AckermannTeleop()
{
  vel_publisher_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd", 1);
  vel_publish_timer_ = nh_.createTimer(ros::Duration(0.1), &AckermannTeleop::updateVelocity, this);
}

void AckermannTeleop::setSteeringAngle(float angle)
{
  float maxSteeringAngle = 3.141;
  vel_msg_.drive.steering_angle = maxSteeringAngle * angle;
}

void AckermannTeleop::setSpeed(float speed)
{
  float maxSpeed = 3;
  vel_msg_.drive.speed = maxSpeed * speed;
}

void AckermannTeleop::setAcceleration(float acceleration)
{
  vel_msg_.drive.acceleration = acceleration;
}

void AckermannTeleop::updateVelocity(const ros::TimerEvent& event)
{
  vel_msg_.header.seq++;
  vel_msg_.header.stamp = ros::Time::now();

  vel_publisher_.publish(vel_msg_);
}
