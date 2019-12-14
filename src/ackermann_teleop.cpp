#include "ackermann_teleop/ackermann_teleop.h"

AckermannTeleop::AckermannTeleop() : new_values_received_(true)
{
  std::string cmd_pub;
  float cmd_publish_rate;
  float input_timeout;

  ros::NodeHandle nh_private("~");

  nh_private.param<float>("max_speed", max_speed_, 3);
  nh_private.param<float>("max_steering_angle", max_steering_angle_, 3.141);

  nh_private.param<std::string>("cmd_pub", cmd_pub, "ackermann_cmd");
  nh_private.param<float>("cmd_publish_rate", cmd_publish_rate, 0.2);
  nh_private.param<float>("input_timeout", input_timeout, 1.2);

  vel_publisher_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(cmd_pub, 1);
  vel_publish_timer_ = nh_.createTimer(ros::Duration(cmd_publish_rate), &AckermannTeleop::updateVelocity, this);
  input_timeout_timer_ = nh_.createTimer(ros::Duration(input_timeout), &AckermannTeleop::checkTimeout, this);
}

void AckermannTeleop::setSteeringAngle(float angle)
{
  vel_msg_.drive.steering_angle = max_steering_angle_ * angle;
  new_values_received_ = true;
}

void AckermannTeleop::setSpeed(float speed)
{
  vel_msg_.drive.speed = max_speed_ * speed;
  new_values_received_ = true;
}

void AckermannTeleop::setAcceleration(float acceleration)
{
  vel_msg_.drive.acceleration = acceleration;
  new_values_received_ = true;
}

void AckermannTeleop::updateVelocity(const ros::TimerEvent& event)
{
  vel_msg_.header.seq++;
  vel_msg_.header.stamp = ros::Time::now();

  vel_publisher_.publish(vel_msg_);
}

void AckermannTeleop::checkTimeout(const ros::TimerEvent& event)
{
  if (!new_values_received_)
  {
    setSpeed(0);
    setSteeringAngle(0);

    ROS_WARN("Input timeout! - no input values received");
  }
  new_values_received_ = false;
}
