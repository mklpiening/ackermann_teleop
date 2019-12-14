#ifndef ACKERMANN_TELEOP_H
#define ACKERMANN_TELEOP_H

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class AckermannTeleop
{
public:
  /**
   * @brief uses ROS params of the node to initialize the the teoperation
   *
   * Available Parameters are:
   * max_speed: maximum allowed speed in both directions in m/s; default: 3m/s
   * max_steering_angle: maximum allowed steering angle in one direction in radians; default: 3.141°
   * cmd_pub: topic of the commands; default: ackermann_cmd
   * cmd_publish_rate: publish rate in seconds; default: 0.2
   * input_timeout: time until timeout when no message is received in seconds; default: 1.2
   */
  AckermannTeleop();

  /**
   * @brief sets the steering angle
   *
   * takes a value of the scale from -1 to 1 and maps it to the angle in radians by multiplying the input value with the
   * maximum speering angle of the vehicle. Positive values steer to the left, negative values to the right.
   *
   * @param angle new steering angle
   */
  void setSteeringAngle(float angle);

  /**
   * @brief changes the speed of the vehicle
   *
   * takes a value of the scale from -1 to 1 and maps it to the speed in m/s by multiplying the given value by the
   * maximum allowed speed. By doing this 1 marks the maximum speed forward and -1 the maximum speed backwards;
   *
   * @param speed new speed in a scale from -1 to 1
   */
  void setSpeed(float speed);

  /**
   * @brief sets the acceleration of the vehicle
   *
   * @param acceleration acceleration in m/s^2
   */
  void setAcceleration(float acceleration);

protected:
  ros::NodeHandle nh_;

private:
  /**
   * @brief gets called by timer to send command
   *
   * @param event timer event
   */
  void updateVelocity(const ros::TimerEvent& event);

  /**
   * @brief gets called by timer to check for timeout
   *
   * @param event timer event
   */
  void checkTimeout(const ros::TimerEvent& event);

  ros::Publisher vel_publisher_;
  ros::Timer vel_publish_timer_;
  ros::Timer input_timeout_timer_;
  ackermann_msgs::AckermannDriveStamped vel_msg_;

  float max_speed_;
  float max_steering_angle_;

  /**
   * @brief for input timeout: true if new input values were set; else false
   */
  bool new_values_received_;
};

#endif
