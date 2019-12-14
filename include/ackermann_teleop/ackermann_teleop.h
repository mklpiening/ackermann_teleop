#ifndef ACKERMANN_TELEOP_H
#define ACKERMANN_TELEOP_H

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class AckermannTeleop
{
public:
  AckermannTeleop();

  /**
   * @brief sets the steering angle
   *
   * @param angle new steering angle in radians
   */
  void setSteeringAngle(float angle);

  /**
   * @brief changes the speed of the vehicle
   *
   * @param speed new speed in m/s
   */
  void setSpeed(float speed);

  /**
   * @brief sets the acceleration of the vehicle
   *
   * @param acceleration acceleration in m/s^2
   */
  void setAcceleration(float acceleration);

  /**
   * @brief gets called by timer to update Velocity
   *
   * @param event timer event
   */
  void updateVelocity(const ros::TimerEvent& event);

protected:
  ros::NodeHandle nh_;

private:
  ros::Publisher vel_publisher_;
  ros::Timer vel_publish_timer_;
  ackermann_msgs::AckermannDriveStamped vel_msg_;
};

#endif
