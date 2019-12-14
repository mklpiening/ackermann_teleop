#ifndef ACKERMANN_TELEOP_JOY_H
#define ACKERMANN_TELEOP_JOY_H

#include "ackermann_teleop/ackermann_teleop.h"

#include <sensor_msgs/Joy.h>

class AckermannTeleopJoy : public AckermannTeleop
{
public:
  /**
   * @brief initializes the joy based teleoperation for ackermann vehicles
   *
   * Avaliable Parameters are:
   * joy_topic: topic of thje incoming joy messages; default: joy
   */
  AckermannTeleopJoy();

  /**
   * @brief gets called for every incoming joy message
   *
   * @param joy incoming joy message
   */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

private:
  ros::Subscriber joy_subscriber_;
};

#endif
