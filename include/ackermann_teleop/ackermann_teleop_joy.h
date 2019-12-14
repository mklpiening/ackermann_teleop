#ifndef ACKERMANN_TELEOP_JOY_H
#define ACKERMANN_TELEOP_JOY_H

#include "ackermann_teleop/ackermann_teleop.h"

#include <sensor_msgs/Joy.h>

class AckermannTeleopJoy : public AckermannTeleop
{
public:
    AckermannTeleopJoy();

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
private:
    ros::Subscriber joy_subscriber_;
};

#endif
