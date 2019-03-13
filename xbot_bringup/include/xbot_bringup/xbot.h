#ifndef XBOT_H
#define XBOT_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <serial/serial.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Vector3.h"
#include "tf/LinearMath/Quaternion.h"
#include "angles/angles.h"

namespace xbot
{

struct Speed{
    float vel_left_front;
    float vel_right_front;
    float vel_left_back;
    float vel_right_back;
};

class xbot
{
public:
    xbot();
    ~xbot();
    bool init();
    bool trans(double RobotV, double YawRate, const sensor_msgs::Imu imu);
   sensor_msgs::Imu imu_;

private:
    bool readSpeed(Speed *speed);
    void CalculateOdom();
    void writeSpeed(double RobotV, double YawRate,const sensor_msgs::Imu imu);
    unsigned char getCrc8(unsigned char *ptr, unsigned short len);
   
private:
    ros::Time current_time_, last_time_;

    double x_;
    double y_;
    double th_;

    double vx_;
    double vy_;
    double vth_;
    double Roll, Pitch, Yaw;
  

    ros::NodeHandle n;
    ros::Publisher pub_;
    tf::TransformBroadcaster odom_broadcaster_;
};
    
}

#endif /* xbot_H */
