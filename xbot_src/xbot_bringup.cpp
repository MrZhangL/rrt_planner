#include "xbot_bringup/xbot.h"   

double RobotV_ = 0; 
double YawRate_ = 0; 
sensor_msgs::Imu imu_;

// 速度控制消息的回调函数 

void cmdCallback(const geometry_msgs::Twist& msg) 
{ 
	// 在回调函数中将接收到的cmd_vel速度消息转换为自定义的结构体（或者union）类型的数据
    RobotV_ = msg.linear.x * 10000; //机器人的速度 
    YawRate_ = msg.angular.z;   //机器人的旋转角度 

  //RobotV_ = 1000; //机器人的速度 
    //YawRate_ =80;   //机器人的旋转角度 
} 
//IMU回调函数
void imuCallback(const sensor_msgs::Imu &imu_msg)
{
    imu_ = imu_msg;

//ROS_INFO("imu initialized successful.");
}
     
int main(int argc, char** argv) 
{ 
   //初始化ROS节点 
    ros::init(argc,argv,"xbot_bringup");                                      
    ros::NodeHandle n; 

    //初始化xbot 
    xbot::xbot robot; 
    if(!robot.init()) 
        ROS_ERROR("xbot initialized failed."); 
   
    ROS_INFO("xbot initialized successful."); 

    //ros::Publisher pub_ = n.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Subscriber sub = n.subscribe("cmd_vel", 100, cmdCallback); 
    ros::Subscriber imu_sub = n.subscribe("imu", 100, imuCallback);
    
    //循环运行 
    ros::Rate loop_rate(10); 

    while (ros::ok())  
    { 
        ros::spinOnce(); 

        // 机器人控制 
        robot.trans(RobotV_, YawRate_, imu_); 
        //ros::spin(); 

        loop_rate.sleep(); 

    } 
    return 0; 
} 

 
