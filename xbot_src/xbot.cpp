#include <vector> 
#include "xbot_bringup/xbot.h" 
#include <iostream>
using namespace std;
namespace xbot 

{ 
const unsigned char ender[2] = {0x0d, 0x0a}; 
const unsigned char header[2] = {0x55, 0xaa}; 

const int SPEED_INFO = 0xa5;  
const int GET_SPEED  = 0x5a; 
//机器人的尺寸参数 
const double ROBOT_WIDTH = 320.00; //圆形此处改为直径
const double ROBOT_LENGTH = 230.00; 

serial::Serial sp;

//机器人在坐标系中的估计位置以及用于该位态估计的确定性的可选协方差  

boost::array<double, 36> odom_pose_covariance = { 

    {1e-9, 0, 0, 0, 0, 0,  
    0, 1e-3, 1e-9, 0, 0, 0,  
    0, 0, 1e6, 0, 0, 0, 
    0, 0, 0, 1e6, 0, 0,  
    0, 0, 0, 0, 1e6, 0,  
    0, 0, 0, 0, 0, 1e-9}}; 
/*
boost::array<double, 36> odom_pose_covariance1 = { 

    {1e-3, 0, 0, 0, 0, 0, 
     0, 1e-3, 0, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e3}}; 
*/
//机器人在子坐标系中的速度，通常是移动基站的坐标系，以及用于该速度估计的确定性的可选协方差 

boost::array<double, 36> odom_twist_covariance = { 

    {1e-9, 0, 0, 0, 0, 0,  
    0, 1e-3, 1e-9, 0, 0, 0,  
    0, 0, 1e6, 0, 0, 0,  
    0, 0, 0, 1e6, 0, 0,  
    0, 0, 0, 0, 1e6, 0,  
    0, 0, 0, 0, 0, 1e-9}}; 
/*
boost::array<double, 36> odom_twist_covariance1 = { 

    {1e-3, 0, 0, 0, 0, 0, 
     0, 1e-3, 0, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e3}}; 

*/
  
//发送数据
union sendData 
{ 
    int d; 
    unsigned char data[4]; 
}left_front_data,left_back_data,right_front_data,right_back_data,imu_yaw; 

union checkSum 
{ 
    short d; 
    unsigned char data[1]; 
}SendCheckSum, ReceiveCheckSum; 
 
union receiveHeader 
{ 
    int d; 
    unsigned char data[2]; 
}receive_command, receive_header; 

//发送请求
union sendCommand 
{ 
    int d; 
    unsigned char data[2]; 
}send_command; 

//里程计数据
union odometry 
{ 
    float odoemtry_float; 
    unsigned char odometry_char[2]; 
}vel_left_front, vel_right_front,vel_left_back, vel_right_back; 



xbot::xbot(): 
    x_(0.0), y_(0.0), th_(0.0), 
    vx_(0.0), vy_(0.0), vth_(0.0) 
{ 

} 

xbot::~xbot() 
{ 

} 

  //串口通信
bool xbot::init() 
{ 
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);
 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }

    ros::Time::init(); 
    current_time_ = ros::Time::now(); 
    last_time_ = ros::Time::now(); 

   // 定义发布消息的名称 
    pub_ = n.advertise<nav_msgs::Odometry>("odom", 100);        
    return true; 
} 
unsigned char get_data[16];
//读取速度
bool xbot::readSpeed() 
{ 
    int i, length = 0; 
    unsigned char checkSum; 
    unsigned char buf[200];      //缓冲区的大小 

    // 读取串口数据 


        //获取缓冲区内的字节数
        unsigned char buffer = sp.available();
        if(buffer!=0)
        {
              //读出数据
             sp.read(buf, buffer);
            
           // for(int i=0; i<buf; i++)
           // {
                //16进制的方式打印到屏幕
             //  std::cout << std::hex << (buffer[i] & 0xff) << " ";
           //}
           // std::cout << std::endl;

        }
   

      ros::Time curr_time; 

//buf[0]、buf[1]信息头数据
    for (i = 0; i < 2; i++) 
        receive_header.data[i] = buf[i];     

    // 检查信息头 
    if (receive_header.data[0] != header[0] || receive_header.data[1] != header[1]) 
    { 
        ROS_ERROR("Received message header error!"); 
        return false; 
    } 
//buf2]、buf[3]接收请求数据
    for (i = 0; i < 2; i++) 
    receive_command.data[i] = buf[i + 2];   
    length = buf[4]; 
     checkSum = getCrc8(buf, 5 + length);    

    // 检查信息类型 
    if(receive_command.data[0] != 0xa5||receive_command.data[1] != 0x5a) 
    { 
        ROS_ERROR("Received command error!"); 
       return false; 
    } 
    // 检查信息尾 

    if (buf[6 + length] != ender[0] || buf[6 + length + 1] != ender[1]) 
    { 
        ROS_ERROR("Received message header error!"); 
        return false; 
    } 

    // 检查信息校验值 
    ReceiveCheckSum.data[0] = buf[5 + length]; 

    if (checkSum != ReceiveCheckSum.d) 
    { 
        ROS_ERROR("Received data check sum error!"); 
        return false; 
    } 

    // 读取速度值 
    for(i = 0; i < 2; i++) 
    { 
        vel_left_front.odometry_char[i]  = buf[i + 5]; 
        vel_right_front.odometry_char[i] = buf[i + 7]; 
        vel_right_back.odometry_char[i]  = buf[i + 9]; 
        vel_left_back.odometry_char[i] = buf[i + 11];  
	//get_data[i]=vel_left_front;
//ROS_INFO("received%d,%d",buf[5],buf[6]);
    } 

  //解码
        vel_left_front.odoemtry_float  = ((buf[5]<<8)|buf[6]); 
        vel_right_front.odoemtry_float = ((buf[7]<<8)|buf[8]); 
        vel_right_back.odoemtry_float   = ((buf[9]<<8)|buf[10]); 
        vel_left_back.odoemtry_float = ((buf[11]<<8)|buf[12]);  
	
ROS_INFO("received%f,%f,%f,%f",vel_left_front.odoemtry_float,vel_right_front.odoemtry_float,vel_left_back.odoemtry_float,vel_right_back.odoemtry_float);

    // 积分计算里程计信息，更改为四个编码器的计算 
    vx_ = (vel_left_front.odoemtry_float + vel_right_front.odoemtry_float+ vel_left_back.odoemtry_float+ vel_right_back.odoemtry_float)/4/ 10000;
    vth_= (vel_left_front.odoemtry_float - vel_right_front.odoemtry_float - vel_left_back.odoemtry_float + vel_right_back.odoemtry_float) / 2/(ROBOT_WIDTH + ROBOT_LENGTH);

    curr_time = ros::Time::now(); 

    double dt = (curr_time - last_time_).toSec(); 
    double delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * dt; 
    double delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * dt; 
    double delta_th = vth_ * dt; 

    x_ += delta_x; 
    y_ += delta_y; 
    th_ += delta_th; 

    last_time_ = curr_time;                
    return true; 

} 

void xbot::writeSpeed(double RobotV, double YawRate ,const sensor_msgs::Imu imu) 

{ 
    unsigned char buf[18] = {0}; 
    int i, length = 0; 
    double L = ROBOT_WIDTH+ ROBOT_LENGTH; 
    
    // 计算左右轮期望速度，更改为四个轮的期望速度 

//实际yaw轴角度	

    tf::Quaternion quat;
    geometry_msgs::Quaternion position;
    position.x=imu.orientation.x;
    position.y=imu.orientation.y;
    position.z=imu.orientation.z;
    position.w=imu.orientation.w;

    tf::quaternionMsgToTF(position, quat);
 
    // the tf::Quaternion has a method to acess roll pitch and yaw
    
    tf::Matrix3x3(quat).getRPY(Roll, Pitch, Yaw);
    Yaw=angles::normalize_angle_positive(Yaw);

    // the found angles are written in a geometry_msgs::Vector3
   // rpy.x = Roll*180/3.1415926;
   //  rpy.y = Pitch*180/3.1415926;
   // rpy.z = Yaw*180/3.1415926;

   imu_yaw.d= Yaw*18000/3.1416;


 //ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", rpy.x, rpy.y, rpy.z);

//自转
    if(RobotV == 0) 
    { 
        right_front_data.d = YawRate * L/2;
	    left_front_data.d = -YawRate * L/ 2;
	    left_back_data.d = -YawRate * L/ 2;
	    right_back_data.d = YawRate * L/ 2;

        
    }  
	//前后直线运动
    else if(YawRate == 0) 
    { 

	    right_front_data.d = RobotV;
	    left_front_data.d = RobotV;
	    left_back_data.d = RobotV;
	    right_back_data.d = RobotV;

    } 

    else 
    { 
	    right_front_data.d = RobotV + YawRate * L / 2;
	    left_front_data.d = RobotV - YawRate * L / 2;
	    left_back_data.d = RobotV - YawRate * L / 2;
	    right_back_data.d = RobotV + YawRate * L / 2;

    } 

    // 设置消息头 
    for(i = 0; i < 2; i++){
 
        buf[0] = 0x55;    
 buf[1] = 0xaa;
}
    // 设置消息类型 
    send_command.d = GET_SPEED; 
    for(i = 0; i < 2; i++) 
        buf[i + 2] = send_command.data[i]; 

     
    // 设置机器人左右轮速度，更改为四个轮的期望速度 

    length = 10; 
    buf[4] = length; 
   
        buf[ 5] =((right_front_data.d>>8)&0xff);
        buf[ 6] =((right_front_data.d)&0xff);
        buf[ 7] = ((left_front_data.d>>8)&0xff);
        buf[ 8] = ((left_front_data.d)&0xff);
	buf[ 9] = ((left_back_data.d)>>8&0xff);
 	buf[ 10] = ((left_back_data.d)&0xff);
	buf[ 11] = ((right_back_data.d>>8)&0xff);
	buf[ 12] = ((right_back_data.d)&0xff);
        buf[ 13] = ((imu_yaw.d>>8)&0xff);
        buf[ 14] = ((imu_yaw.d)&0xff);

    // 设置校验值、消息尾 

    buf[5 + length] = getCrc8(buf, 5 + length); 
    buf[6 + length] = ender[0]; 
    buf[6 + length + 1] = ender[1]; 

           //把数据发送回去
          sp.write(buf,18);

} 


 //里程计、imu消息 
//bool xbot::

bool xbot::trans(double RobotV, double YawRate,const sensor_msgs::Imu imu) 
{ 
    // 下发机器人期望速度 
    writeSpeed(RobotV, YawRate,imu); 

     // 读取机器人实际速度 
    readSpeed(); 

  //ros::Time::init(); 
   //  current_time_ = ros::Time::now(); 
    // last_time_ = ros::Time::now(); 

    // 发布TF 
    geometry_msgs::TransformStamped odom_trans; 
    odom_trans.header.stamp = current_time_; 
    odom_trans.header.frame_id = "odom"; 
    odom_trans.child_frame_id  = "base_footprint"; 

    geometry_msgs::Quaternion odom_quat; 
    odom_quat = tf::createQuaternionMsgFromYaw(th_); 

    odom_trans.transform.translation.x = x_; 
    odom_trans.transform.translation.y = y_; 
    odom_trans.transform.translation.z = 0.0; 
    odom_trans.transform.rotation = odom_quat; 

//odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);
    odom_trans.transform.rotation.x = imu.orientation.x;
    odom_trans.transform.rotation.y = imu.orientation.y;
    odom_trans.transform.rotation.z = imu.orientation.z;
    odom_trans.transform.rotation.w = imu.orientation.w;

     
    //发送转换
    odom_broadcaster_.sendTransform(odom_trans); 

    // 发布里程计消息 （使用里程数据填充消息，并发送出去）
    nav_msgs::Odometry odom; 
    odom.header.stamp = current_time_; 
    odom.header.frame_id = "odom"; 
    odom.child_frame_id = "base_footprint";
  //设置位置消息
    odom.pose.pose.position.x = x_; 
    odom.pose.pose.position.y = y_; 
    odom.pose.pose.position.z = 0.0; 
    odom.pose.pose.orientation = odom_quat; //更改注释
    odom.pose.covariance = odom_pose_covariance; 
		
//odom.pose.pose.orientation = odom_quat;
    odom.pose.pose.orientation.x = imu.orientation.x;
    odom.pose.pose.orientation.y = imu.orientation.y;
    odom.pose.pose.orientation.z = imu.orientation.z;
    odom.pose.pose.orientation.w = imu.orientation.w;


//设置速度消息
  //  msgl.child_frame_id = "base_footprint"; 
    odom.twist.twist.linear.x = vx_; 
    odom.twist.twist.linear.y = vy_; 
    odom.twist.twist.angular.z = vth_; 
    odom.twist.covariance = odom_twist_covariance; 

//发布消息
    pub_.publish(odom); 
} 

unsigned char xbot::getCrc8(unsigned char *ptr, unsigned short len) 
{ 
    unsigned char crc; 
    unsigned char i; 

    crc = 0; 
    while(len--) 
    { 
        crc ^= *ptr++; 
        for(i = 0; i < 8; i++) 
        { 
           if(crc&0x01) 
               crc=(crc>>1)^0x8C; 
            else  
                crc >>= 1; 
        } 
    } 
    return crc; 
} 

}
