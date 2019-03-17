#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
     
int main(int argc, char** argv) 
{ 
   //初始化ROS节点 
    ros::init(argc,argv,"get_pose");                                      
    ros::NodeHandle n;  

   ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("robot_localization", 100); 
    //循环运行 
    ros::Rate loop_rate(50);
    costmap_2d::Costmap2DROS* planner_costmap_ros;

    while (ros::ok())  
    { 
        ros::spinOnce(); 

        tf::Stamped<tf::Pose> global_pose;
        planner_costmap_ros->getRobotPose(global_pose);
        geometry_msgs::PoseStamped current_position;
        tf::poseStampedTFToMsg(global_pose, current_position);
        pub.publish(current_position);

        loop_rate.sleep(); 

    } 
    return 0; 
} 

 
