#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub__ = n__.advertise<visualization_msgs::Marker>("/RRT_tree", 10);

    //Topic you want to subscribe
    sub__ = n__.subscribe("/robot_1/move_base_node/srrt_planner/visualization_marker", 10, &SubscribeAndPublish::callback,this);
  }

  void callback(const visualization_msgs::Marker& input)
  {
    //.... do something with the input and generate the output...

    pub__.publish(input);
  }

private:
  ros::NodeHandle n__; 
  ros::Publisher pub__;
  ros::Subscriber sub__;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}