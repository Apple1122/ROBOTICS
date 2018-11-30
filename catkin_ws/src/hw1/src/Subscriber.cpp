#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>

void chatterCallback(const geometry_msgs::Point::ConstPtr& msg, geometry_msgs::Point &msg2)
{
  msg2.x += msg->x;
  msg2.y += msg->y;
  msg2.z += msg->z;
  ROS_INFO("move to (%0f %0f %0f)", msg2.x, msg2.y, msg2.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Subscriber");
  ros::NodeHandle n;
  geometry_msgs::Point group ;
  group.x = 0;
  group.y = 0;
  group.z = 0;

 // ros::Subscriber sub = n.subscribe<geometry_msgs::Point>("Topic", 100, boost::bind(chatterCallback, _1, boost::ref(group)));

  ros::Subscriber sub = n.subscribe<geometry_msgs::Point>("Topic", 100, boost::bind(chatterCallback, _1, boost::ref(group)));
  ros::spin();

  return 0;
}

