#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>
#include <sstream>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Publisher");
  ros::NodeHandle n;
  
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Point>("Topic", 1000);


  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    geometry_msgs::Point msg2;

    std::stringstream ss;
    std::cout << "請輸入座標點:\n";
    std::cin >> msg2.x >> msg2.y >> msg2.z;
    
    //ss << "請輸入座標點:" << count;
    //msg.data = ss.str();
  //` ROS_INFO("%f%f%f ", msg2.x, msg2.y, msg2.z);
    chatter_pub.publish(msg2);
    ros::spinOnce();
    loop_rate.sleep();
    
    ++count;  
  }
  return 0;
}

