#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "geometry_msgs/Twist.h"
#include "vesc_msgs/VescStateStamped.h"

#include <sstream>
 
std::vector<float> window;
void chatterCallback(const vesc_msgs::VescStateStamped::ConstPtr& msg)
{

  ROS_INFO(" Speed value recieved at Callback: [%f]", msg->state.speed); // prints x value of keyboard teleop
  window.push_back(msg->state.speed);

}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "patravali_average"); //node name
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/sensors/core", 1000, chatterCallback); //subcriber to keyboard teleop
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("average_velocity", 20); // std_msgs/float64 type message

  ros::Rate loop_rate(5); // set publishing rate to 5

  while (ros::ok())
  {

    std_msgs::Float64 msg;
    float sum = 0.0;


    if (window.size()>10) //minimum window size is 10
      {
      // loop over the vector for sum and then take average
      for (int i = 0; i < window.size(); i++) 
          {
              sum += window[i];
              std::cout<<window.at(i)<<' ';
          }
      for (int i = 0; i < window.size(); i++)  
          std::cout<<std::endl;


      window.erase(window.begin()); // keeping window size at 10
      msg.data = sum/(window.size()+1);
      ROS_INFO(" Publishing Moving Window average speed: %f ",sum/(window.size()+1)); //prints average speed
    }
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
