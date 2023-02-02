#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
using namespace std;

ros::Publisher vel_pub;

void vel_callback(const nav_msgs::OdometryConstPtr& msg) {
  geometry_msgs::TwistStamped vel_data;
  vel_data.header.stamp = msg->header.stamp; 
  vel_data.header.frame_id = "base_link";
  vel_data.twist.linear.x = msg->twist.twist.linear.x;
  vel_data.twist.linear.y = msg->twist.twist.linear.y;
  vel_data.twist.linear.z = msg->twist.twist.linear.z;
  vel_data.twist.angular.x = msg->twist.twist.angular.x;
  vel_data.twist.angular.y = msg->twist.twist.angular.y;
  vel_data.twist.angular.z = msg->twist.twist.angular.z;
  vel_pub.publish(vel_data);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "republish_time_vel");
  cout << "republish_time_vel start" << endl;

  ros::NodeHandle nh_;
  vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("/vel_use_now_time", 10);
  ros::Subscriber vel_sub = nh_.subscribe("/vehv_pub", 10, vel_callback);

  ros::spin();
  return 0;
}
