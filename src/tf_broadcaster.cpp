#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

struct OdomToTf {
  ros::Subscriber sub;
  tf::TransformBroadcaster br;

  OdomToTf(ros::NodeHandle& nh) {
    sub = nh.subscribe("/odometry", 10, &OdomToTf::callback, this);
  }

  void callback(const nav_msgs::Odometry::ConstPtr& msg) {
    tf::Transform t;
    t.setOrigin(tf::Vector3(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z));
    tf::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    t.setRotation(q);
    br.sendTransform(tf::StampedTransform(t, msg->header.stamp, "map", "base_link"));
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle nh;
  OdomToTf node(nh);
  ros::spin();
  return 0;
}