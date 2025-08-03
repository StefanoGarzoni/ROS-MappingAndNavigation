#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <tf/transform_datatypes.h>

struct Goal { double x, y, theta; };

int main(int argc, char** argv) {
  ros::init(argc, argv, "goal_publisher");
  ros::NodeHandle nh("~");
  std::string csv_file;
  nh.param("csv_file", csv_file, std::string("goals.csv"));
  std::ifstream file(csv_file);
  if (!file) {
    ROS_ERROR("Impossibile aprire %s", csv_file.c_str());
    return 1;
  }

  std::vector<Goal> goals;
  std::string line;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    Goal g;
    ss >> g.x; ss.ignore();
    ss >> g.y; ss.ignore();
    ss >> g.theta;
    goals.push_back(g);
  }

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  ac.waitForServer();
  for (const auto& g : goals) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = g.x;
    goal.target_pose.pose.position.y = g.y;
    tf::Quaternion q = tf::createQuaternionFromYaw(g.theta * M_PI/180.0);
    quaternionTFToMsg(q, goal.target_pose.pose.orientation);

    ROS_INFO("Invio goal: x=%.2f y=%.2f θ=%.1f°", g.x, g.y, g.theta);
    ac.sendGoal(goal);
    ac.waitForResult();
    if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_WARN("Goal fallito o abortito");
  }
  ROS_INFO("Tutti i goal processati.");
  return 0;
}