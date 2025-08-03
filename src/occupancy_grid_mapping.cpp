#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <mutex>
#include <cmath>

struct GridMapper {
  double resolution = 0.05;
  int width = 400, height = 400;
  double origin_x = -10.0, origin_y = -10.0;
  double log_odds0 = 0.0, log_odds_occ = 0.85, log_odds_free = -0.4;
  std::vector<double> log_odds;
  nav_msgs::OccupancyGrid map_msg;
  double pose_x = 0, pose_y = 0, pose_theta = 0;
  ros::Subscriber sub_scan, sub_odom;
  ros::Publisher pub_map;
  ros::Timer timer;
  std::mutex mtx;

  GridMapper(ros::NodeHandle& nh) {
    log_odds.assign(width*height, log_odds0);
    map_msg.info.resolution = resolution;
    map_msg.info.width = width;
    map_msg.info.height = height;
    map_msg.info.origin.position.x = origin_x;
    map_msg.info.origin.position.y = origin_y;
    map_msg.info.origin.orientation.w = 1.0;
    map_msg.data.assign(width*height, -1);

    sub_scan = nh.subscribe("/scan", 10, &GridMapper::scanCB, this);
    sub_odom = nh.subscribe("/odometry", 10, &GridMapper::odomCB, this);
    pub_map  = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    timer    = nh.createTimer(ros::Duration(1.0), &GridMapper::publishMap, this);
  }

  void odomCB(const nav_msgs::Odometry::ConstPtr& odom) {
    std::lock_guard<std::mutex> lk(mtx);
    pose_x = odom->pose.pose.position.x;
    pose_y = odom->pose.pose.position.y;
    pose_theta = tf::getYaw(odom->pose.pose.orientation);
  }

  void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::lock_guard<std::mutex> lk(mtx);
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      double r = scan->ranges[i];
      if (std::isnan(r) || r < scan->range_min || r > scan->range_max) continue;
      double ang = scan->angle_min + i*scan->angle_increment + pose_theta;
      double ex = pose_x + r * cos(ang);
      double ey = pose_y + r * sin(ang);
      updateCell(ex, ey, log_odds_occ);
      updateFree(pose_x, pose_y, ex, ey);
    }
  }

  void updateCell(double wx, double wy, double delta) {
    int mx, my;
    if (!worldToMap(wx, wy, mx, my)) return;
    int idx = my*width + mx;
    log_odds[idx] = std::min(5.0, std::max(-5.0, log_odds[idx] + delta));
  }

  void updateFree(double x0, double y0, double x1, double y1) {
    int x0m, y0m, x1m, y1m;
    if (!worldToMap(x0, y0, x0m, y0m) || !worldToMap(x1, y1, x1m, y1m)) return;
    int dx = std::abs(x1m - x0m), dy = std::abs(y1m - y0m);
    int sx = x0m < x1m ? 1 : -1, sy = y0m < y1m ? 1 : -1;
    int err = dx - dy;
    int x = x0m, y = y0m;
    while (!(x == x1m && y == y1m)) {
      int idx = y*width + x;
      log_odds[idx] = std::min(5.0, std::max(-5.0, log_odds[idx] + log_odds_free));
      int e2 = 2*err;
      if (e2 > -dy) { err -= dy; x += sx; }
      if (e2 <  dx) { err += dx; y += sy; }
    }
  }

  bool worldToMap(double wx, double wy, int& mx, int& my) {
    mx = int((wx - origin_x)/resolution);
    my = int((wy - origin_y)/resolution);
    return mx >= 0 && mx < width && my >= 0 && my < height;
  }

  void publishMap(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lk(mtx);
    for (size_t i = 0; i < log_odds.size(); ++i) {
      double p = 1.0 - 1.0/(1.0 + exp(log_odds[i]));
      map_msg.data[i] = (log_odds[i] == 0.0 ? -1 : int(p*100));
    }
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = "map";
    pub_map.publish(map_msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "occupancy_grid_mapping");
  ros::NodeHandle nh;
  GridMapper gm(nh);
  ros::spin();
  return 0;
}