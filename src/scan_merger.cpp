#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

struct ScanMerger {
    ros::Publisher pub;
    double robot_radius;

    ScanMerger(ros::NodeHandle& nh) {
        nh.param("robot_radius", robot_radius, 0.3);
        message_filters::Subscriber<sensor_msgs::LaserScan> subF(nh, "/scan_front", 10);
        message_filters::Subscriber<sensor_msgs::LaserScan> subB(nh, "/scan_back", 10);
        message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::LaserScan>
          sync(subF, subB, 10);
        sync.registerCallback(boost::bind(&ScanMerger::callback, this, _1, _2));
        pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 10);
    }

    void callback(const sensor_msgs::LaserScan::ConstPtr& f,
                  const sensor_msgs::LaserScan::ConstPtr& b) {
        sensor_msgs::LaserScan out = *f;
        out.ranges.clear();
        for (double r : f->ranges) if (r > robot_radius) out.ranges.push_back(r);
        for (double r : b->ranges) if (r > robot_radius) out.ranges.push_back(r);
        out.header.stamp = ros::Time::now();
        out.header.frame_id  = "base_link";
        pub.publish(out);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_merger");
    ros::NodeHandle nh;
    ScanMerger m(nh);
    ros::spin();
    return 0;
}