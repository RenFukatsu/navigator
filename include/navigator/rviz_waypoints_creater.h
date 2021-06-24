#ifndef MULTI_ROBOTS_RVIZ_WAYPOINTS_CREATER_H_
#define MULTI_ROBOTS_RVIZ_WAYPOINTS_CREATER_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <fstream>

class RvizWaypointsCreater {
 public:
    RvizWaypointsCreater();
    void pose_callback(const geometry_msgs::PoseStampedConstPtr &pose);
    void process();

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber pose_sub_;

    std::string filename;
};

#endif  // MULTI_ROBOTS_RVIZ_WAYPOINTS_CREATER_H_
