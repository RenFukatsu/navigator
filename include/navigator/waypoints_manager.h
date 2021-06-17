#ifndef MULTI_ROBOTS_WAYPOINTS_MANAGER_H_
#define MULTI_ROBOTS_WAYPOINTS_MANAGER_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

class WaypointsManager {
 public:
    WaypointsManager();
    void process();
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose);
    void read_waypoints();
    bool is_close_local_goal(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose,
                             const geometry_msgs::PoseStamped &local_goal);

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber pose_sub_;
    ros::Publisher reached_goal_pub_;
    ros::Publisher local_goal_pub_;
    bool WITH_RVIZ;
    double GOAL_THRESHOLD;
    std::vector<geometry_msgs::PoseStamped> waypoints_;
};

#endif  // MULTI_ROBOTS_WAYPOINTS_MANAGER_H_
