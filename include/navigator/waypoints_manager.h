#ifndef MULTI_ROBOTS_WAYPOINTS_MANAGER_H_
#define MULTI_ROBOTS_WAYPOINTS_MANAGER_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

class WaypointsManager {
 public:
    WaypointsManager();
    void process();
    void read_waypoints();
    void timer_callback(const ros::TimerEvent &event);
    void rviz_local_goal_callback(const geometry_msgs::PoseStampedConstPtr &pose);
    bool clear_waypoints_service(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose);
    bool is_close_local_goal(const geometry_msgs::PoseWithCovarianceStamped &amcl_pose,
                             const geometry_msgs::PoseStamped &local_goal);

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Timer timer;
    ros::Subscriber pose_sub_;
    ros::Subscriber local_goal_sub_;
    ros::Publisher local_goal_pub_;
    ros::ServiceClient reached_goal_client_;
    ros::ServiceServer clear_waypoints_server_;
    int HZ;
    bool WITH_RVIZ;
    double GOAL_THRESHOLD;
    geometry_msgs::PoseWithCovarianceStamped current_pose_;
    std::vector<geometry_msgs::PoseStamped> waypoints_;
    bool reached_goal_;
};

#endif  // MULTI_ROBOTS_WAYPOINTS_MANAGER_H_
