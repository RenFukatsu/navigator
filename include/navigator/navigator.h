#ifndef MULTI_ROBOTS_NAVIGATOR_H_
#define MULTI_ROBOTS_NAVIGATOR_H_

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include "roomba_500driver_meiji/RoombaCtrl.h"

class Navigator {
 public:
    Navigator();
    void process();
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose);
    void cmd_vel_callback(const geometry_msgs::TwistConstPtr &twist);
    void read_waypoints();
    bool is_close_local_goal(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose,
                             const geometry_msgs::PoseStamped &local_goal);

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher local_goal_pub_;
    ros::Publisher roomba_ctrl_pub_;
    double TOLERANCE;
    std::vector<geometry_msgs::PoseStamped> way_points;
};

#endif  // MULTI_ROBOTS_NAVIGATOR_H_
