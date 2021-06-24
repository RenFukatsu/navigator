#include "navigator/waypoints_manager.h"

WaypointsManager::WaypointsManager() : private_nh_("~") {
    pose_sub_ = nh_.subscribe("amcl_pose", 1, &WaypointsManager::pose_callback, this);
    reached_goal_pub_ = nh_.advertise<std_msgs::Bool>("reached_goal", 1);
    local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("local_goal", 1);
    private_nh_.param("WITH_RVIZ", WITH_RVIZ, false);
    private_nh_.param("GOAL_THRESHOLD", GOAL_THRESHOLD, 0.8);
    if (!WITH_RVIZ) read_waypoints();
}

void WaypointsManager::read_waypoints() {
    XmlRpc::XmlRpcValue param_list;
    for (size_t i = 0;; i++) {
        std::string param_name = "waypoint" + std::to_string(i);
        if (!private_nh_.getParam(param_name.c_str(), param_list)) break;
        ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

        ROS_ASSERT(param_list.size() == 7);
        for (size_t j = 0; j < param_list.size(); j++) {
            ROS_ASSERT(param_list[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        }

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = param_list[0];
        pose_stamped.pose.position.y = param_list[1];
        pose_stamped.pose.position.z = param_list[2];
        pose_stamped.pose.orientation.w = param_list[3];
        pose_stamped.pose.orientation.x = param_list[4];
        pose_stamped.pose.orientation.y = param_list[5];
        pose_stamped.pose.orientation.z = param_list[6];

        waypoints_.push_back(pose_stamped);
    }
}

void WaypointsManager::pose_callback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose) {
    static size_t way_points_idx = 0;
    static auto publish_reached_goal = [](const ros::Publisher &pub, bool reached_goal) {
        std_msgs::Bool msg;
        msg.data = reached_goal;
        pub.publish(msg);
    };
    if (way_points_idx == waypoints_.size()) {
        ROS_INFO_THROTTLE(15.0, "Robot reached goal.");
        publish_reached_goal(reached_goal_pub_, true);
        return;
    } else {
        publish_reached_goal(reached_goal_pub_, false);
    }
    if (is_close_local_goal(amcl_pose, waypoints_[way_points_idx])) {
        way_points_idx++;
    }
    geometry_msgs::PoseStamped local_goal = waypoints_[way_points_idx];
    local_goal.header.stamp = ros::Time::now();
    local_goal_pub_.publish(local_goal);
}

bool WaypointsManager::is_close_local_goal(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose,
    const geometry_msgs::PoseStamped &local_goal) {
    double diff_x = amcl_pose->pose.pose.position.x - local_goal.pose.position.x;
    double diff_y = amcl_pose->pose.pose.position.y - local_goal.pose.position.y;
    if (std::sqrt(diff_x * diff_x + diff_y * diff_y) <= GOAL_THRESHOLD) return true;
    return false;
}

void WaypointsManager::process() { ros::spin(); }
