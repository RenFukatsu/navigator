#include "navigator/waypoints_manager.h"

WaypointsManager::WaypointsManager()
    : private_nh_("~"), reached_goal_(false), mt_(std::random_device {}()), dist_(0, 1000) {
    pose_sub_ = nh_.subscribe("amcl_pose", 1, &WaypointsManager::pose_callback, this);
    local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("local_goal", 1);
    reached_goal_client_ = nh_.serviceClient<std_srvs::SetBool>(nh_.getNamespace() + "/reached_goal");
    private_nh_.param("HZ", HZ, 10);
    private_nh_.param("LOOP_WAYPOINTS", LOOP_WAYPOINTS, false);
    private_nh_.param("RANDOM_WAYPOINTS", RANDOM_WAYPOINTS, false);
    private_nh_.param("GOAL_THRESHOLD", GOAL_THRESHOLD, 0.8);
    bool WITH_RVIZ;
    private_nh_.param("WITH_RVIZ", WITH_RVIZ, false);
    if (RANDOM_WAYPOINTS) {
        read_waypoints(&points_);
        read_points_relation();
        int idx;
        ROS_ASSERT(private_nh_.getParam("FIRST_POSITION", idx));
        pre_pos_idx_ = idx;
        ROS_ASSERT(private_nh_.getParam("FIRST_LOCAL_GOAL", idx));
        next_pos_idx_ = idx;
        waypoints_.push_back(points_[next_pos_idx_]);
    } else if (!WITH_RVIZ) {
        read_waypoints(&waypoints_);
        ROS_INFO_STREAM("Read waypoints size = " << waypoints_.size() << ".");
    } else {
        local_goal_sub_ = nh_.subscribe("rviz/local_goal", 1, &WaypointsManager::rviz_local_goal_callback, this);
        clear_waypoints_server_ =
            nh_.advertiseService("clear_waypoints", &WaypointsManager::clear_waypoints_service, this);
    }
}

void WaypointsManager::read_waypoints(std::vector<geometry_msgs::PoseStamped> *waypoints) {
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

        waypoints->push_back(pose_stamped);
    }
}

void WaypointsManager::read_points_relation() {
    XmlRpc::XmlRpcValue param_list;
    for (size_t i = 0;; i++) {
        std::string param_name = "relation" + std::to_string(i);
        if (!private_nh_.getParam(param_name.c_str(), param_list)) break;
        ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

        std::vector<int> points;
        for (size_t j = 0; j < param_list.size(); j++) {
            ROS_ASSERT(param_list[j].getType() == XmlRpc::XmlRpcValue::TypeInt);
            points.push_back(param_list[j]);
        }
        ROS_ASSERT(points_relation_.size() == i);
        points_relation_.push_back(points);
    }
}

void WaypointsManager::rviz_local_goal_callback(const geometry_msgs::PoseStampedConstPtr &pose) {
    waypoints_.push_back(*pose);
    ROS_INFO_STREAM("Add waypoints. The Number of waypoints is " << waypoints_.size() << ".");
}

bool WaypointsManager::clear_waypoints_service(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    waypoints_.clear();
    res.success = true;
    res.message = "Clear waypoints.";
    ROS_INFO_STREAM("Clear waypoints.");
    return true;
}

void WaypointsManager::timer_callback(const ros::TimerEvent &event) {
    static size_t way_points_idx = 0;
    static auto call_reached_goal = [](ros::ServiceClient &client, bool reached_goal) -> void {
        std_srvs::SetBool set_bool;
        set_bool.request.data = reached_goal;
        if (client.call(set_bool)) {
            ROS_DEBUG_STREAM(set_bool.response.message);
        } else {
            ROS_ERROR("Failed to call service reached_goal");
        }
    };
    if (way_points_idx < waypoints_.size() && is_close_local_goal(current_pose_, waypoints_[way_points_idx])) {
        way_points_idx++;
    }
    if (way_points_idx == waypoints_.size() && RANDOM_WAYPOINTS) {
        const auto &candidate = points_relation_[next_pos_idx_];
        int random_idx = dist_(mt_) % candidate.size();
        if (candidate[random_idx] == pre_pos_idx_) {
            random_idx++;
            random_idx %= candidate.size();
        }
        pre_pos_idx_ = next_pos_idx_;
        next_pos_idx_ = candidate[random_idx];
        waypoints_.push_back(points_[next_pos_idx_]);
    } else if (way_points_idx == waypoints_.size() && LOOP_WAYPOINTS) {
        way_points_idx = 0;
    } else if (way_points_idx >= waypoints_.size()) {  // >= for when clear_waypoints is called
        if (way_points_idx != waypoints_.size()) way_points_idx = waypoints_.size();
        ROS_INFO_THROTTLE(15.0, "Robot reached last waypoint.");
        if (!reached_goal_) {
            call_reached_goal(reached_goal_client_, true);
            reached_goal_ = true;
        }
        return;
    } else if (reached_goal_) {
        call_reached_goal(reached_goal_client_, false);
        reached_goal_ = false;
    }
    geometry_msgs::PoseStamped local_goal = waypoints_[way_points_idx];
    local_goal.header.stamp = ros::Time::now();
    local_goal_pub_.publish(local_goal);
}

void WaypointsManager::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose) {
    current_pose_ = *amcl_pose;
}

bool WaypointsManager::is_close_local_goal(const geometry_msgs::PoseWithCovarianceStamped &amcl_pose,
                                           const geometry_msgs::PoseStamped &local_goal) {
    double diff_x = amcl_pose.pose.pose.position.x - local_goal.pose.position.x;
    double diff_y = amcl_pose.pose.pose.position.y - local_goal.pose.position.y;
    if (std::sqrt(diff_x * diff_x + diff_y * diff_y) <= GOAL_THRESHOLD) return true;
    return false;
}

void WaypointsManager::process() {
    timer = nh_.createTimer(ros::Duration(1.0 / HZ), &WaypointsManager::timer_callback, this);
    ros::spin();
}
