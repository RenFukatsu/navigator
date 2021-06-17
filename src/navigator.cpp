#include "navigator/navigator.h"

Navigator::Navigator() : private_nh_("~") {
    pose_sub_ = nh_.subscribe("amcl_pose", 1, &Navigator::pose_callback, this);
    cmd_vel_sub_ = nh_.subscribe("local_path/cmd_vel", 1, &Navigator::cmd_vel_callback, this);
    local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("local_goal", 1);
    private_nh_.param("TOLERANCE", TOLERANCE, 0.5);
    read_waypoints();
}

void Navigator::read_waypoints() {
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
        pose_stamped.pose.orientation.z =
            std::sqrt(1.0 - std::pow(pose_stamped.pose.orientation.y, 2.));

        way_points.push_back(pose_stamped);
    }
}

void Navigator::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose) {
    static size_t way_points_idx = 0;
    if (is_close_local_goal(amcl_pose, way_points[way_points_idx])) {
        way_points_idx++;
        if (way_points_idx == way_points.size()) {
            ros::shutdown();
            return;
        }
    }
    geometry_msgs::PoseStamped local_goal = way_points[way_points_idx];
    local_goal.header.stamp = ros::Time::now();
    local_goal_pub_.publish(local_goal);
}

void Navigator::cmd_vel_callback(const geometry_msgs::TwistConstPtr &twist) {
    roomba_500driver_meiji::RoombaCtrl roomba_ctrl;
    roomba_ctrl.mode = roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
    roomba_ctrl.cntl.linear = twist->linear;
    roomba_ctrl.cntl.angular = twist->angular;
    roomba_ctrl_pub_.publish(roomba_ctrl);
}

bool Navigator::is_close_local_goal(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose,
    const geometry_msgs::PoseStamped &local_goal) {
    double diff_x = amcl_pose->pose.pose.position.x - local_goal.pose.position.x;
    double diff_y = amcl_pose->pose.pose.position.y - local_goal.pose.position.y;
    if (std::sqrt(diff_x * diff_x + diff_y * diff_y) <= TOLERANCE) return true;
    return false;
}

void Navigator::process() { ros::spin(); }
