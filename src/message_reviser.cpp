#include "navigator/message_reviser.h"

MessageReviser::MessageReviser() : private_nh_("~"), update_local_cmd_vel_(false) {
    roomba_odometry_sub_ =
        nh_.subscribe("roomba/odometry", 1, &MessageReviser::roomba_odometry_callback, this);
    local_cmd_vel_sub_ =
        nh_.subscribe("local_path/cmd_vel", 1, &MessageReviser::local_cmd_vel_callback, this);
    reached_goal_sub_ =
        nh_.subscribe("reached_goal", 1, &MessageReviser::reached_goal_callback, this);
    corrected_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("roomba/corrected_odometry", 1);
    roomba_ctrl_pub_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1);
    private_nh_.param("HZ", HZ, 10);
    private_nh_.param("LINEAR_COEF", LINEAR_COEF, 2.0);
    private_nh_.param("START_SPEED", START_SPEED, 0.3);
}

void MessageReviser::roomba_odometry_callback(const nav_msgs::OdometryConstPtr &odom) {
    nav_msgs::Odometry corrected_odom = *odom;
    corrected_odom.twist.twist.linear.x /= LINEAR_COEF;
    corrected_odom.twist.twist.angular.z /= LINEAR_COEF;
    corrected_odom_pub_.publish(corrected_odom);
}

void MessageReviser::local_cmd_vel_callback(const geometry_msgs::TwistConstPtr &twist) {
    if (reached_goal_) return;
    roomba_500driver_meiji::RoombaCtrl roomba_ctrl =
        create_ctrl(LINEAR_COEF * twist->linear.x, LINEAR_COEF * twist->angular.z);
    roomba_ctrl_pub_.publish(roomba_ctrl);
    update_local_cmd_vel_ = true;
}

void MessageReviser::reached_goal_callback(const std_msgs::BoolConstPtr &msg) {
    if (msg->data) {
        reached_goal_ = true;
        roomba_500driver_meiji::RoombaCtrl roomba_ctrl = create_ctrl(0.0, 0.0);
        roomba_ctrl_pub_.publish(roomba_ctrl);
    } else {
        reached_goal_ = false;
    }
}

roomba_500driver_meiji::RoombaCtrl MessageReviser::create_ctrl(double linear_x, double angular_z) {
    roomba_500driver_meiji::RoombaCtrl roomba_ctrl;
    roomba_ctrl.mode = roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
    roomba_ctrl.cntl.linear.x = linear_x;
    roomba_ctrl.cntl.angular.z = angular_z;
    return roomba_ctrl;
}

void MessageReviser::process() {
    ros::Rate loop_rate(HZ);
    static auto start_time = ros::Time::now();
    while (ros::ok()) {
        double elasped_time = (ros::Time::now() - start_time).toSec();
        if (elasped_time <= 5.0 && !update_local_cmd_vel_) {
            ROS_WARN_THROTTLE(1.0, "Move Forward");
            roomba_500driver_meiji::RoombaCtrl roomba_ctrl = create_ctrl(START_SPEED, 0.);
            roomba_ctrl_pub_.publish(roomba_ctrl);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
