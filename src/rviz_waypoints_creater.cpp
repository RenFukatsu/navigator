#include "navigator/rviz_waypoints_creater.h"

RvizWaypointsCreater::RvizWaypointsCreater() : private_nh_("~") {
    pose_sub_ =
        nh_.subscribe("/move_base_simple/goal", 1, &RvizWaypointsCreater::pose_callback, this);
    if (!private_nh_.hasParam("filename")) {
        ROS_ERROR_STREAM("RvizWaypointsCreater required \"filename\" parameter.");
        ros::shutdown();
        return;
    }
    private_nh_.getParam("filename", filename);
}

void RvizWaypointsCreater::pose_callback(const geometry_msgs::PoseStampedConstPtr &pose) {
    static std::ofstream ofs(filename);
    static int index = 0;
    ofs << "waypoints" << index++ << ": [" << pose->pose.position.x << ", " << pose->pose.position.y
        << ", " << pose->pose.position.z << ", " << pose->pose.orientation.w << ", "
        << pose->pose.orientation.x << ", " << pose->pose.orientation.y << ", "
        << pose->pose.orientation.z << "]\n";
}

void RvizWaypointsCreater::process() { ros::spin(); }

int main(int argc, char **argv) {
    ros::init(argc, argv, "rviz_waypoints_creater");
    RvizWaypointsCreater rviz_waypoints_creater;
    rviz_waypoints_creater.process();
    return 0;
}
