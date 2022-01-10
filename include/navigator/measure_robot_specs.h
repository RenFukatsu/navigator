#ifndef MEASURE_ROBOT_SPECS_H_
#define MEASURE_ROBOT_SPECS_H_

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "roomba_500driver_meiji/RoombaCtrl.h"

class MeasureRobotSpecs {
 public:
    MeasureRobotSpecs();
    void process();
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose);

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::string filename;
};

#endif  // MEASURE_ROBOT_SPECS_H_
