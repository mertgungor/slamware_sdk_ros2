#ifndef SLAMWARE_NODE_HPP_
#define SLAMWARE_NODE_HPP_
// #pragma warning(disable: 4345) // disable warning 4345

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "server_map_holder.hpp"


#include <rpos/features/statistics_provider.h>
#include <rpos/robot_platforms/slamware_core_platform.h>
#include <rpos/features/system_resource/laser_scan.h>
#include <rpos/robot_platforms/slamware_core_platform.h>
#include <rpos/core/laser_point.h>


class SlamwareNode : public rclcpp::Node
{
public:
  SlamwareNode();
  void publish_laser_scan();
  void fillRangeMinMaxInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
            , sensor_msgs::msg::LaserScan::SharedPtr& msgScan
            ) const;

  void getMapDataRos();
  void broadcastMap2Laser();

private:
    
    sensor_msgs::msg::LaserScan::SharedPtr  scan_msg_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    nav_msgs::msg::Odometry::SharedPtr      odom_msg_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock::SharedPtr clock_;
    geometry_msgs::msg::TransformStamped transform_stamped;
    
    rpos::features::system_resource::LaserScan  rpos_laser_scan;
    rpos::robot_platforms::SlamwareCorePlatform rpos_platform;
    rpos::core::Pose laser_pose;
    std::vector<rpos::core::LaserPoint> laser_points;

    slamware_ros_sdk::ServerMapHolder map_holder;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2::Quaternion q;
    };


#endif // SLAMWARE_NODE_HPP_
