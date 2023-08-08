#include "../include/slamware_ros2/slamware_node.hpp"


SlamwareNode::SlamwareNode() : Node("slamware_node")
{

  rpos_platform = rpos::robot_platforms::SlamwareCorePlatform::connect("172.16.44.240", 1445);
  map_holder = slamware_ros_sdk::ServerMapHolder();

  clock_    = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  scan_msg_ = std::make_shared<sensor_msgs::msg::LaserScan>();
  map_msg_  = std::make_shared<nav_msgs::msg::OccupancyGrid>();

  laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
  map_pub_        = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&SlamwareNode::publish_laser_scan, this));
}

void SlamwareNode::publish_laser_scan()
{
  // RCLCPP_INFO(this->get_logger(), "Publishing Laser Scan");

  rclcpp::Time start_scan_time = clock_->now();
  rpos_laser_scan = rpos_platform.getLaserScan();
  laser_points    = rpos_laser_scan.getLaserPoints();
  rclcpp::Time end_scan_time = clock_->now();

  double scan_duration = (end_scan_time - start_scan_time).seconds();

  scan_msg_->header.stamp = start_scan_time;
  scan_msg_->header.frame_id = "base_link";
  fillRangeMinMaxInMsg_(laser_points, scan_msg_);

  scan_msg_->ranges.resize(laser_points.size());

  for (size_t i = 0; i < laser_points.size(); ++i)
  {
      if (!laser_points[i].valid())
      {
          scan_msg_->ranges[i] = std::numeric_limits<float>::infinity();
      }
      else
      {
          scan_msg_->ranges[i] = laser_points[i].distance();
      }
  }

  scan_msg_->angle_min =  laser_points.front().angle();
  scan_msg_->angle_max =  laser_points.back().angle();
  scan_msg_->angle_increment = (scan_msg_->angle_max - scan_msg_->angle_min) / (double)(scan_msg_->ranges.size() - 1);
  scan_msg_->scan_time = scan_duration;
  scan_msg_->time_increment = scan_duration / (double)(scan_msg_->ranges.size() - 1);

  laser_pose = rpos_laser_scan.getLaserPointsPose();
  std::cout << "x: " << laser_pose.x() << " y: " << laser_pose.y() << " yaw: " << laser_pose.yaw() << std::endl;

  laser_scan_pub_->publish(*scan_msg_);
  broadcastMap2Laser();
  getMapDataRos();
}

void SlamwareNode::getMapDataRos(){

  rpos::core::RectangleF knownArea = rpos_platform.getKnownArea(
    rpos::features::location_provider::MapTypeBitmap8Bit, rpos::features::location_provider::EXPLORERMAP
  );

  rpos::features::location_provider::Map map = rpos_platform.getMap(
    rpos::features::location_provider::MapTypeBitmap8Bit, knownArea, rpos::features::location_provider::EXPLORERMAP
  );

  map_msg_->header.stamp = clock_->now();
  map_msg_->header.frame_id = "map";
  map_holder.ServerMapHolder::fillRosMapMsg(knownArea, *map_msg_);
  map_holder.setMapData(map);

  map_pub_->publish(*map_msg_);

  // auto map_data = map.getMapData();
  // for(auto data : map_data){
  //   RCLCPP_INFO(this->get_logger(), "Map Data: %d", data);
  // }
  // RCLCPP_INFO(this->get_logger(), "===========================================");

  
}

void SlamwareNode::broadcastMap2Laser(){
  
  transform_stamped.header.stamp = clock_->now();
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = "base_link";
  transform_stamped.transform.translation.x = laser_pose.x();
  transform_stamped.transform.translation.y = laser_pose.y();
  transform_stamped.transform.translation.z = 0.0;
  q.setRPY(0, 0, laser_pose.yaw());

  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  transform_stamped.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(transform_stamped);

}

void SlamwareNode::fillRangeMinMaxInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
            , sensor_msgs::msg::LaserScan::SharedPtr& msgScan
            ) const
{
    msgScan->range_min = std::numeric_limits<float>::infinity();
    msgScan->range_max = 0.0f;
    for (auto cit = laserPoints.cbegin(), citEnd = laserPoints.cend(); citEnd != cit; ++cit)
    {
        if (cit->valid())
        {
            const float tmpDist = cit->distance();
            
            if (tmpDist < msgScan->range_min)
            {
                msgScan->range_min = std::max<float>(0.0f, tmpDist);
            }

            if (msgScan->range_max < tmpDist)
            {
                msgScan->range_max = tmpDist;
            }
        }
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamwareNode>());
  rclcpp::shutdown();
  return 0;
}