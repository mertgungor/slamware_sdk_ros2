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

  timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&SlamwareNode::publish_laser_scan, this));
}

void SlamwareNode::publish_laser_scan()
{
  // RCLCPP_INFO(this->get_logger(), "Publishing Laser Scan");

  rclcpp::Time start_scan_time = clock_->now();
  laser_points = rpos_platform.getLaserScan().getLaserPoints();
  rclcpp::Time end_scan_time = clock_->now();

  double scan_duration = (end_scan_time - start_scan_time).seconds();

  scan_msg_->header.stamp = start_scan_time;
  scan_msg_->header.frame_id = "laser_frame";
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

  laser_scan_pub_->publish(*scan_msg_);
  getMapDataRos();
}

void SlamwareNode::getMapDataRos(){

  rpos::core::RectangleF knownArea = rpos_platform.getKnownArea(
    rpos::features::location_provider::MapTypeBitmap8Bit, rpos::features::location_provider::EXPLORERMAP
  );

  rpos::features::location_provider::Map map = rpos_platform.getMap(
    rpos::features::location_provider::MapTypeBitmap8Bit, knownArea, rpos::features::location_provider::EXPLORERMAP
  );

  // RCLCPP_INFO(this->get_logger(), "Map Area: %f", map.getMapArea().area());
  // map_msg_->header.stamp = clock_->now();
  // map_msg_->header.frame_id = "map";
  // map_msg_->info.resolution = map.getMapResolution().x();
  // map_msg_->info.width = knownArea.width();
  // map_msg_->info.height = knownArea.height();
  // map_msg_->info.origin.position.x = map.getMapArea().left();
  // map_msg_->info.origin.position.y = map.getMapArea().top();
  // map_msg_->info.origin.position.z = 0.0;
  // map_msg_->info.origin.orientation.x = 0.0;
  // map_msg_->info.origin.orientation.y = 0.0;
  // map_msg_->info.origin.orientation.z = 0.0;
  // map_msg_->info.origin.orientation.w = 1.0;

  // map_msg_->data.resize(map.getMapData().width() * map.getMapData().height());

  
  map_msg_->header.stamp = clock_->now();
  map_msg_->header.frame_id = "map";
  map_holder.ServerMapHolder::fillRosMapMsg(knownArea, *map_msg_);
  map_holder.setMapData(map);

  map_pub_->publish(*map_msg_);

  auto map_data = map.getMapData();
  for(auto data : map_data){
    RCLCPP_INFO(this->get_logger(), "Map Data: %d", data);
  }
  RCLCPP_INFO(this->get_logger(), "===========================================");

  
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