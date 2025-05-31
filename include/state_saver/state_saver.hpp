#include <array>
#include <cmath>
#include <iostream>
#include <vector>
#include <iostream>
#include <fstream>

#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "interfaces/srv/get_target.hpp"
#include "interfaces/srv/save_status.hpp"

#include "common/common.hpp"

#ifndef SaveStatus_H
#define SaveStatus_H

class SaveStatusNode: public rclcpp::Node
{
public:
	SaveStatusNode();
	~SaveStatusNode();

private:
  // ros interface
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

  rclcpp::Service<interfaces::srv::SaveStatus>::SharedPtr srv_save_status_;

  // input callback
  void cb_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_in);
  void cb_odom(const nav_msgs::msg::Odometry::SharedPtr msg_in); 
  void cb_path(const nav_msgs::msg::Path::SharedPtr msg_in);
  void cb_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg_in);
  void imageCallBack(const sensor_msgs::msg::CompressedImage::SharedPtr msg_in);

  void save_status_server(const std::shared_ptr<interfaces::srv::SaveStatus::Request> request,
    std::shared_ptr<interfaces::srv::SaveStatus::Response> response);

  std::shared_ptr<tf2_ros::Buffer> tfBuffer;  
  std::shared_ptr<tf2_ros::TransformListener> listener;
  bool save_odom_{true};  // i
  bool save_cam_pose_{true};  // i
  bool save_cost_map_{true};  // i
  bool save_path_{true};  // i
  bool save_image_{true};  // i
  bool save_scan_{true};  // i

  nav_msgs::msg::OccupancyGrid cost_map_{};
  nav_msgs::msg::Odometry odom_{};
  nav_msgs::msg::Path path_{};
  sensor_msgs::msg::LaserScan scan_{};
  cv::Mat image_{};

  Common common_{};              // tools cone finder node 
};

#endif