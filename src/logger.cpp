#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "state_saver/logger.hpp"
#include <cv_bridge/cv_bridge.hpp>               // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp>                  // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include "common/common.hpp"
#include <cmath>
#include <boost/math/special_functions/round.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

LoggerNode::LoggerNode()
    : Node("logger")
{
  // ros interfaaces
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&LoggerNode::cb_imu, this, _1));
  
  timer_ = this->create_wall_timer(
      50ms, std::bind(&LoggerNode::timer_callback, this));

  // parameters
  this->declare_parameter("file_name", "/home/davide-work/humble_ws/wheele/csv_logs/t0.csv");

  file_name_ = this->get_parameter("file_name").as_string();
  logger_.init(file_name_);
}

LoggerNode::~LoggerNode()
{
}

void LoggerNode::cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg_in)
{
  imu_ = *msg_in;
}

void LoggerNode::timer_callback()
{
  rclcpp::Time now = this->get_clock()->now();
        
  // Convert nanoseconds to milliseconds
  int64_t millis = now.nanoseconds() / 1'000'000;
  logger_.logData(millis, 
  imu_.orientation.x, imu_.orientation.y, imu_.orientation.z,
  imu_.linear_acceleration.x, imu_.linear_acceleration.y, imu_.linear_acceleration.z);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoggerNode>());
  rclcpp::shutdown();
  return 0;
}