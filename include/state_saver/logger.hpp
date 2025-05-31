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
#include <sensor_msgs/msg/imu.hpp>
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

class DataLogger {
public:
    DataLogger() {
        
    }

    void init(const std::string& filename){
      filename_ = filename;
        // Open the file and write the header
      std::ofstream file(filename_);
        if (file.is_open()) {
            file << "Timestamp,x_gyro,y_gyro,z_gyro,x_accel,y_accel,z_accel\n";
            file.close();
        } else {
            std::cerr << "Unable to open file: " << filename_ << std::endl;
        }
    }

    void logData(int64_t time, double x_gyro, double y_gyro,double z_gyro, double x_accel,double y_accel, double z_accel) {
        // Get the current timestamp
        // auto now = std::time(nullptr);
        // auto tm_info = std::localtime(&now);
        // std::stringstream timestamp;
        // timestamp << std::put_time(tm_info, "%Y-%m-%d %H:%M:%S");

        // Append the data to the file
        std::ofstream file(filename_, std::ios_base::app);
        if (file.is_open()) {
            file << time << "," << x_gyro << "," << y_gyro << "," << z_gyro<< "," << x_accel<< "," << y_accel<< "," << z_accel<< "\n";
            file.close();
        } else {
            std::cerr << "Unable to open file: " << filename_ << std::endl;
        }
    }

private:
    std::string filename_;
};

class LoggerNode: public rclcpp::Node
{
public:
	LoggerNode();
	~LoggerNode();

private:
  DataLogger logger_;
  // ros interface
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();
  // input callback
  void cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg_in);
  sensor_msgs::msg::Imu imu_{};
  
  Common common_{};              // tools cone finder node 
  std::string file_name_;
};

#endif