#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "state_saver/state_saver.hpp"
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

SaveStatusNode::SaveStatusNode()
    : Node("state_server")
{
  // ros interfaaces
  sub_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&SaveStatusNode::cb_costmap, this, _1));
  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&SaveStatusNode::cb_odom, this, _1));
  sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&SaveStatusNode::cb_path, this, _1));
  sub_image_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/compressed", 10, std::bind(&SaveStatusNode::imageCallBack, this, _1));    
  sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(), std::bind(&SaveStatusNode::cb_scan, this, _1));

  srv_save_status_ = this->create_service<interfaces::srv::SaveStatus>(
      "/save_status", std::bind(&SaveStatusNode::save_status_server, this, _1, _2));

  tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface());

  tfBuffer->setCreateTimerInterface(timer_interface);
  listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

  // parameters
  this->declare_parameter("save_odom_", true);
  this->declare_parameter("save_cam_pose_", true);
  this->declare_parameter("save_cost_map_", true);
  this->declare_parameter("save_path_", true);
  this->declare_parameter("save_image_", true);
  this->declare_parameter("save_scan_", true);

  save_odom_ = this->get_parameter("save_odom_").as_bool();
  save_cam_pose_ = this->get_parameter("save_cam_pose_").as_bool();
  save_cost_map_ = this->get_parameter("save_cost_map_").as_bool();
  save_path_ = this->get_parameter("save_path_").as_bool();
  save_image_ = this->get_parameter("save_image_").as_bool();
  save_scan_ = this->get_parameter("save_scan_").as_bool();
}

SaveStatusNode::~SaveStatusNode()
{
}

void SaveStatusNode::cb_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_in)
{
  cost_map_ = *msg_in;
}

void SaveStatusNode::cb_odom(const nav_msgs::msg::Odometry::SharedPtr msg_in)
{
  odom_ = *msg_in;
}

void SaveStatusNode::cb_path(const nav_msgs::msg::Path::SharedPtr msg_in)
{
  path_ = *msg_in;
}

void SaveStatusNode::cb_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg_in)
{
  scan_ = *msg_in;
}

void SaveStatusNode::imageCallBack(const sensor_msgs::msg::CompressedImage::SharedPtr msg_in)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg_in, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    return;
  }
  image_ =cv_ptr->image;
}

void SaveStatusNode::save_status_server(const std::shared_ptr<interfaces::srv::SaveStatus::Request> request,
                                            std::shared_ptr<interfaces::srv::SaveStatus::Response> response)
{
  bool result{false};
  if (save_odom_)
  {
      result &= common_.WritePoseToYaml(request->name+ "_pose.yaml", odom_.pose.pose);
  }
  if (save_cam_pose_)
  {
    geometry_msgs::msg::TransformStamped transformCamera;
    transformCamera = tfBuffer->lookupTransform("map", "cam_frame", rclcpp::Time(0) );
    geometry_msgs::msg::Pose bot_camera{};
    geometry_msgs::msg::Pose bot_camera_tf{};
    tf2::doTransform(bot_camera, bot_camera_tf, transformCamera); // get the pose of the camera
    result &= common_.WritePoseToYaml(request->name+ "_pose_cam.yaml", bot_camera_tf);
  }
  if (save_cost_map_)
  {
      result &= common_.WriteMapToYaml(request->name+ "_map.yaml", cost_map_);
  }
  if (save_path_)
  {
      result &= common_.WritePathToYaml(request->name+ "_path.yaml", path_);
  }
  if (save_image_)
  {
      result &= common_.WriteImage(request->name+ "_img.jpg", image_);
  }
  if (save_scan_)
  {
      // result &= common_.WriteScan(request->name+ "_scan.yaml", scan_);
  }

  if (result)
  {    
    // RCLCPP_INFO(this->get_logger(), " founded !!");
    response->result = true;
  }
  else
  {
    response->result = false;
  }  
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SaveStatusNode>());
  rclcpp::shutdown();
  return 0;
}