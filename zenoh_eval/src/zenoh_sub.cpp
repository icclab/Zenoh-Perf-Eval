#include <memory>
#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"

using std::placeholders::_1;

class ZenohSubscriber : public rclcpp::Node
{
  public:
    ZenohSubscriber()
    : Node("ZenohSubscriber")
    {
      double file_time = this->get_clock()->now().seconds();
      std::string time_str = std::to_string(file_time);

      image_raw_.open(time_str + "_oak_image_raw.csv");
      image_raw_ << "msg_published_time" << " " << "msg_received_time" << " " << "uint32_height" << " " << "uint32_width" << " " << "uint32_step" << " " << "uint8_data_size.bytes" << std::endl;

      image_rect_.open(time_str + "_oak_image_rect.csv");
      image_rect_ << "msg_published_time" << " " << "msg_received_time" << " " << "uint32_height" << " " << "uint32_width" << " " << "uint32_step" << " " << "uint8_data_size.bytes" << std::endl;

      image_raw_comp_.open(time_str + "_oak_image_raw_comp.csv");
      image_raw_comp_ << "msg_published_time" << " " << "msg_received_time" << " " << "uint8_data_size.bytes" << std::endl;

      image_stereo_.open(time_str + "_oak_image_stereo.csv");
      image_stereo_ << "msg_published_time" << " " << "msg_received_time" << " " << "uint32_height" << " " << "uint32_width" << " " << "uint32_step" << " " << "uint8_data_size.bytes" << std::endl; 

      costmap_.open(time_str + "_costmap.csv");
      costmap_ << "msg_published_time" << " " << "msg_received_time" << " " << "info.width" << " " << "info.height" << " " << "int8_data_size.bytes" << std::endl; 

      costmap_raw_.open(time_str + "_costmap_raw.csv");
      costmap_raw_ << "msg_published_time" << " " << "msg_received_time" << " " << "map_load_time.secs" << " " << "update_time.secs" << " " << "uint32_metadata.size_x" << " " << "uint32_metadata.size_y" << " " << "uint8_data_size.bytes" << std::endl;   

      costmap_updates_.open(time_str + "_costmap_updates.csv");
      costmap_updates_ << "msg_published_time" << " " << "msg_received_time" << " " << "uint32_width" << " " << "uint32_height" << " " << "int8_data_size.bytes" << std::endl; 

      rclcpp::SubscriptionOptions sub_options;
      sub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

      image_raw_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/summit/oak/rgb/image_raw", rclcpp::QoS(10), std::bind(&ZenohSubscriber::image_raw_callback, this, _1), sub_options);
      image_rect_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/summit/oak/rgb/image_rect", rclcpp::QoS(10), std::bind(&ZenohSubscriber::image_rect_callback, this, _1), sub_options);
      image_raw_comp_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("/summit/oak/rgb/image_raw/compressed", rclcpp::QoS(10), std::bind(&ZenohSubscriber::image_raw_comp_callback, this, _1), sub_options);
      image_stereo_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/summit/oak/stereo/image_raw", rclcpp::QoS(10), std::bind(&ZenohSubscriber::image_stereo_callback, this, _1), sub_options);

      auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

      costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/summit/local_costmap/costmap", custom_qos, std::bind(&ZenohSubscriber::costmap_callback, this, _1));
      costmap_raw_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>("/summit/local_costmap/costmap_raw", custom_qos, std::bind(&ZenohSubscriber::costmap_raw_callback, this, _1));
      costmap_updates_sub_ = this->create_subscription<map_msgs::msg::OccupancyGridUpdate>("/summit/local_costmap/costmap_updates", custom_qos, std::bind(&ZenohSubscriber::costmap_updates_callback, this, _1));
    }

    void image_raw_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
      double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
      double time_now = this->get_clock()->now().seconds();

      image_raw_ << std::fixed << std::setprecision(3) << msg_time << " " << time_now << " " << msg->height << " " << msg->width << " " << msg->step << " " << msg->data.size() << std::endl;

      // RCLCPP_INFO(this->get_logger(), "time in msg: %f secs", msg_time);
      // RCLCPP_INFO(this->get_logger(), "time now: %f secs", time_now);
    }

    void image_rect_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
      double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
      double time_now = this->get_clock()->now().seconds();

      image_rect_ << std::fixed << std::setprecision(3) << msg_time << " " << time_now << " " << msg->height << " " << msg->width << " " << msg->step << " " << msg->data.size() << std::endl;

      // RCLCPP_INFO(this->get_logger(), "time in msg: %f secs", msg_time);
      // RCLCPP_INFO(this->get_logger(), "time now: %f secs", time_now);
    }

    void image_raw_comp_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) 
    {
      double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
      double time_now = this->get_clock()->now().seconds();

      image_raw_comp_ << std::fixed << std::setprecision(3) << msg_time << " " << time_now << " " << msg->data.size() << std::endl;

      // RCLCPP_INFO(this->get_logger(), "time in msg: %f secs", msg_time);
      // RCLCPP_INFO(this->get_logger(), "time now: %f secs", time_now);
    }

    void image_stereo_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
      double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
      double time_now = this->get_clock()->now().seconds();

      image_stereo_ << std::fixed << std::setprecision(3) << msg_time << " " << time_now << " " << msg->height << " " << msg->width << " " << msg->step << " " << msg->data.size() << std::endl;

      // RCLCPP_INFO(this->get_logger(), "time in msg: %f secs", msg_time);
      // RCLCPP_INFO(this->get_logger(), "time now: %f secs", time_now);
    }

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
    {
      double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
      double time_now = this->get_clock()->now().seconds();

      costmap_ << std::fixed << std::setprecision(3) << msg_time << " " << time_now << " " << msg->info.width << " " << msg->info.height << " " <<  msg->data.size() << std::endl;

      // RCLCPP_INFO(this->get_logger(), "time in msg: %f secs", msg_time);
      // RCLCPP_INFO(this->get_logger(), "time now: %f secs", time_now);
    }

    void costmap_raw_callback(const nav2_msgs::msg::Costmap::SharedPtr msg) 
    {
      double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
      double time_now = this->get_clock()->now().seconds();

      costmap_raw_ << std::fixed << std::setprecision(3) << msg_time << " " << time_now << " " << msg->metadata.map_load_time.sec + (1e-9 * msg->metadata.map_load_time.nanosec) << " " << msg->metadata.update_time.sec + (1e-9 * msg->metadata.update_time.nanosec) << " "<< msg->metadata.size_x << " " << msg->metadata.size_y << " " << msg->data.size() << std::endl;

      // RCLCPP_INFO(this->get_logger(), "time in msg: %f secs", msg_time);
      // RCLCPP_INFO(this->get_logger(), "time now: %f secs", time_now);
    }

    void costmap_updates_callback(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) 
    {
      double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
      double time_now = this->get_clock()->now().seconds();

      costmap_updates_ << std::fixed << std::setprecision(3) << msg_time << " " << time_now << " " << msg->width << " " << msg->height << " " << msg->data.size() << std::endl;

      // RCLCPP_INFO(this->get_logger(), "time in msg: %f secs", msg_time);
      // RCLCPP_INFO(this->get_logger(), "time now: %f secs", time_now);
    }

    ~ZenohSubscriber()
    {
      image_raw_.close();
      image_rect_.close();
      image_raw_comp_.close();
      image_stereo_.close();
      costmap_.close();
      costmap_raw_.close();
      costmap_updates_.close();
    }
    
  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_rect_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_raw_comp_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_stereo_sub_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_raw_sub_;
    rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_updates_sub_;

    std::ofstream image_raw_;
    std::ofstream image_rect_;
    std::ofstream image_raw_comp_;
    std::ofstream image_stereo_;
    std::ofstream costmap_;
    std::ofstream costmap_raw_;
    std::ofstream costmap_updates_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ZenohSubscriber>();

  // auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  // executor->add_node(node);

  // executor->spin();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
