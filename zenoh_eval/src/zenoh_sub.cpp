#include <memory>

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

      RCLCPP_INFO(this->get_logger(), "time in msg: %f secs", msg_time);
      RCLCPP_INFO(this->get_logger(), "time now: %f secs", time_now);
    }

    void image_rect_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
      double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
      double time_now = this->get_clock()->now().seconds();

      // RCLCPP_INFO(this->get_logger(), "time in msg: %f secs", msg_time);
      // RCLCPP_INFO(this->get_logger(), "time now: %f secs", time_now);
    }

    void image_raw_comp_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) 
    {
      double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
      double time_now = this->get_clock()->now().seconds();

      // RCLCPP_INFO(this->get_logger(), "time in msg: %f secs", msg_time);
      // RCLCPP_INFO(this->get_logger(), "time now: %f secs", time_now);
    }

    void image_stereo_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
      double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
      double time_now = this->get_clock()->now().seconds();

      // RCLCPP_INFO(this->get_logger(), "time in msg: %f secs", msg_time);
      // RCLCPP_INFO(this->get_logger(), "time now: %f secs", time_now);
    }

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
    {
      double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
      double time_now = this->get_clock()->now().seconds();

      // RCLCPP_INFO(this->get_logger(), "time in msg: %f secs", msg_time);
      // RCLCPP_INFO(this->get_logger(), "time now: %f secs", time_now);
    }

    void costmap_raw_callback(const nav2_msgs::msg::Costmap::SharedPtr msg) 
    {
      double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
      double time_now = this->get_clock()->now().seconds();

      // RCLCPP_INFO(this->get_logger(), "time in msg: %f secs", msg_time);
      // RCLCPP_INFO(this->get_logger(), "time now: %f secs", time_now);
    }

    void costmap_updates_callback(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) 
    {
      double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
      double time_now = this->get_clock()->now().seconds();

      // RCLCPP_INFO(this->get_logger(), "time in msg: %f secs", msg_time);
      // RCLCPP_INFO(this->get_logger(), "time now: %f secs", time_now);
    }
    
  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_rect_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_raw_comp_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_stereo_sub_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_raw_sub_;
    rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_updates_sub_;
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