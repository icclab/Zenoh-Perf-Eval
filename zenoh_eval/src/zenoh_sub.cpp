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
#include "std_msgs/msg/bool.hpp"

#include <ctime>
#include <sstream>

using std::placeholders::_1;
using namespace std::chrono_literals;

class ZenohSubscriber : public rclcpp::Node
{
  public:
    ZenohSubscriber()
    : Node("ZenohSubscriber")
    {
      this->declare_parameter("filename_append", "test");
      filename_append_ = this->get_parameter("filename_append").as_string();

      this->declare_parameter("oak_camera_topic", "stereo/image_raw_throttle");
      oak_camera_topic_ = this->get_parameter("oak_camera_topic").as_string();

      double file_time = this->get_clock()->now().seconds();

      std::time_t file_time_int = static_cast<std::time_t>(file_time);

      std::tm* time_info = std::localtime(&file_time_int);

      std::stringstream time_stream;
      time_stream << std::put_time(time_info, "%d-%m-%Y_%H-%M-%S");

      std::string time_str = time_stream.str();

      image_raw_.open(filename_append_ + "_" + time_str + "_oak_image_raw.csv");
      image_raw_ << "msg_published_time" << " " << "msg_received_time" << " " << "uint32_height" << " " << "uint32_width" << " " << "uint32_step" << " " << "uint8_data_size.bytes" << " " << "latency_secs" << std::endl;

      // image_rect_.open(filename_append_ + "_" + time_str + "_oak_image_rect.csv");
      // image_rect_ << "msg_published_time" << " " << "msg_received_time" << " " << "uint32_height" << " " << "uint32_width" << " " << "uint32_step" << " " << "uint8_data_size.bytes" << " " << "latency_secs" << std::endl;

      // image_raw_comp_.open(filename_append_ + "_" + time_str + "_oak_image_raw_comp.csv");
      // image_raw_comp_ << "msg_published_time" << " " << "msg_received_time" << " " << "uint8_data_size.bytes" << " " << "latency_secs" << std::endl;

      // image_stereo_.open(filename_append_ + "_" + time_str + "_oak_image_stereo.csv");
      // image_stereo_ << "msg_published_time" << " " << "msg_received_time" << " " << "uint32_height" << " " << "uint32_width" << " " << "uint32_step" << " " << "uint8_data_size.bytes" << " " << "latency_secs" << std::endl;

      costmap_.open(filename_append_ + "_" + time_str + "_costmap.csv");
      costmap_ << "msg_published_time" << " " << "msg_received_time" << " " << "info.width" << " " << "info.height" << " " << "int8_data_size.bytes" << " " << "latency_secs" << std::endl;

      // costmap_raw_.open(filename_append_ + "_" + time_str + "_costmap_raw.csv");
      // costmap_raw_ << "msg_published_time" << " " << "msg_received_time" << " " << "map_load_time.secs" << " " << "update_time.secs" << " " << "uint32_metadata.size_x" << " " << "uint32_metadata.size_y" << " " << "uint8_data_size.bytes" << " " << "latency_secs" << std::endl;   

      evaluations_.open(filename_append_ + "_" + time_str + "_evaluations.csv");
      evaluations_ << filename_append_ << " " << "msg_count" << " " << "evaluation_time_secs" << " " << "median_latency_secs" << " " << "mean_latency_secs" << " " << "msg_freq_Hz" << " " << "throughput_MB_p_sec" << " " << "mean_data_size_B" << std::endl;

      rclcpp::SubscriptionOptions sub_options;
      sub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

      image_raw_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/summit/oak/" + oak_camera_topic_, rclcpp::QoS(10), std::bind(&ZenohSubscriber::image_raw_callback, this, _1), sub_options);
      // image_rect_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/summit/oak/rgb/image_rect", rclcpp::QoS(10), std::bind(&ZenohSubscriber::image_rect_callback, this, _1), sub_options);
      // image_raw_comp_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("/summit/oak/rgb/image_raw/compressed", rclcpp::QoS(10), std::bind(&ZenohSubscriber::image_raw_comp_callback, this, _1), sub_options);
      // image_stereo_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/summit/oak/stereo/image_raw", rclcpp::QoS(10), std::bind(&ZenohSubscriber::image_stereo_callback, this, _1), sub_options);

      auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

      costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/summit/local_costmap/costmap", custom_qos, std::bind(&ZenohSubscriber::costmap_callback, this, _1));
      // costmap_raw_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>("/summit/local_costmap/costmap_raw", custom_qos, std::bind(&ZenohSubscriber::costmap_raw_callback, this, _1));

      start_script_sub_ = this->create_subscription<std_msgs::msg::Bool>("/summit/start_zenoh_script", custom_qos, std::bind(&ZenohSubscriber::start_script_callback, this, _1)); 
    }

    void timer_callback()
    {
      RCLCPP_INFO(this->get_logger(), "5 mins finished, we can kill this node now");

      end_script_image_raw_ = true;
      // end_script_image_rect_ = true;
      // end_script_image_stereo_ = true;
      // end_script_image_raw_comp_ = true;
      end_script_costmap_ = true;
      // end_script_costmap_raw_ = true;
    }
    
    void start_script_callback(const std_msgs::msg::Bool::SharedPtr msg) 
    {
      start_script_image_raw_ = msg->data;
      // start_script_image_rect_ = msg->data;
      // start_script_image_stereo_ = msg->data;
      // start_script_image_raw_comp_ = msg->data;
      start_script_costmap_ = msg->data;
      // start_script_costmap_raw_ = msg->data;

      if (start_script_image_raw_ && flag_)
      {   
        RCLCPP_INFO(this->get_logger(), "starting data recording");

        timer_ = this->create_wall_timer(5min, std::bind(&ZenohSubscriber::timer_callback, this));
        flag_ = false;
        get_start_time_image_raw_ = true;
        // get_start_time_image_rect_ = true;
        // get_start_time_image_stereo_ = true;
        // get_start_time_image_raw_comp_ = true;
        get_start_time_costmap_ = true;
        // get_start_time_costmap_raw_ = true;
      }
    }

    double calculate_median(std::vector<double> count_vector_)
    {
      double median = -1.0;

      std::sort(count_vector_.begin(), count_vector_.end());
                
      if (count_vector_.size() % 2 == 0)
      {
        median = ( count_vector_.at(count_vector_.size() / 2 - 1) + count_vector_.at(count_vector_.size() / 2) ) / 2.0;
      }

      else
      {
        median = count_vector_.at((count_vector_.size() - 1) / 2);
      }

      return median;
    }

    void image_raw_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
      if (start_script_image_raw_ && flag_1)
      {   
        double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
        double time_now = this->get_clock()->now().seconds();

        if (get_start_time_image_raw_)
        {
          first_msg_time_image_raw_ = msg_time; 
          start_time_image_raw_ = time_now;
          get_start_time_image_raw_ = false;
          // return;
        }

        // time_now += first_msg_time_image_raw_ - start_time_image_raw_; 

        double latency = time_now - msg_time;

        count_image_raw_vector_.push_back(latency);

        image_raw_ << std::fixed << std::setprecision(6) << msg_time << " " << time_now << " " << msg->height << " " << msg->width << " " << msg->step << " " << msg->data.size() << " " << latency << std::endl;
        
        count_image_raw_++;
        latency_mean_image_raw_ += latency;

        if (end_script_image_raw_)
        {
          // double total_time = time_now - start_time_image_raw_ - (first_msg_time_image_raw_ - start_time_image_raw_);
          double total_time = time_now - start_time_image_raw_;

          double msg_freq = count_image_raw_ / total_time;
          double throughput = 1e-6 * msg->data.size() * msg_freq;

          double median = -1.0;
          median = calculate_median(count_image_raw_vector_);

          image_raw_ << std::endl;
          image_raw_ << filename_append_ << " " << "msg_count" << " " << "evaluation_time_secs" << " " << "median_latency_secs" << " " << "mean_latency_secs" << " " << "msg_freq_Hz" << " " << "throughput_MB_p_sec" << std::endl;
          image_raw_ << std::fixed << std::setprecision(6) << "oak/rgb/image_raw" << " " << count_image_raw_ << " " << total_time << " " << median << " " << latency_mean_image_raw_ / count_image_raw_ << " " << msg_freq << " " << throughput << std::endl; 
          evaluations_ << std::fixed << std::setprecision(6) << "oak/rgb/image_raw" << " " << count_image_raw_ << " " << total_time << " " << median << " " << latency_mean_image_raw_ / count_image_raw_ << " " << msg_freq << " " << throughput << std::endl;
          // image_raw_.close();
          flag_1 = false;
          end_script_image_raw_ = false;
          start_time_image_raw_ = -1.0;
          count_image_raw_ = 0;
          start_script_image_raw_ = false;
          latency_mean_image_raw_ = 0.0;
          count_image_raw_vector_.clear();
        }
      }
    }

    // void image_rect_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
    // {
    //   if (start_script_image_rect_ && flag_2)
    //   {  
    //     double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
    //     double time_now = this->get_clock()->now().seconds();

    //     if (get_start_time_image_rect_)
    //     {
    //       first_msg_time_image_rect_ = msg_time;
    //       start_time_image_rect_ = time_now;
    //       get_start_time_image_rect_ = false;
    //       // return;
    //     }

    //     // time_now += first_msg_time_image_rect_ - start_time_image_rect_; 
        
    //     double latency = time_now - msg_time;

    //     count_image_rect_vector_.push_back(latency);

    //     image_rect_ << std::fixed << std::setprecision(6) << msg_time << " " << time_now << " " << msg->height << " " << msg->width << " " << msg->step << " " << msg->data.size() << " " << latency << std::endl;
        
    //     count_image_rect_++;
    //     latency_mean_image_rect_ += latency;

    //     if (end_script_image_rect_)
    //     {
    //       double total_time = time_now - start_time_image_rect_;
    //       double msg_freq = count_image_rect_ / total_time;
    //       double throughput = 1e-6 * msg->data.size() * msg_freq;

    //       double median = -1.0;
    //       median = calculate_median(count_image_rect_vector_);

    //       image_rect_ << std::endl;
    //       image_rect_ << filename_append_ << " " << "msg_count" << " " << "evaluation_time_secs" << " " << "median_latency_secs" << " " << "mean_latency_secs" << " " << "msg_freq_Hz" << " " << "throughput_MB_p_sec" << std::endl;
    //       image_rect_ << std::fixed << std::setprecision(6) << "oak/rgb/image_rect" << " " << count_image_rect_ << " " << total_time << " " << median << " " << latency_mean_image_rect_ / count_image_rect_ << " " << msg_freq << " " << throughput << std::endl; 
    //       evaluations_ << std::fixed << std::setprecision(6) << "oak/rgb/image_rect" << " " << count_image_rect_ << " " << total_time << " " << median << " " << latency_mean_image_rect_ / count_image_rect_ << " " << msg_freq << " " << throughput << std::endl; 
    //       // image_rect_.close();
    //       flag_2 = false;
    //       start_time_image_rect_ = -1.0;
    //       count_image_rect_ = 0;
    //       end_script_image_rect_ = false;
    //       latency_mean_image_rect_ = 0.0;
    //       count_image_rect_vector_.clear();
    //     }
    //   }
    // }

    // void image_raw_comp_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) 
    // {
    //   if (start_script_image_raw_comp_ && flag_3)
    //   { 
    //     double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
    //     double time_now = this->get_clock()->now().seconds();

    //     if (get_start_time_image_raw_comp_)
    //     {
    //       first_msg_time_image_raw_comp_ = msg_time;
    //       start_time_image_raw_comp_ = time_now;
    //       get_start_time_image_raw_comp_ = false;
    //       // return;
    //     }
        
    //     // time_now += first_msg_time_image_raw_comp_ - start_time_image_raw_comp_; 

    //     double latency = time_now - msg_time;

    //     count_image_raw_comp_vector_.push_back(latency);

    //     data_image_raw_comp_ += msg->data.size();

    //     image_raw_comp_ << std::fixed << std::setprecision(6) << msg_time << " " << time_now << " " << msg->data.size() << " " << latency << std::endl;
        
    //     count_image_raw_comp_++;
    //     latency_mean_image_raw_comp_ += latency;

    //     if (end_script_image_raw_comp_)
    //     {
    //       double total_time = time_now - start_time_image_raw_comp_;
    //       double msg_freq = count_image_raw_comp_ / total_time;
    //       long double throughput = 1e-6 * data_image_raw_comp_ / total_time;

    //       double median = -1.0;
    //       median = calculate_median(count_image_raw_comp_vector_);

    //       image_raw_comp_ << std::endl;
    //       image_raw_comp_ << filename_append_ << " " << "msg_count" << " " << "evaluation_time_secs" << " " << "median_latency_secs" << " " << "mean_latency_secs" << " " << "msg_freq_Hz" << " " << "throughput_MB_p_sec" << " " << "mean_data_size_B" << std::endl;
    //       image_raw_comp_ << std::fixed << std::setprecision(6) << "oak/rgb/image_raw/compressed" << " " << count_image_raw_comp_ << " " << total_time << " " << median << " " << latency_mean_image_raw_comp_ / count_image_raw_comp_ << " " << msg_freq << " " << throughput << " " << static_cast<unsigned long int>(std::round(1e6 * throughput / msg_freq)) << std::endl;
    //       evaluations_ << std::fixed << std::setprecision(6) << "oak/rgb/image_raw/compressed" << " " << count_image_raw_comp_ << " " << total_time << " " << median << " " << latency_mean_image_raw_comp_ / count_image_raw_comp_ << " " << msg_freq << " " << throughput << " " << static_cast<unsigned long int>(std::round(1e6 * throughput / msg_freq)) << std::endl;
    //       // image_raw_comp_.close();
    //       flag_3 = false;
    //       end_script_image_raw_comp_ = false;
    //       start_time_image_raw_comp_ = -1.0;
    //       count_image_raw_comp_ = 0;
    //       start_script_image_raw_comp_ = false;
    //       data_image_raw_comp_ = 0;
    //       latency_mean_image_raw_comp_ = 0.0;
    //       count_image_raw_comp_vector_.clear();
    //     }
    //   }
    // }

    // void image_stereo_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
    // {
    //   if (start_script_image_stereo_ && flag_4)
    //   { 
    //     double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
    //     double time_now = this->get_clock()->now().seconds();

    //     if (get_start_time_image_stereo_)
    //     {
    //       first_msg_time_image_stereo_ = msg_time;
    //       start_time_image_stereo_ = time_now;
    //       get_start_time_image_stereo_ = false;
    //       // return;
    //     }
        
    //     // time_now += first_msg_time_image_stereo_ - start_time_image_stereo_; 

    //     double latency = time_now - msg_time;

    //     count_image_stereo_vector_.push_back(latency);

    //     image_stereo_ << std::fixed << std::setprecision(6) << msg_time << " " << time_now << " " << msg->height << " " << msg->width << " " << msg->step << " " << msg->data.size() << " " << latency << std::endl;

    //     count_image_stereo_++;
    //     latency_mean_image_stereo_ += latency; 

    //     if (end_script_image_stereo_)
    //     {
    //       double total_time = time_now - start_time_image_stereo_;
    //       double msg_freq = count_image_stereo_ / total_time;
    //       double throughput = 1e-6 * msg->data.size() * msg_freq;

    //       double median = -1.0;
    //       median = calculate_median(count_image_stereo_vector_);

    //       image_stereo_ << std::endl;
    //       image_stereo_ << filename_append_ << " " << "msg_count" << " " << "evaluation_time_secs" << " " << "median_latency_secs" << " " << "mean_latency_secs" << " " << "msg_freq_Hz" << " " << "throughput_MB_p_sec" << std::endl;
    //       image_stereo_ << std::fixed << std::setprecision(6) << "oak/stereo/image_raw" << " " << count_image_stereo_ << " " << total_time << " " << median << " " << latency_mean_image_stereo_ / count_image_stereo_ << " " << msg_freq << " " << throughput << std::endl;
    //       evaluations_ << std::fixed << std::setprecision(6) << "oak/stereo/image_raw" << " " << count_image_stereo_ << " " << total_time << " " << median << " " << latency_mean_image_stereo_ / count_image_stereo_ << " " << msg_freq << " " << throughput << std::endl;
    //       // image_stereo_.close();
    //       flag_4 = false;
    //       start_time_image_stereo_ = -1.0;
    //       count_image_stereo_ = 0;
    //       end_script_image_stereo_ = false;
    //       latency_mean_image_stereo_ = 0.0;
    //       count_image_stereo_vector_.clear();
    //     }
    //   }
    // }

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
    {
      if (start_script_costmap_ && flag_5)
      { 
        double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
        double time_now = this->get_clock()->now().seconds();

        if (get_start_time_costmap_)
        {
          first_msg_time_costmap_ = msg_time;
          start_time_costmap_ = time_now;
          get_start_time_costmap_ = false;
          // return;
        }
        
        // time_now += first_msg_time_costmap_ - start_time_costmap_; 

        double latency = time_now - msg_time;

        count_costmap_vector_.push_back(latency);

        costmap_ << std::fixed << std::setprecision(6) << msg_time << " " << time_now << " " << msg->info.width << " " << msg->info.height << " " <<  msg->data.size() << " " << latency << std::endl;
        
        count_costmap_++;
        latency_mean_costmap_ += latency;

        if (end_script_costmap_)
        {
          double total_time = time_now - start_time_costmap_;
          double msg_freq = count_costmap_ / total_time;
          double throughput = 1e-6 * msg->data.size() * msg_freq;

          double median = -1.0;
          median = calculate_median(count_costmap_vector_);

          costmap_ << std::endl;
          costmap_ << filename_append_ << " " << "msg_count" << " " << "evaluation_time_secs" << " " << "median_latency_secs" << " " << "mean_latency_secs" << " " << "msg_freq_Hz" << " " << "throughput_MB_p_sec" << std::endl;
          costmap_ << std::fixed << std::setprecision(6) << "local_costmap/costmap" << " " << count_costmap_ << " " << total_time << " " << median << " " << latency_mean_costmap_ / count_costmap_ << " " << msg_freq << " " << throughput << std::endl; 
          evaluations_ << std::fixed << std::setprecision(6) << "local_costmap/costmap" << " " << count_costmap_ << " " << total_time << " " << median << " " << latency_mean_costmap_ / count_costmap_ << " " << msg_freq << " " << throughput << std::endl; 
          // costmap_.close();
          flag_5 = false;
          end_script_costmap_ = false;
          start_time_costmap_ = -1.0;
          count_costmap_ = 0;
          end_script_costmap_ = false;
          latency_mean_costmap_ = 0.0;
          count_costmap_vector_.clear();
        }
      }
    }

    // void costmap_raw_callback(const nav2_msgs::msg::Costmap::SharedPtr msg) 
    // {
    //   if (start_script_costmap_raw_ && flag_6)
    //   { 
    //     double msg_time = msg->header.stamp.sec + (1e-9 * msg->header.stamp.nanosec);
    //     double time_now = this->get_clock()->now().seconds();

    //     if (get_start_time_costmap_raw_)
    //     {
    //       first_msg_time_costmap_raw_ = msg_time;
    //       start_time_costmap_raw_ = time_now;
    //       get_start_time_costmap_raw_ = false;
    //       // return;
    //     }
        
    //     // time_now += first_msg_time_costmap_raw_ - start_time_costmap_raw_; 

    //     double latency = time_now - msg_time;

    //     count_costmap_raw_vector_.push_back(latency);

    //     costmap_raw_ << std::fixed << std::setprecision(6) << msg_time << " " << time_now << " " << msg->metadata.map_load_time.sec + (1e-9 * msg->metadata.map_load_time.nanosec) << " " << msg->metadata.update_time.sec + (1e-9 * msg->metadata.update_time.nanosec) << " "<< msg->metadata.size_x << " " << msg->metadata.size_y << " " << msg->data.size() << " " << latency << std::endl;

    //     count_costmap_raw_++;
    //     latency_mean_costmap_raw_ += latency;

    //     if (end_script_costmap_raw_)
    //     {
    //       double total_time = time_now - start_time_costmap_raw_;
    //       double msg_freq = count_costmap_raw_ / total_time;
    //       double throughput = 1e-6 * msg->data.size() * msg_freq;

    //       double median = -1.0;
    //       median = calculate_median(count_costmap_raw_vector_);

    //       costmap_raw_ << std::endl;
    //       costmap_raw_ << filename_append_ << " " << "msg_count" << " " << "evaluation_time_secs" << " " << "median_latency_secs" << " " << "mean_latency_secs" << " " << "msg_freq_Hz" << " " << "throughput_MB_p_sec" << std::endl;
    //       costmap_raw_ << std::fixed << std::setprecision(6) << "local_costmap/costmap_raw" << " " << count_costmap_raw_ << " " << total_time << " " << median << " " << latency_mean_costmap_raw_ / count_costmap_raw_ << " " << msg_freq << " " << throughput << std::endl; 
    //       evaluations_ << std::fixed << std::setprecision(6) << "local_costmap/costmap_raw" << " " << count_costmap_raw_ << " " << total_time << " " << median << " " << latency_mean_costmap_raw_ / count_costmap_raw_ << " " << msg_freq << " " << throughput << std::endl; 
    //       // costmap_raw_.close();
    //       flag_6 = false;
    //       end_script_costmap_raw_ = false;
    //       start_time_costmap_raw_ = -1.0;
    //       count_costmap_raw_ = 0;
    //       end_script_costmap_raw_ = false;
    //       latency_mean_costmap_raw_ = 0.0;
    //       count_costmap_raw_vector_.clear();
    //     }
    //   }
    // }

    ~ZenohSubscriber()
    {
      image_raw_.close();
      // image_rect_.close();
      // image_raw_comp_.close();
      // image_stereo_.close();
      costmap_.close();
      // costmap_raw_.close();
      evaluations_.close();
    }
    
  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_rect_sub_;
    // rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_raw_comp_sub_;
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_stereo_sub_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    // rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_raw_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_script_sub_;

    std::ofstream image_raw_;
    // std::ofstream image_rect_;
    // std::ofstream image_raw_comp_;
    // std::ofstream image_stereo_;
    std::ofstream costmap_;
    // std::ofstream costmap_raw_;
    std::ofstream evaluations_;

    bool start_script_image_raw_{false};
    // bool start_script_image_rect_{false};
    // bool start_script_image_stereo_{false};
    // bool start_script_image_raw_comp_{false};
    bool start_script_costmap_{false};
    // bool start_script_costmap_raw_{false};

    bool end_script_image_raw_{false};
    // bool end_script_image_rect_{false};
    // bool end_script_image_stereo_{false};
    // bool end_script_image_raw_comp_{false};  
    bool end_script_costmap_{false}; 
    // bool end_script_costmap_raw_{false}; 
     
    unsigned long int count_image_raw_{0};
    // unsigned long int count_image_rect_{0};
    // unsigned long int count_image_raw_comp_{0};
    // unsigned long int count_image_stereo_{0};
    unsigned long int count_costmap_{0};
    // unsigned long int count_costmap_raw_{0};

    // unsigned long long int data_image_raw_comp_{0};

    rclcpp::TimerBase::SharedPtr timer_;

    bool flag_{true};
    bool flag_1{true};
    // bool flag_2{true};
    // bool flag_3{true};
    // bool flag_4{true};
    bool flag_5{true};
    // bool flag_6{true};

    bool get_start_time_image_raw_{false};
    // bool get_start_time_image_rect_{false};
    // bool get_start_time_image_stereo_{false};
    // bool get_start_time_image_raw_comp_{false};
    bool get_start_time_costmap_{false};
    // bool get_start_time_costmap_raw_{false};

    double start_time_image_raw_{-1.0};
    // double start_time_image_rect_{-1.0};
    // double start_time_image_stereo_{-1.0};
    // double start_time_image_raw_comp_{-1.0};
    double start_time_costmap_{-1.0};
    // double start_time_costmap_raw_{-1.0};

    double first_msg_time_image_raw_{-1.0};
    // double first_msg_time_image_rect_{-1.0};
    // double first_msg_time_image_stereo_{-1.0};
    // double first_msg_time_image_raw_comp_{-1.0};
    double first_msg_time_costmap_{-1.0};
    // double first_msg_time_costmap_raw_{-1.0};

    double latency_mean_image_raw_{0.0};
    // double latency_mean_image_rect_{0.0};
    // double latency_mean_image_raw_comp_{0.0};
    // double latency_mean_image_stereo_{0.0};
    double latency_mean_costmap_{0.0};
    // double latency_mean_costmap_raw_{0.0};

    std::vector<double> count_image_raw_vector_;
    // std::vector<double> count_image_rect_vector_;
    // std::vector<double> count_image_stereo_vector_;
    // std::vector<double> count_image_raw_comp_vector_;
    std::vector<double> count_costmap_vector_;
    // std::vector<double> count_costmap_raw_vector_;
    std::string filename_append_;
    std::string oak_camera_topic_;
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
