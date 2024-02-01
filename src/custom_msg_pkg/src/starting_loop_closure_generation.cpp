#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>
#include <bits/stdc++.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.h"
#include <std_msgs/msg/header.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "cslam_common_interfaces/msg/keyframe_odom.hpp"
#include "cslam_common_interfaces/msg/inter_robot_loop_closure.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

int c = 0;

// For subscriptions
auto odom_1 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_2 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_3 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_4 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_5 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_6 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_7 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_8 = cslam_common_interfaces::msg::KeyframeOdom();
std::vector<cslam_common_interfaces::msg::KeyframeOdom> odom_vector(8);

class LoopClosurePublisher : public rclcpp::Node
{  
  public:
    LoopClosurePublisher()
    : Node("loop_closure_publisher"), count_(0)
    {

// SUBSCRIPTIONS

      // Subscriptions to Custom Odometry
      subscription1_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r0/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic1_callback, this, _1));
      subscription2_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r1/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic2_callback, this, _1));
      subscription3_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r2/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic3_callback, this, _1));
      subscription8_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r7/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic8_callback, this, _1));

// PUBLISHERS

      // Transformation for blockchain
      publisher_transf_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_C0", 100);

    }

// SUBSCRIPTIONS FUNCTIONS

  private:
    void topic1_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_vector[0].id = msg->id;
      odom_vector[0].odom.pose = msg->odom.pose;
    }

  private:
    void topic2_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_vector[1].id = msg->id;
      odom_vector[1].odom.pose = msg->odom.pose;
    }

  private:
    void topic3_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_vector[2].id = msg->id;
      odom_vector[2].odom.pose = msg->odom.pose;
    }

  private:
    void topic8_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      float dx1 = 0.0;
      float dy1 = 0.0;
      float dx2 = 0.0;
      float dy2 = 0.0;
      float dx3 = 0.0;
      float dy3 = 0.0;

      auto message_transf1 = std_msgs::msg::Float64MultiArray();
      auto message_transf2 = std_msgs::msg::Float64MultiArray();
      auto message_transf3 = std_msgs::msg::Float64MultiArray();

      if(c == 1000 || c == 5000 || c == 10000 || c == 20000 || c == 30000){

      dx1 = odom_vector[1].odom.pose.pose.position.x - odom_vector[0].odom.pose.pose.position.x;
      dy1 = odom_vector[1].odom.pose.pose.position.y - odom_vector[0].odom.pose.pose.position.y;
      dx2 = odom_vector[2].odom.pose.pose.position.x - odom_vector[1].odom.pose.pose.position.x;
      dy2 = odom_vector[2].odom.pose.pose.position.y - odom_vector[1].odom.pose.pose.position.y;
      dx3 = odom_vector[0].odom.pose.pose.position.x - odom_vector[2].odom.pose.pose.position.x;
      dy3 = odom_vector[0].odom.pose.pose.position.y - odom_vector[2].odom.pose.pose.position.y;

      message_transf1.data = {(-c+1), 2, odom_vector[1].odom.pose.pose.position.x, odom_vector[1].odom.pose.pose.position.y, odom_vector[1].id, 1, odom_vector[0].odom.pose.pose.position.x, odom_vector[0].odom.pose.pose.position.y, odom_vector[0].id, dx1, dy1, 1};
      message_transf2.data = {(-c+2), 3, odom_vector[2].odom.pose.pose.position.x, odom_vector[2].odom.pose.pose.position.y, odom_vector[2].id, 2, odom_vector[1].odom.pose.pose.position.x, odom_vector[1].odom.pose.pose.position.y, odom_vector[1].id, dx2, dy2, 1};
      message_transf3.data = {(-c+3), 1, odom_vector[0].odom.pose.pose.position.x, odom_vector[0].odom.pose.pose.position.y, odom_vector[0].id, 3, odom_vector[2].odom.pose.pose.position.x, odom_vector[2].odom.pose.pose.position.y, odom_vector[2].id, dx3, dy3, 1};

      RCLCPP_INFO_STREAM(this->get_logger(), "New Loop Closure, x " << dx1 << " ,y " << dy1);
      RCLCPP_INFO_STREAM(this->get_logger(), "New Loop Closure, x " << dx2 << " ,y " << dy2);
      RCLCPP_INFO_STREAM(this->get_logger(), "New Loop Closure, x " << dx3 << " ,y " << dy3);

      publisher_transf_-> publish(message_transf1);
      publisher_transf_-> publish(message_transf2);
      publisher_transf_-> publish(message_transf3);
      }
      c++;
    }

    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription1_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription2_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription3_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription8_;
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoopClosurePublisher>());
  rclcpp::shutdown();
  return 0;
}

