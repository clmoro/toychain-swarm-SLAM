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

// Constants
int e = 1;

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
      subscription4_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r3/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic4_callback, this, _1));
      subscription5_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r4/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic5_callback, this, _1));
      subscription6_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r5/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic6_callback, this, _1));
      subscription7_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r6/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic7_callback, this, _1));
      subscription8_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r7/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic8_callback, this, _1));

// PUBLISHERS

      // Loop Closure Publisher
      publisher_ = this->create_publisher<cslam_common_interfaces::msg::InterRobotLoopClosure>("/cslam/inter_robot_loop_closure", 100);

    }

// SUBSCRIPTIONS FUNCTIONS

  private:
    void topic1_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_vector[0].id = msg->id;
      odom_vector[0].odom.pose = msg->odom.pose;

      // Link from 0 to all at starting in order to have a fully connected graph (no errors)
      if (odom_vector[0].id == 100) {
        while (e < 8) {
          if(e != 1) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Connection of r0 with the other individuals ...");
            auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
            float dx = 0.0;
            float dy = 0.0;
            dx = odom_vector[e].odom.pose.pose.position.x - odom_vector[0].odom.pose.pose.position.x;
            dy = odom_vector[e].odom.pose.pose.position.y - odom_vector[0].odom.pose.pose.position.y;
            message.robot0_keyframe_id = odom_vector[0].id;
            message.robot0_id = 0;
            message.robot1_keyframe_id = odom_vector[e].id;
            message.robot1_id = e;
            message.transform.translation.x = dx;
            message.transform.translation.y = dy;
            message.success = true;
            publisher_-> publish(message);
          }
            e ++;
        }
      }
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
    void topic4_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_vector[3].id = msg->id;
      odom_vector[3].odom.pose = msg->odom.pose;
    }

  private:
    void topic5_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_vector[4].id = msg->id;
      odom_vector[4].odom.pose = msg->odom.pose;
    }

  private:
    void topic6_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_vector[5].id = msg->id;
      odom_vector[5].odom.pose = msg->odom.pose;
    }

  private:
    void topic7_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_vector[6].id = msg->id;
      odom_vector[6].odom.pose = msg->odom.pose;
    }

  private:
    void topic8_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_vector[7].id = msg->id;
      odom_vector[7].odom.pose = msg->odom.pose;
    }

    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription1_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription2_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription3_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription4_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription5_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription6_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription7_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription8_;

    rclcpp::Publisher<cslam_common_interfaces::msg::InterRobotLoopClosure>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoopClosurePublisher>());
  rclcpp::shutdown();
  return 0;
}

