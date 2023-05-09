#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.h"
#include <std_msgs/msg/header.hpp>

#include "cslam_common_interfaces/msg/keyframe_odom.hpp"
#include "cslam_common_interfaces/msg/inter_robot_loop_closure.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

int c = 0;

auto odom_1 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_2 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_3 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_4 = cslam_common_interfaces::msg::KeyframeOdom();

class LoopClosurePublisher : public rclcpp::Node
{
  public:
    LoopClosurePublisher()
    : Node("loop_closure_publisher"), count_(0)
    {
      // Subscription 1
      subscription1_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/bot1/odom", 10, std::bind(&LoopClosurePublisher::topic1_callback, this, _1));

      // Subscription 2
      subscription2_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/bot2/odom", 10, std::bind(&LoopClosurePublisher::topic2_callback, this, _1));

      // Subscription 3
      subscription3_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/bot3/odom", 10, std::bind(&LoopClosurePublisher::topic3_callback, this, _1));

      // Subscription 4
      subscription4_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/bot4/odom", 10, std::bind(&LoopClosurePublisher::topic4_callback, this, _1));

      // Loop Closure Publisher
      publisher_ = this->create_publisher<cslam_common_interfaces::msg::InterRobotLoopClosure>("/cslam/inter_robot_loop_closure", 10);

      // Timer 1-2
      timer2_ = this->create_wall_timer(
      10000ms, std::bind(&LoopClosurePublisher::timer2_callback, this));

      // Timer 1-3
      timer3_ = this->create_wall_timer(
      10000ms, std::bind(&LoopClosurePublisher::timer3_callback, this));

      // Timer 1-4
      timer4_ = this->create_wall_timer(
      10000ms, std::bind(&LoopClosurePublisher::timer4_callback, this));
    }

  private:
    void topic1_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_1.id = msg->id;
      odom_1.odom.pose = msg->odom.pose;
    }

  private:
    void topic2_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_2.id = msg->id;
      odom_2.odom.pose = msg->odom.pose;
    }

  private:
    void topic3_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_3.id = msg->id;
      odom_3.odom.pose = msg->odom.pose;
    }

  private:
    void topic4_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_4.id = msg->id;
      odom_4.odom.pose = msg->odom.pose;
    }

  private:
    void timer2_callback() const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      
      message.robot0_keyframe_id = odom_1.id;
      message.robot0_id = 1;
      message.robot1_keyframe_id = odom_2.id;
      message.robot1_id = 2;
      message.success = true;

      publisher_-> publish(message);
    }

  private:
    void timer3_callback() const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      
      message.robot0_keyframe_id = odom_1.id;
      message.robot0_id = 1;
      message.robot1_keyframe_id = odom_3.id;
      message.robot1_id = 3;
      message.success = true;

      publisher_-> publish(message);
    }

  private:
    void timer4_callback() const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      
      message.robot0_keyframe_id = odom_1.id;
      message.robot0_id = 1;
      message.robot1_keyframe_id = odom_4.id;
      message.robot1_id = 4;
      message.success = true;


      publisher_-> publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::TimerBase::SharedPtr timer3_;
    rclcpp::TimerBase::SharedPtr timer4_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription1_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription2_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription3_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription4_;
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