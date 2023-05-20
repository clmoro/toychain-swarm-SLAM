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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "cslam_common_interfaces/msg/keyframe_odom.hpp"
#include "cslam_common_interfaces/msg/inter_robot_loop_closure.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

auto odom_1 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_2 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_2_noisy = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_3 = cslam_common_interfaces::msg::KeyframeOdom();

//geometry_msgs::msg::TransformStamped tf_result;

class LoopClosurePublisher : public rclcpp::Node
{
  public:
    LoopClosurePublisher()
    : Node("loop_closure_publisher"), count_(0)
    {
      //tf_result.header.stamp = this->get_clock()->now();

      // Subscription 1
      subscription1_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r0/cslam/keyframe_odom", 10, std::bind(&LoopClosurePublisher::topic1_callback, this, _1));

      // Subscription 2
      subscription2_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r1/cslam/keyframe_odom", 10, std::bind(&LoopClosurePublisher::topic2_callback, this, _1));

      // Subscription 2 NOISY
      subscription2_noisy_ = this->create_subscription<nav_msgs::msg::Odometry>("/bot2/noisy_odom", 10, std::bind(&LoopClosurePublisher::topic2_noisy_callback, this, _1));

      // Subscription 3
      subscription3_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r2/cslam/keyframe_odom", 10, std::bind(&LoopClosurePublisher::topic3_callback, this, _1));

      // Loop Closure Publisher
      publisher_ = this->create_publisher<cslam_common_interfaces::msg::InterRobotLoopClosure>("/cslam/inter_robot_loop_closure", 10);

      // Timer 1-2
      timer12_ = this->create_wall_timer(
      10000ms, std::bind(&LoopClosurePublisher::timer12_callback, this));

      // Timer 2-1
      timer21_ = this->create_wall_timer(
      10000ms, std::bind(&LoopClosurePublisher::timer21_callback, this));

      // Timer 1-3
      timer13_ = this->create_wall_timer(
      10000ms, std::bind(&LoopClosurePublisher::timer13_callback, this));

      // Timer 3-1
      timer31_ = this->create_wall_timer(
      10000ms, std::bind(&LoopClosurePublisher::timer31_callback, this));
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
    void topic2_noisy_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
      odom_2_noisy.odom.pose = msg->pose;
    }

  private:
    void topic3_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_3.id = msg->id;
      odom_3.odom.pose = msg->odom.pose;
    }

  private:
    void timer12_callback() const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      
      message.robot0_keyframe_id = odom_1.id;
      message.robot0_id = 0;
      message.robot1_keyframe_id = odom_2.id;
      message.robot1_id = 1;
      message.success = true;
      
      message.transform.translation.x = odom_2_noisy.odom.pose.pose.position.x - odom_1.odom.pose.pose.position.x;
      message.transform.translation.y = odom_2_noisy.odom.pose.pose.position.y - odom_1.odom.pose.pose.position.y;

        publisher_-> publish(message);

    }

  private:
    void timer21_callback() const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      
      message.robot0_keyframe_id = odom_2.id;
      message.robot0_id = 1;
      message.robot1_keyframe_id = odom_1.id;
      message.robot1_id = 0;
      message.success = true;

        //publisher_-> publish(message);

    }

  private:
    void timer13_callback() const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      
      message.robot0_keyframe_id = odom_1.id;
      message.robot0_id = 0;
      message.robot1_keyframe_id = odom_3.id;
      message.robot1_id = 2;
      message.success = true;

      message.transform.translation.x = odom_3.odom.pose.pose.position.x - odom_1.odom.pose.pose.position.x;
      message.transform.translation.y = odom_3.odom.pose.pose.position.y - odom_1.odom.pose.pose.position.y;

        publisher_-> publish(message);

    }

  private:
    void timer31_callback() const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      
      message.robot0_keyframe_id = odom_3.id;
      message.robot0_id = 2;
      message.robot1_keyframe_id = odom_1.id;
      message.robot1_id = 0;
      message.success = true;

      message.transform.translation.x = odom_3.odom.pose.pose.position.x - odom_1.odom.pose.pose.position.x;
      message.transform.translation.y = odom_3.odom.pose.pose.position.y - odom_1.odom.pose.pose.position.y;

        //publisher_-> publish(message);

    }

    rclcpp::TimerBase::SharedPtr timer12_;
    rclcpp::TimerBase::SharedPtr timer21_;
    rclcpp::TimerBase::SharedPtr timer13_;
    rclcpp::TimerBase::SharedPtr timer31_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription1_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription2_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription2_noisy_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription3_;
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