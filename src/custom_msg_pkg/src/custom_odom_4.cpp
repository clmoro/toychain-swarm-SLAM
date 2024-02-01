#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.h"
#include <std_msgs/msg/header.hpp>

#include "cslam_common_interfaces/msg/keyframe_odom.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

int pose_id = 0;
int pose_id_n = 0;

class PubSub : public rclcpp::Node
{
  public:
    PubSub()
    : Node("pubsub")
    {
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/bot4/odom", 100,
      std::bind(&PubSub::topic_callback, this, _1));
      subscription_noisy_ = this->create_subscription<nav_msgs::msg::Odometry>("/bot4/noisy_odom", 100,
      std::bind(&PubSub::topic_callback_noisy, this, _1));

      publisher_ = this->create_publisher<cslam_common_interfaces::msg::KeyframeOdom>("/r3/cslam/keyframe_odom_ideal", 1000);
      publisher_noisy_ = this->create_publisher<cslam_common_interfaces::msg::KeyframeOdom>("/r3/cslam/keyframe_odom", 1000);
    }

  private:
    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg_odometry) const
    {
      auto message = cslam_common_interfaces::msg::KeyframeOdom();
      message.id = pose_id;
      message.odom.header = msg_odometry->header;
      message.odom.pose = msg_odometry->pose;
      message.odom.pose.pose.orientation.x = 0.0;
      message.odom.pose.pose.orientation.y = 0.0;
      message.odom.pose.pose.orientation.z = 0.0;
      message.odom.pose.pose.orientation.w = 0.0;
      publisher_-> publish(message);
      // publisher_noisy_-> publish(message);

      pose_id ++;
    }

  private:
    void topic_callback_noisy(const nav_msgs::msg::Odometry::SharedPtr msg_odometry) const
    {
      auto message = cslam_common_interfaces::msg::KeyframeOdom();
      message.id = pose_id_n;
      message.odom.header = msg_odometry->header;
      message.odom.pose = msg_odometry->pose;
      message.odom.pose.pose.orientation.x = 0.0;
      message.odom.pose.pose.orientation.y = 0.0;
      message.odom.pose.pose.orientation.z = 0.0;
      message.odom.pose.pose.orientation.w = 0.0;
      publisher_noisy_-> publish(message);

      pose_id_n ++;
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_noisy_;
    rclcpp::Publisher<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr publisher_;
    rclcpp::Publisher<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr publisher_noisy_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubSub>());
  rclcpp::shutdown();
  return 0;
}