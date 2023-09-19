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

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class PubSub : public rclcpp::Node
{
  public:
    PubSub()
    : Node("pubsub")
    {
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/bot2/odom", 10,
      std::bind(&PubSub::topic_callback, this, _1));

      publisher_ = this->create_publisher<cslam_common_interfaces::msg::KeyframeOdom>("/r1/cslam/keyframe_odom", 1000);
      // publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/keyframe_odom", 10);
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

      pose_id ++;

      // auto message = nav_msgs::msg::Odometry();
      // message.header = msg_odometry->header;
      // message.pose = msg_odometry->pose;
      // publisher_-> publish(message);
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Publisher<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubSub>());
  rclcpp::shutdown();
  return 0;
}