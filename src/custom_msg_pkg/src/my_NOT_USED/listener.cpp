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

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class PubSub : public rclcpp::Node
{
  public:
    PubSub()
    : Node("pubsub")
    {
      subscription_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("cslam/keyframe_odom", 1000,
      std::bind(&PubSub::topic_callback, this, _1));

      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/listener", 10);
    }

  private:
    void topic_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      auto message = nav_msgs::msg::Odometry();
      message.header = msg->odom.header;
      message.pose = msg->odom.pose;
      publisher_-> publish(message);

      // auto message = nav_msgs::msg::Odometry();
      // message.header = msg_odometry->header;
      // message.pose = msg_odometry->pose;
      // publisher_-> publish(message);
    }
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubSub>());
  rclcpp::shutdown();
  return 0;
}