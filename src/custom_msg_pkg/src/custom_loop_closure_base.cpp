#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.h"
#include <std_msgs/msg/header.hpp>

#include "cslam_common_interfaces/msg/keyframe_odom.hpp"
#include "cslam_common_interfaces/msg/inter_robot_loop_closure.hpp"

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
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/bot1/odom", 10,
      std::bind(&PubSub::topic_callback, this, _1));

      publisher_ = this->create_publisher<cslam_common_interfaces::msg::InterRobotLoopClosure>("/cslam/inter_robot_loop_closure", 10);
    }

  private:
    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg_odometry) const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      message.robot0_keyframe_id = 0;
      message.robot0_id = 1;
      message.robot1_keyframe_id = 0;
      message.robot1_id = 2;
      message.success = true;
      message.transform.translation.x = msg_odometry->pose.pose.position.x;
      message.transform.translation.y = msg_odometry->pose.pose.position.y;

      int r = 1+(rand()%1000);

      if(r == 500)
        publisher_-> publish(message);

    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<cslam_common_interfaces::msg::InterRobotLoopClosure>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubSub>());
  rclcpp::shutdown();
  return 0;
}