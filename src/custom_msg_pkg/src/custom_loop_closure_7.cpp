#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>

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
auto odom_3 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_4 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_5 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_6 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_7 = cslam_common_interfaces::msg::KeyframeOdom();
auto odom_8 = cslam_common_interfaces::msg::KeyframeOdom();
/* NOISY
auto odom_2_noisy = cslam_common_interfaces::msg::KeyframeOdom();
*/
float t = 5.0;

class LoopClosurePublisher : public rclcpp::Node
{
  public:
    LoopClosurePublisher()
    : Node("loop_closure_publisher"), count_(0)
    {

// SUBSCRIPTIONS

      // Subscription 1
      subscription1_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r0/cslam/keyframe_odom", 10, std::bind(&LoopClosurePublisher::topic1_callback, this, _1));

      // Subscription 2
      subscription2_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r1/cslam/keyframe_odom", 10, std::bind(&LoopClosurePublisher::topic2_callback, this, _1));

      // Subscription 3
      subscription3_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r2/cslam/keyframe_odom", 10, std::bind(&LoopClosurePublisher::topic3_callback, this, _1));

      // Subscription 4
      subscription4_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r3/cslam/keyframe_odom", 10, std::bind(&LoopClosurePublisher::topic4_callback, this, _1));

      // Subscription 5
      subscription5_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r4/cslam/keyframe_odom", 10, std::bind(&LoopClosurePublisher::topic5_callback, this, _1));

      // Subscription 6
      subscription6_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r5/cslam/keyframe_odom", 10, std::bind(&LoopClosurePublisher::topic6_callback, this, _1));

      // Subscription 7
      subscription7_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r6/cslam/keyframe_odom", 10, std::bind(&LoopClosurePublisher::topic7_callback, this, _1));

      // Subscription 8
      subscription8_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r7/cslam/keyframe_odom", 10, std::bind(&LoopClosurePublisher::topic8_callback, this, _1));

/* NOISY
      // Subscription 2 NOISY
      subscription2_noisy_ = this->create_subscription<nav_msgs::msg::Odometry>("/bot2/noisy_odom", 10, std::bind(&LoopClosurePublisher::topic2_noisy_callback, this, _1));
*/

// PUBLISHERS

      // Loop Closure Publisher
      publisher_ = this->create_publisher<cslam_common_interfaces::msg::InterRobotLoopClosure>("/cslam/inter_robot_loop_closure", 100);

// LOOP CLOSURES

      // Timer 7-1
      timer71_ = this->create_wall_timer(
      50000ms, std::bind(&LoopClosurePublisher::timer71_callback, this));

      // Timer 7-2
      timer72_ = this->create_wall_timer(
      50000ms, std::bind(&LoopClosurePublisher::timer72_callback, this));

      // Timer 7-3
      timer73_ = this->create_wall_timer(
      50000ms, std::bind(&LoopClosurePublisher::timer73_callback, this));

      // Timer 7-4
      timer74_ = this->create_wall_timer(
      50000ms, std::bind(&LoopClosurePublisher::timer74_callback, this));

      // Timer 7-5
      timer75_ = this->create_wall_timer(
      50000ms, std::bind(&LoopClosurePublisher::timer75_callback, this));

      // Timer 7-6
      timer76_ = this->create_wall_timer(
      50000ms, std::bind(&LoopClosurePublisher::timer76_callback, this));

      // Timer 7-8
      timer78_ = this->create_wall_timer(
      50000ms, std::bind(&LoopClosurePublisher::timer78_callback, this));
    }

// SUBSCRIPTIONS FUNCTIONS

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
    void topic5_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_5.id = msg->id;
      odom_5.odom.pose = msg->odom.pose;
    }

  private:
    void topic6_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_6.id = msg->id;
      odom_6.odom.pose = msg->odom.pose;
    }

  private:
    void topic7_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_7.id = msg->id;
      odom_7.odom.pose = msg->odom.pose;
    }

  private:
    void topic8_callback(const cslam_common_interfaces::msg::KeyframeOdom::SharedPtr msg) const
    {
      odom_8.id = msg->id;
      odom_8.odom.pose = msg->odom.pose;
    }

/* NOISY
  private:
    void topic2_noisy_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
      odom_2_noisy.odom.pose = msg->pose;
    }
*/

// LOOP CLOSURES FUNCTIONS

  private:
    void timer71_callback() const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      float dx = 0.0;
      float dy = 0.0;
      
      message.robot0_keyframe_id = odom_7.id;
      message.robot0_id = 6;
      message.robot1_keyframe_id = odom_1.id;
      message.robot1_id = 0;
      
      dx = odom_1.odom.pose.pose.position.x - odom_7.odom.pose.pose.position.x;
      dy = odom_1.odom.pose.pose.position.y - odom_7.odom.pose.pose.position.y;

      message.transform.translation.x = dx;
      message.transform.translation.y = dy;

      if (sqrt (pow(dx, 2) + pow(dy, 2)) <= t)
        message.success = true;
      else
        message.success = false;

      //publisher_-> publish(message);
    }

  private:
    void timer72_callback() const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      float dx = 0.0;
      float dy = 0.0;
      
      message.robot0_keyframe_id = odom_7.id;
      message.robot0_id = 6;
      message.robot1_keyframe_id = odom_2.id;
      message.robot1_id = 1;
      
      dx = odom_2.odom.pose.pose.position.x - odom_7.odom.pose.pose.position.x;
      dy = odom_2.odom.pose.pose.position.y - odom_7.odom.pose.pose.position.y;

      message.transform.translation.x = dx;
      message.transform.translation.y = dy;

      if (sqrt (pow(dx, 2) + pow(dy, 2)) <= t)
        message.success = true;
      else
        message.success = false;

      //publisher_-> publish(message);
    }

    private:
    void timer73_callback() const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      float dx = 0.0;
      float dy = 0.0;
      
      message.robot0_keyframe_id = odom_7.id;
      message.robot0_id = 6;
      message.robot1_keyframe_id = odom_3.id;
      message.robot1_id = 2;

      dx = odom_3.odom.pose.pose.position.x - odom_7.odom.pose.pose.position.x;
      dy = odom_3.odom.pose.pose.position.y - odom_7.odom.pose.pose.position.y;

      message.transform.translation.x = dx;
      message.transform.translation.y = dy;

      if (sqrt (pow(dx, 2) + pow(dy, 2)) <= t)
        message.success = true;
      else
        message.success = false;

      //publisher_-> publish(message);
    }

    private:
    void timer74_callback() const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      float dx = 0.0;
      float dy = 0.0;
      
      message.robot0_keyframe_id = odom_7.id;
      message.robot0_id = 6;
      message.robot1_keyframe_id = odom_4.id;
      message.robot1_id = 3;
      
      dx = odom_4.odom.pose.pose.position.x - odom_7.odom.pose.pose.position.x;
      dy = odom_4.odom.pose.pose.position.y - odom_7.odom.pose.pose.position.y;

      message.transform.translation.x = dx;
      message.transform.translation.y = dy;

      if (sqrt (pow(dx, 2) + pow(dy, 2)) <= t)
        message.success = true;
      else
        message.success = false;

      //publisher_-> publish(message);
    }

    private:
    void timer75_callback() const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      float dx = 0.0;
      float dy = 0.0;
      
      message.robot0_keyframe_id = odom_7.id;
      message.robot0_id = 6;
      message.robot1_keyframe_id = odom_5.id;
      message.robot1_id = 4;
      
      dx = odom_5.odom.pose.pose.position.x - odom_7.odom.pose.pose.position.x;
      dy = odom_5.odom.pose.pose.position.y - odom_7.odom.pose.pose.position.y;

      message.transform.translation.x = dx;
      message.transform.translation.y = dy;

      if (sqrt (pow(dx, 2) + pow(dy, 2)) <= t)
        message.success = true;
      else
        message.success = false;

      //publisher_-> publish(message);
    }

    private:
    void timer76_callback() const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      float dx = 0.0;
      float dy = 0.0;
      
      message.robot0_keyframe_id = odom_7.id;
      message.robot0_id = 6;
      message.robot1_keyframe_id = odom_6.id;
      message.robot1_id = 5;
      
      dx = odom_6.odom.pose.pose.position.x - odom_7.odom.pose.pose.position.x;
      dy = odom_6.odom.pose.pose.position.y - odom_7.odom.pose.pose.position.y;

      message.transform.translation.x = dx;
      message.transform.translation.y = dy;

      if (sqrt (pow(dx, 2) + pow(dy, 2)) <= t)
        message.success = true;
      else
        message.success = false;

      //publisher_-> publish(message);
    }

    private:
    void timer78_callback() const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      float dx = 0.0;
      float dy = 0.0;
      
      message.robot0_keyframe_id = odom_7.id;
      message.robot0_id = 6;
      message.robot1_keyframe_id = odom_8.id;
      message.robot1_id = 7;
      
      dx = odom_8.odom.pose.pose.position.x - odom_7.odom.pose.pose.position.x;
      dy = odom_8.odom.pose.pose.position.y - odom_7.odom.pose.pose.position.y;

      message.transform.translation.x = dx;
      message.transform.translation.y = dy;

      if (sqrt (pow(dx, 2) + pow(dy, 2)) <= t)
        message.success = true;
      else
        message.success = false;

      //publisher_-> publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer71_;
    rclcpp::TimerBase::SharedPtr timer72_;
    rclcpp::TimerBase::SharedPtr timer73_;
    rclcpp::TimerBase::SharedPtr timer74_;
    rclcpp::TimerBase::SharedPtr timer75_;
    rclcpp::TimerBase::SharedPtr timer76_;
    rclcpp::TimerBase::SharedPtr timer78_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription1_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription2_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription3_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription4_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription5_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription6_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription7_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription8_;
    /* NOISY
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription2_noisy_;
    */
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