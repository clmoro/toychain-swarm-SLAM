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
int t2 = 0;
int t3 = 0;
int t4 = 0;
int t5 = 0;
int t6 = 0;
int t7 = 0;
int t8 = 0;

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

      // Timer 1-2
      timer12_ = this->create_wall_timer(
      50000ms, std::bind(&LoopClosurePublisher::timer12_callback, this));

      // Timer 1-3
      timer13_ = this->create_wall_timer(
      50000ms, std::bind(&LoopClosurePublisher::timer13_callback, this));

      // Timer 1-4
      timer14_ = this->create_wall_timer(
      50000ms, std::bind(&LoopClosurePublisher::timer14_callback, this));

      // Timer 1-5
      timer15_ = this->create_wall_timer(
      50000ms, std::bind(&LoopClosurePublisher::timer15_callback, this));

      // Timer 1-6
      timer16_ = this->create_wall_timer(
      50000ms, std::bind(&LoopClosurePublisher::timer16_callback, this));

      // Timer 1-7
      timer17_ = this->create_wall_timer(
      50000ms, std::bind(&LoopClosurePublisher::timer17_callback, this));

      // Timer 1-8
      timer18_ = this->create_wall_timer(
      50000ms, std::bind(&LoopClosurePublisher::timer18_callback, this));
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
    void timer12_callback() const
    {if (t2 == 0){

      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      float dx = 0.0;
      float dy = 0.0;
      
      message.robot0_keyframe_id = odom_1.id;
      message.robot0_id = 0;
      message.robot1_keyframe_id = odom_2.id;
      message.robot1_id = 1;
      
      dx = odom_2.odom.pose.pose.position.x - odom_1.odom.pose.pose.position.x;
      dy = odom_2.odom.pose.pose.position.y - odom_1.odom.pose.pose.position.y;

      message.transform.translation.x = dx;
      message.transform.translation.y = dy;

      if (t2 == 0){
        message.success = true;
        t2 = 1;}
      else
        message.success = false;

      publisher_-> publish(message);
    }}

  private:
    void timer13_callback() const
    {if (t3 == 0){

      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      float dx = 0.0;
      float dy = 0.0;
      
      message.robot0_keyframe_id = odom_1.id;
      message.robot0_id = 0;
      message.robot1_keyframe_id = odom_3.id;
      message.robot1_id = 2;
      
      dx = odom_3.odom.pose.pose.position.x - odom_1.odom.pose.pose.position.x;
      dy = odom_3.odom.pose.pose.position.y - odom_1.odom.pose.pose.position.y;

      message.transform.translation.x = dx;
      message.transform.translation.y = dy;

      if (t3 == 0){
        message.success = true;
        t3 = 1;}
      else
        message.success = false;

      publisher_-> publish(message);
    }}

    private:
    void timer14_callback() const
    {if (t4 == 0){

      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      float dx = 0.0;
      float dy = 0.0;
      
      message.robot0_keyframe_id = odom_1.id;
      message.robot0_id = 0;
      message.robot1_keyframe_id = odom_4.id;
      message.robot1_id = 3;

      dx = odom_4.odom.pose.pose.position.x - odom_1.odom.pose.pose.position.x;
      dy = odom_4.odom.pose.pose.position.y - odom_1.odom.pose.pose.position.y;

      message.transform.translation.x = dx;
      message.transform.translation.y = dy;

      if (t4 == 0){
        message.success = true;
        t4 = 1;}
      else
        message.success = false;

      publisher_-> publish(message);
    }}

    private:
    void timer15_callback() const
    {if (t5 == 0){

      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      float dx = 0.0;
      float dy = 0.0;
      
      message.robot0_keyframe_id = odom_1.id;
      message.robot0_id = 0;
      message.robot1_keyframe_id = odom_5.id;
      message.robot1_id = 4;
      
      dx = odom_5.odom.pose.pose.position.x - odom_1.odom.pose.pose.position.x;
      dy = odom_5.odom.pose.pose.position.y - odom_1.odom.pose.pose.position.y;

      message.transform.translation.x = dx;
      message.transform.translation.y = dy;

      if (t5 == 0){
        message.success = true;
        t5 = 1;}
      else
        message.success = false;

      publisher_-> publish(message);
    }}

    private:
    void timer16_callback() const
    {if (t6 == 0){

      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      float dx = 0.0;
      float dy = 0.0;
      
      message.robot0_keyframe_id = odom_1.id;
      message.robot0_id = 0;
      message.robot1_keyframe_id = odom_6.id;
      message.robot1_id = 5;
      
      dx = odom_6.odom.pose.pose.position.x - odom_1.odom.pose.pose.position.x;
      dy = odom_6.odom.pose.pose.position.y - odom_1.odom.pose.pose.position.y;

      message.transform.translation.x = dx;
      message.transform.translation.y = dy;

      if (t6 == 0){
        message.success = true;
        t6 = 1;}
      else
        message.success = false;

      publisher_-> publish(message);
    }}

    private:
    void timer17_callback() const
    {if (t7 == 0){

      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      float dx = 0.0;
      float dy = 0.0;
      
      message.robot0_keyframe_id = odom_1.id;
      message.robot0_id = 0;
      message.robot1_keyframe_id = odom_7.id;
      message.robot1_id = 6;
      
      dx = odom_7.odom.pose.pose.position.x - odom_1.odom.pose.pose.position.x;
      dy = odom_7.odom.pose.pose.position.y - odom_1.odom.pose.pose.position.y;

      message.transform.translation.x = dx;
      message.transform.translation.y = dy;

      if (t7 == 0){
        message.success = true;
        t7 = 1;}
      else
        message.success = false;

      publisher_-> publish(message);
    }}

    private:
    void timer18_callback() const
    {if (t8 == 0){

      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();
      float dx = 0.0;
      float dy = 0.0;
      
      message.robot0_keyframe_id = odom_1.id;
      message.robot0_id = 0;
      message.robot1_keyframe_id = odom_8.id;
      message.robot1_id = 7;
      
      dx = odom_8.odom.pose.pose.position.x - odom_1.odom.pose.pose.position.x;
      dy = odom_8.odom.pose.pose.position.y - odom_1.odom.pose.pose.position.y;

      message.transform.translation.x = dx;
      message.transform.translation.y = dy;

      if (t8 == 0){
        message.success = true;
        t8 = 1;}
      else
        message.success = false;

      publisher_-> publish(message);
    }}

    rclcpp::TimerBase::SharedPtr timer12_;
    rclcpp::TimerBase::SharedPtr timer13_;
    rclcpp::TimerBase::SharedPtr timer14_;
    rclcpp::TimerBase::SharedPtr timer15_;
    rclcpp::TimerBase::SharedPtr timer16_;
    rclcpp::TimerBase::SharedPtr timer17_;
    rclcpp::TimerBase::SharedPtr timer18_;
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









/*    TEMPTATIVE TO CONSIDER ORIENTATION

      tf2::Quaternion q_1(odom_1.odom.pose.pose.orientation.x, odom_1.odom.pose.pose.orientation.y, odom_1.odom.pose.pose.orientation.z, odom_1.odom.pose.pose.orientation.w);
      tf2::Quaternion q_2(odom_2.odom.pose.pose.orientation.x, odom_2.odom.pose.pose.orientation.y, odom_2.odom.pose.pose.orientation.z, odom_2.odom.pose.pose.orientation.w);
      tf2::Matrix3x3 m_1(q_1);
      tf2::Matrix3x3 m_2(q_2);
      double roll_1, pitch_1, yaw_1, roll_2, pitch_2, yaw_2, droll, dpitch, dyaw;
      m_1.getRPY(roll_1, pitch_1, yaw_1);
      m_2.getRPY(roll_2, pitch_2, yaw_2);
      droll = 0.0;
      dpitch = 0.0;
      dyaw = - yaw_2;
      tf2::Quaternion quat;
      quat.setRPY(droll, dpitch, dyaw);
      quat = quat.normalize();

      message.transform.rotation.x = quat.x();
      message.transform.rotation.y = quat.y();
      message.transform.rotation.z = quat.z();
      message.transform.rotation.w = quat.w(); 
*/