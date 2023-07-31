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
#include "std_msgs/msg/int64_multi_array.hpp"

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
std::vector<cslam_common_interfaces::msg::KeyframeOdom> odom_vector = {odom_1, odom_2, odom_3, odom_4, odom_5, odom_6, odom_7, odom_8};

int i = 0;
float candidate[1000][2] = {0};


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

      // Subscription to the Candidate topic
      subscription_ = this->create_subscription<std_msgs::msg::Int64MultiArray>("/candidate_information", 10, std::bind(&LoopClosurePublisher::topic_callback, this, _1));

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

  private:
    void topic_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg) const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();

      for (int j = 0; j < i + 1; j++) {
        float dx = 0.0;
        float dy = 0.0;

        // The vector that defines which robot is Byzantine, adding random noise to the generation of its transformation
        int byzantine_vector[] = {0, 1, 0, 0, 0, 0, 0, 0};

        if (candidate[j][1] == msg->data[1] && candidate[j][0] != msg->data[0] && candidate[j][0] != 0) {
          int random_noise_point_x = -9 + (rand() % 19);
          int random_noise_point_y = -9 + (rand() % 19);

          message.robot0_keyframe_id = odom_vector[msg->data[0]-1].id;
          message.robot0_id = msg->data[0]-1;
          message.robot1_keyframe_id = odom_vector[candidate[j][0]-1].id;
          message.robot1_id = candidate[j][0]-1;
             
          dx = odom_vector[candidate[j][0]-1].odom.pose.pose.position.x - odom_vector[msg->data[0]-1].odom.pose.pose.position.x;
          dy = odom_vector[candidate[j][0]-1].odom.pose.pose.position.y - odom_vector[msg->data[0]-1].odom.pose.pose.position.y;

          message.transform.translation.x = dx + byzantine_vector[msg->data[0]-1] * random_noise_point_x;
          message.transform.translation.y = dy + byzantine_vector[msg->data[0]-1] * random_noise_point_y;

          message.success = true;

          RCLCPP_INFO_STREAM(this->get_logger(), "Publishing Loop Closure from robot " << msg->data[0] << " on robot " << candidate[j][0] << " at scene " << msg->data[1]);
          publisher_-> publish(message);
        }
        if (j == i) {
          // i is the INDEX in the database, [i][0] is the ROBOT_ID, [i][1] is the SCENE_ID
          candidate[i][0] = msg->data[0];
          candidate[i][1] = msg->data[1];
          RCLCPP_INFO_STREAM(this->get_logger(), "New Candidate descriptor. Updated database: " <<  i << " - [" << candidate[i][0] << "][" << candidate[i][1] <<"]");
        }
      }

      i++;
    }

    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription1_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription2_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription3_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription4_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription5_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription6_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription7_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription8_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr subscription_;
    
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
