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
int id_candidate = 0;
int id_loop_closure = 0;

// Arrays
float candidate[10000][5] = {0};
float loop_closure[10000][13] = {0};
int candidate_history[10000][8] = {0};

class LoopClosurePublisher : public rclcpp::Node
{  
  public:
    LoopClosurePublisher()
    : Node("loop_closure_publisher"), count_(0)
    {

// SUBSCRIPTIONS

      // Subscriptions to Custom Odometry
      subscription1_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r0/cslam/keyframe_odom", 100, std::bind(&LoopClosurePublisher::topic1_callback, this, _1));
      subscription2_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r1/cslam/keyframe_odom", 100, std::bind(&LoopClosurePublisher::topic2_callback, this, _1));
      subscription3_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r2/cslam/keyframe_odom", 100, std::bind(&LoopClosurePublisher::topic3_callback, this, _1));
      subscription4_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r3/cslam/keyframe_odom", 100, std::bind(&LoopClosurePublisher::topic4_callback, this, _1));
      subscription5_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r4/cslam/keyframe_odom", 100, std::bind(&LoopClosurePublisher::topic5_callback, this, _1));
      subscription6_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r5/cslam/keyframe_odom", 100, std::bind(&LoopClosurePublisher::topic6_callback, this, _1));
      subscription7_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r6/cslam/keyframe_odom", 100, std::bind(&LoopClosurePublisher::topic7_callback, this, _1));
      subscription8_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r7/cslam/keyframe_odom", 100, std::bind(&LoopClosurePublisher::topic8_callback, this, _1));

      // Subscriptions to Adjacency Matrix
      sub_peers_1_ = this->create_subscription<std_msgs::msg::Int64MultiArray>("/peering_1", 10, std::bind(&LoopClosurePublisher::sub_peers_callback, this, _1));
      sub_peers_2_ = this->create_subscription<std_msgs::msg::Int64MultiArray>("/peering_2", 10, std::bind(&LoopClosurePublisher::sub_peers_callback, this, _1));
      sub_peers_3_ = this->create_subscription<std_msgs::msg::Int64MultiArray>("/peering_3", 10, std::bind(&LoopClosurePublisher::sub_peers_callback, this, _1));
      sub_peers_4_ = this->create_subscription<std_msgs::msg::Int64MultiArray>("/peering_4", 10, std::bind(&LoopClosurePublisher::sub_peers_callback, this, _1));
      sub_peers_5_ = this->create_subscription<std_msgs::msg::Int64MultiArray>("/peering_5", 10, std::bind(&LoopClosurePublisher::sub_peers_callback, this, _1));
      sub_peers_6_ = this->create_subscription<std_msgs::msg::Int64MultiArray>("/peering_6", 10, std::bind(&LoopClosurePublisher::sub_peers_callback, this, _1));
      sub_peers_7_ = this->create_subscription<std_msgs::msg::Int64MultiArray>("/peering_7", 10, std::bind(&LoopClosurePublisher::sub_peers_callback, this, _1));
      sub_peers_8_ = this->create_subscription<std_msgs::msg::Int64MultiArray>("/peering_8", 10, std::bind(&LoopClosurePublisher::sub_peers_callback, this, _1));

      // Subscription to the Candidate topic
      subscription_ = this->create_subscription<std_msgs::msg::Int64MultiArray>("/candidate_information", 10, std::bind(&LoopClosurePublisher::topic_candidate_callback, this, _1));

      // Subscription to the Blockchain topic
      subscription_blockchain_ = this->create_subscription<std_msgs::msg::Int64MultiArray>("/blockchain_approved_transformation", 10, std::bind(&LoopClosurePublisher::topic_blockchain_callback, this, _1));


// PUBLISHERS

      // Candidate for blockchain
      publisher_cand_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_candidate", 100);

      // Transformation for blockchain
      publisher_transf_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation", 100);

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
    void topic_candidate_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg) const
    {
      auto message_cand = std_msgs::msg::Float64MultiArray();

      // id_candidate is the INDEX in the database, [id_candidate][0] is the ROBOT_ID, [id_candidate][1] is the SCENE_ID, [id_candidate][2] is the ODOMETRY X, [id_candidate][3] is the ODOMETRY Y, [id_candidate][4] is the KEYFRAME_ID
      candidate[id_candidate][0] = msg->data[0];
      candidate[id_candidate][1] = msg->data[1];
      candidate[id_candidate][2] = odom_vector[candidate[id_candidate][0]-1].odom.pose.pose.position.x;
      candidate[id_candidate][3] = odom_vector[candidate[id_candidate][0]-1].odom.pose.pose.position.y;
      candidate[id_candidate][4] = odom_vector[candidate[id_candidate][0]-1].id;
      RCLCPP_INFO_STREAM(this->get_logger(), "New Candidate descriptor. Updated database: " <<  id_candidate + 1 << " - [" << candidate[id_candidate][0] << "][" << candidate[id_candidate][1] <<"]" << "[" << candidate[id_candidate][2] << "][" << candidate[id_candidate][3] << "][" << candidate[id_candidate][4] <<"]");

      // The message_cand is: [candidate_id, candidate[id_candidate][0], candidate[id_candidate][1], candidate[id_candidate][2], candidate[id_candidate][3], candidate[id_candidate][4]]
      message_cand.data = {id_candidate+1, candidate[id_candidate][0],candidate[id_candidate][1],candidate[id_candidate][2],candidate[id_candidate][3],candidate[id_candidate][4]};
      publisher_cand_-> publish(message_cand);

      id_candidate++;
    }

  private:
    void sub_peers_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg) const
    {
      auto message_transf = std_msgs::msg::Float64MultiArray();

      // To randomize who generate the loop closure between the two (as in Swarm-SLAM)
      int randombin = rand() % 2;

      // Loop to scan the new peers
      for (int k = 0; (msg->data[k] != -1) && (k < 8) ; k++) {

        RCLCPP_INFO_STREAM(this->get_logger(), "New meeting " << msg->data[8] << " with " << msg->data[k]);

        // Loop to scan the candidates of the peer
        for (int j = 0; j < id_candidate + 1; j++) {

          // Loop to scan the candidates of the publisher
          for (int z = 0; z < id_candidate + 1; z++) {

            // Check msg->data[k] < msg->data[8], is to execute only once the function, since it's called from both robot every publication
            if (candidate[j][0] == msg->data[k] && candidate[z][0] == msg->data[8] && candidate[j][1] == candidate[z][1] && msg->data[k] < msg->data[8] && ((candidate_history[z][msg->data[k]-1] == 0 && randombin == 0) || (candidate_history[j][msg->data[8]-1] == 0 && randombin == 1))) {

              std::string str_descriptor_R = "";
              std::string str_descriptor_S = "";
              int descriptor = 0;
              float dx = 0.0;
              float dy = 0.0;
              int temp_j = -1;
              int temp_z = -1;

              if (randombin == 0) {
                temp_j = j;
                temp_z = z;
              }
              else {
                temp_j = z;
                temp_z = j;
              }

              // The vector that defines which robot is Byzantine, adding random noise to the generation of its transformation
              //int byzantine_vector[] = {0, 0, 0, 0, 0, 0, 0, 0};
              //int random_noise_point_x = -9 + (rand() % 19);
              //int random_noise_point_y = -9 + (rand() % 19);
             
              dx = candidate[temp_j][2] - candidate[temp_z][2]; //+ byzantine_vector[msg->data[0]-1] * random_noise_point_x;
              dy = candidate[temp_j][3] - candidate[temp_z][3]; //+ byzantine_vector[msg->data[0]-1] * random_noise_point_y;

              RCLCPP_INFO_STREAM(this->get_logger(), "New Loop Closure from robot " << candidate[temp_z][0] << " on robot " << candidate[temp_j][0] << " at scene " << candidate[temp_z][1]);
              // RCLCPP_INFO_STREAM(this->get_logger(), message.transform.translation.x << " " << message.transform.translation.y << " " << message.robot0_keyframe_id << " " << message.robot0_id << " " << message.robot1_keyframe_id << " " << message.robot1_id);

              // The message_transf is a vector of what I want to put on the blockchain: [descriptor, ROBOT_ID_R, odom1x_R, odom1y_R, keyframe1_R, ROBOT_ID_S, odom1x_S, odom1y_S, keyframe1_S, dx, dy, SCENE]
              // The univoque descriptors are "row_candidate_table_Receiver/Sender + SCENE" and "ID_candidate_Sender(row candidate) + SCENE"
              str_descriptor_R = std::to_string(temp_j) + std::to_string(candidate[temp_j][1]);
              str_descriptor_S = std::to_string(temp_z) + std::to_string(candidate[temp_z][1]);
              descriptor = stoi(str_descriptor_R + str_descriptor_S);

              message_transf.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx, dy, candidate[temp_j][1]};
              publisher_transf_-> publish(message_transf);

              loop_closure[id_loop_closure][0] = descriptor;
              loop_closure[id_loop_closure][1] = candidate[temp_j][0];
              loop_closure[id_loop_closure][2] = candidate[temp_j][2];
              loop_closure[id_loop_closure][3] = candidate[temp_j][3];
              loop_closure[id_loop_closure][4] = candidate[temp_j][4];
              loop_closure[id_loop_closure][5] = candidate[temp_z][0];
              loop_closure[id_loop_closure][6] = candidate[temp_z][2];
              loop_closure[id_loop_closure][7] = candidate[temp_z][3];
              loop_closure[id_loop_closure][8] = candidate[temp_z][4];
              loop_closure[id_loop_closure][9] = dx;
              loop_closure[id_loop_closure][10] = dy;
              loop_closure[id_loop_closure][11] = candidate[temp_j][1];

              id_loop_closure++;

              // A binary copy of the candidate table to record the LCs creation, every candidate row keeps track of each other robot interaction in the column, N columns
              int receiver_pos = static_cast<int>(candidate[temp_j][0]);
              candidate_history[temp_z][receiver_pos-1] = 1;

            }
          
          }

        }

      }

    }

  private:
    void topic_blockchain_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg) const
    {
      auto message = cslam_common_interfaces::msg::InterRobotLoopClosure();

      // All the approved Loop Closures will be published, msg->data[0] is the ID of the LC approved, in this way I know which ones to take from the local database (the "loop_closure" table)
      // Remember: robot0_id (Sender) and robot1_id (Receiver) need ROBOT_ID - 1 

      for (int x = 0; x < id_loop_closure + 1; x++) {

        if (loop_closure[x][0] == msg->data[0]) {

          message.robot0_keyframe_id = loop_closure[x][8];
          message.robot0_id = loop_closure[x][5] - 1;
          message.robot1_keyframe_id = loop_closure[x][4];
          message.robot1_id = loop_closure[x][1] - 1;
          message.transform.translation.x = loop_closure[x][9];
          message.transform.translation.y = loop_closure[x][10];
          message.success = true;

          publisher_-> publish(message);

          // Update the "local" database, it means that the loop closure was validated and published
          loop_closure[x][12] = 1;

        }

      }

    }

    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription1_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription2_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription3_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription4_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription5_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription6_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription7_;
    rclcpp::Subscription<cslam_common_interfaces::msg::KeyframeOdom>::SharedPtr subscription8_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr sub_peers_1_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr sub_peers_2_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr sub_peers_3_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr sub_peers_4_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr sub_peers_5_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr sub_peers_6_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr sub_peers_7_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr sub_peers_8_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr subscription_blockchain_;
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_cand_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_;
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

