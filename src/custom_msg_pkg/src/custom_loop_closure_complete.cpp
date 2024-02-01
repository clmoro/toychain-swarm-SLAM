#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>
#include <bits/stdc++.h>

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
int e = 1;

// Arrays
float candidate[10000][5] = {0};
float loop_closure[10000][13] = {0};
int candidate_history[10000][10000] = {0};

class LoopClosurePublisher : public rclcpp::Node
{  
  public:
    LoopClosurePublisher()
    : Node("loop_closure_publisher"), count_(0)
    {

// SUBSCRIPTIONS

      // Subscriptions to Custom Odometry
      subscription1_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r0/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic1_callback, this, _1));
      subscription2_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r1/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic2_callback, this, _1));
      subscription3_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r2/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic3_callback, this, _1));
      subscription4_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r3/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic4_callback, this, _1));
      subscription5_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r4/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic5_callback, this, _1));
      subscription6_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r5/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic6_callback, this, _1));
      subscription7_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r6/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic7_callback, this, _1));
      subscription8_ = this->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>("/r7/cslam/keyframe_odom_ideal", 100, std::bind(&LoopClosurePublisher::topic8_callback, this, _1));

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

<<<<<<< HEAD
=======
      // Subscription to the Blockchain topic
      subscription_blockchain_ = this->create_subscription<std_msgs::msg::Int64MultiArray>("/blockchain_approved_transformation", 100, std::bind(&LoopClosurePublisher::topic_blockchain_callback, this, _1));


>>>>>>> 42dfe2a1a3bbb8073505530247bd1321cc1f37dd
// PUBLISHERS

      // Candidate for blockchain
      publisher_cand_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_candidate", 100);

      // Transformation for blockchain
      publisher_transf_C0 = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_C0", 100);
      publisher_transf_C1 = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_C1", 100);
      publisher_transf_C2 = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_C2", 100);
      publisher_transf_C3 = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_C3", 100);
      publisher_transf_C4 = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_C4", 100);
      publisher_transf_C5 = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_C5", 100);
      publisher_transf_R0 = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_R0", 100);
      publisher_transf_R1 = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_R1", 100);
      publisher_transf_R2 = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_R2", 100);
      publisher_transf_R3 = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_R3", 100);
      publisher_transf_R4 = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_R4", 100);
      publisher_transf_R5 = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_R5", 100);
<<<<<<< HEAD
      publisher_transf_C0t = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_C0t", 100);
      publisher_transf_C1t = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_C1t", 100);
      publisher_transf_C2t = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_C2t", 100);
      publisher_transf_C3t = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_C3t", 100);
      publisher_transf_C4t = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_C4t", 100);
      publisher_transf_C5t = this->create_publisher<std_msgs::msg::Float64MultiArray>("/blockchain_transformation_C5t", 100);
=======
>>>>>>> 42dfe2a1a3bbb8073505530247bd1321cc1f37dd

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
      auto message_transf_C0 = std_msgs::msg::Float64MultiArray();
      auto message_transf_C1 = std_msgs::msg::Float64MultiArray();
      auto message_transf_C2 = std_msgs::msg::Float64MultiArray();
      auto message_transf_C3 = std_msgs::msg::Float64MultiArray();
      auto message_transf_C4 = std_msgs::msg::Float64MultiArray();
      auto message_transf_C5 = std_msgs::msg::Float64MultiArray();
      auto message_transf_R0 = std_msgs::msg::Float64MultiArray();
      auto message_transf_R1 = std_msgs::msg::Float64MultiArray();
      auto message_transf_R2 = std_msgs::msg::Float64MultiArray();
      auto message_transf_R3 = std_msgs::msg::Float64MultiArray();
      auto message_transf_R4 = std_msgs::msg::Float64MultiArray();
      auto message_transf_R5 = std_msgs::msg::Float64MultiArray();
<<<<<<< HEAD
      auto message_transf_C0t = std_msgs::msg::Float64MultiArray();
      auto message_transf_C1t = std_msgs::msg::Float64MultiArray();
      auto message_transf_C2t = std_msgs::msg::Float64MultiArray();
      auto message_transf_C3t = std_msgs::msg::Float64MultiArray();
      auto message_transf_C4t = std_msgs::msg::Float64MultiArray();
      auto message_transf_C5t = std_msgs::msg::Float64MultiArray();
=======
>>>>>>> 42dfe2a1a3bbb8073505530247bd1321cc1f37dd

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
            if (candidate[j][0] == msg->data[k] && candidate[z][0] == msg->data[8] && candidate[j][1] == candidate[z][1] && msg->data[k] < msg->data[8] && (candidate_history[z][j] == 0 || candidate_history[j][z] == 0)) {

<<<<<<< HEAD
              float dx[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
              float dy[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
=======
              float dx[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
              float dy[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
>>>>>>> 42dfe2a1a3bbb8073505530247bd1321cc1f37dd
              int temp_j = -1;
              int temp_z = -1;
              int tmp = 0;

              // Scan candidate[][] to compare its values of j,z with all the others
              for (int p = 0; p < id_candidate + 1; p++) {            
                // Check complete triangle
                if (candidate_history[j][p] == 1 && candidate_history[p][z] == 1 && candidate_history[z][j] == 0 && p != z && p != j) {
                  temp_j = j;
                  temp_z = z;
                  tmp = 1;
                  break;
                }
              }
              if (tmp == 0) {
                for (int p = 0; p < id_candidate + 1; p++) {            
                  // Check complete triangle
                  if (candidate_history[z][p] == 1 && candidate_history[p][j] == 1 && candidate_history[j][z] == 0 && p != z && p != j) {
                    temp_j = z;
                    temp_z = j;
                    tmp = 1;
                    break;
                  }
                }
              }
              if (tmp == 0) {
                for (int p = 0; p < id_candidate + 1; p++) {            
                  // Candidate j was used as a Sender, it will be a Receiver
                  if (candidate_history[j][p] == 1 && candidate_history[z][j] == 0 && p != z) {
                    temp_j = j;
                    temp_z = z;
                    tmp = 1;
                    break;
                  }
                }
              }
              if (tmp == 0) {
                for (int p = 0; p < id_candidate + 1; p++) {
                  // Candidate z was used as a Receiver, it will be a Sender
                  if (candidate_history[p][z] == 1 && candidate_history[z][j] == 0 && p != j) {
                    temp_j = j;
                    temp_z = z;
                    tmp = 1;
                    break;
                  }
                }
              }
              if (tmp == 0) {
                for (int p = 0; p < id_candidate + 1; p++) {
                  // Candidate z was used as a Sender, it will be a Receiver
                  if (candidate_history[z][p] == 1 && candidate_history[j][z] == 0 && p != j) {
                    temp_j = z;
                    temp_z = j;
                    tmp = 1;
                    break;
                  }
                }
              }
              if (tmp == 0) {
                for (int p = 0; p < id_candidate + 1; p++) {
                  // Candidate j was used as a Receiver, it will be a Sender
                  if (candidate_history[p][j] == 1 && candidate_history[j][z] == 0 && p != z) {
                    temp_j = z;
                    temp_z = j;
                    tmp = 1;
                    break;
                  }
                }
              }
              if (tmp == 0) {
                // Candidate was not used yet, use it as a Receiver or Sender randomly
                if (randombin == 0 && candidate_history[z][j] == 0) {
                  temp_j = j;
                  temp_z = z;
                }
                else if (randombin == 1 && candidate_history[j][z] == 0) { 
                  temp_j = z;
                  temp_z = j;
                }
                else if (candidate_history[z][j] == 0) {
                  temp_j = j;
                  temp_z = z;
                }
                else if (candidate_history[j][z] == 0) {
                  temp_j = z;
                  temp_z = j;
                }
              }

              // The vector that defines which robot is Byzantine, adding random noise to the generation of its transformation
              int byzantine_vector_C0[] = {0, 0, 0, 0, 0, 0, 0, 0};
              int byzantine_vector_C1[] = {0, 0, 0, 0, 0, 0, 0, 1};
              int byzantine_vector_C2[] = {0, 0, 0, 0, 0, 0, 1, 1};
              int byzantine_vector_C3[] = {0, 0, 0, 0, 0, 1, 1, 1};
              int byzantine_vector_C4[] = {0, 0, 0, 0, 1, 1, 1, 1};
              int byzantine_vector_C5[] = {0, 0, 0, 1, 1, 1, 1, 1};
              int byzantine_vector_R0[] = {0, 0, 0, 0, 0, 0, 0, 0};
              int byzantine_vector_R1[] = {0, 0, 0, 0, 0, 0, 0, 1};
              int byzantine_vector_R2[] = {0, 0, 0, 0, 0, 0, 1, 1};
              int byzantine_vector_R3[] = {0, 0, 0, 0, 0, 1, 1, 1};
              int byzantine_vector_R4[] = {0, 0, 0, 0, 1, 1, 1, 1};
              int byzantine_vector_R5[] = {0, 0, 0, 1, 1, 1, 1, 1};
              std::default_random_engine rand_number;
              std::uniform_real_distribution<double> distribution(-9.0,9.0);
              float noise_point_x = distribution(rand_number);
              float noise_point_y = distribution(rand_number);
             
              dx[0] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_C0[int(candidate[temp_z][0]-1)] * 9.0;
              dy[0] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_C0[int(candidate[temp_z][0]-1)] * 9.0;
              dx[1] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_C1[int(candidate[temp_z][0]-1)] * 9.0;
              dy[1] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_C1[int(candidate[temp_z][0]-1)] * 9.0;
              dx[2] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_C2[int(candidate[temp_z][0]-1)] * 9.0;
              dy[2] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_C2[int(candidate[temp_z][0]-1)] * 9.0;
              dx[3] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_C3[int(candidate[temp_z][0]-1)] * 9.0;
              dy[3] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_C3[int(candidate[temp_z][0]-1)] * 9.0;
              dx[4] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_C4[int(candidate[temp_z][0]-1)] * 9.0;
              dy[4] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_C4[int(candidate[temp_z][0]-1)] * 9.0;
              dx[5] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_C5[int(candidate[temp_z][0]-1)] * 9.0;
              dy[5] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_C5[int(candidate[temp_z][0]-1)] * 9.0;
              dx[6] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_R0[int(candidate[temp_z][0]-1)] * noise_point_x;
              dy[6] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_R0[int(candidate[temp_z][0]-1)] * noise_point_y;
              dx[7] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_R1[int(candidate[temp_z][0]-1)] * noise_point_x;
              dy[7] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_R1[int(candidate[temp_z][0]-1)] * noise_point_y;
              dx[8] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_R2[int(candidate[temp_z][0]-1)] * noise_point_x;
              dy[8] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_R2[int(candidate[temp_z][0]-1)] * noise_point_y;
              dx[9] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_R3[int(candidate[temp_z][0]-1)] * noise_point_x;
              dy[9] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_R3[int(candidate[temp_z][0]-1)] * noise_point_y;
              dx[10] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_R4[int(candidate[temp_z][0]-1)] * noise_point_x;
              dy[10] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_R4[int(candidate[temp_z][0]-1)] * noise_point_y;
              dx[11] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_R5[int(candidate[temp_z][0]-1)] * noise_point_x;
              dy[11] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_R5[int(candidate[temp_z][0]-1)] * noise_point_y;
<<<<<<< HEAD
              dx[12] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_C0[int(candidate[temp_z][0]-1)] * 0.9;
              dy[12] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_C0[int(candidate[temp_z][0]-1)] * 0.9;
              dx[13] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_C1[int(candidate[temp_z][0]-1)] * 0.9;
              dy[13] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_C1[int(candidate[temp_z][0]-1)] * 0.9;
              dx[14] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_C2[int(candidate[temp_z][0]-1)] * 0.9;
              dy[14] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_C2[int(candidate[temp_z][0]-1)] * 0.9;
              dx[15] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_C3[int(candidate[temp_z][0]-1)] * 0.9;
              dy[15] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_C3[int(candidate[temp_z][0]-1)] * 0.9;
              dx[16] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_C4[int(candidate[temp_z][0]-1)] * 0.9;
              dy[16] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_C4[int(candidate[temp_z][0]-1)] * 0.9;
              dx[17] = candidate[temp_j][2] - candidate[temp_z][2] + byzantine_vector_C5[int(candidate[temp_z][0]-1)] * 0.9;
              dy[17] = candidate[temp_j][3] - candidate[temp_z][3] + byzantine_vector_C5[int(candidate[temp_z][0]-1)] * 0.9;
=======
>>>>>>> 42dfe2a1a3bbb8073505530247bd1321cc1f37dd

              // The message_transf is a vector of what I want to put on the blockchain: [descriptor, ROBOT_ID_R, odom1x_R, odom1y_R, keyframe1_R, ROBOT_ID_S, odom1x_S, odom1y_S, keyframe1_S, dx, dy, SCENE]
              // The univoque descriptors are "SCENE_ID + row_candidate_index_Receiver + ROBOT_ID_R + row_candidate_index_Sender + ROBOT_ID_S"
              std::string str_descriptor_SCENE = std::to_string(candidate[temp_j][1]);
              std::string str_descriptor_R = std::to_string(temp_j + 1) + std::to_string(candidate[temp_j][0]);
              std::string str_descriptor_S = std::to_string(temp_z + 1) + std::to_string(candidate[temp_z][0]);
              std::string str_descriptor = str_descriptor_SCENE + str_descriptor_R + str_descriptor_S;
              str_descriptor.erase(remove(str_descriptor.begin(), str_descriptor.end(), '0'), str_descriptor.end());
              str_descriptor.erase(remove(str_descriptor.begin(), str_descriptor.end(), '.'), str_descriptor.end());
              int descriptor = stoi(str_descriptor);

              RCLCPP_INFO_STREAM(this->get_logger(), "New Loop Closure from robot " << candidate[temp_z][0] << " on robot " << candidate[temp_j][0] << " at scene " << candidate[temp_z][1] << " with descriptor " << str_descriptor);
              // RCLCPP_INFO_STREAM(this->get_logger(), message.transform.translation.x << " " << message.transform.translation.y << " " << message.robot0_keyframe_id << " " << message.robot0_id << " " << message.robot1_keyframe_id << " " << message.robot1_id);

              message_transf_C0.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[0], dy[0], candidate[temp_z][1]};
              message_transf_C1.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[1], dy[1], candidate[temp_z][1]};
              message_transf_C2.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[2], dy[2], candidate[temp_z][1]};
              message_transf_C3.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[3], dy[3], candidate[temp_z][1]};
              message_transf_C4.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[4], dy[4], candidate[temp_z][1]};
              message_transf_C5.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[5], dy[5], candidate[temp_z][1]};
              message_transf_R0.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[6], dy[6], candidate[temp_z][1]};
              message_transf_R1.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[7], dy[7], candidate[temp_z][1]};
              message_transf_R2.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[8], dy[8], candidate[temp_z][1]};
              message_transf_R3.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[9], dy[9], candidate[temp_z][1]};
              message_transf_R4.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[10], dy[10], candidate[temp_z][1]};
              message_transf_R5.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[11], dy[11], candidate[temp_z][1]};
<<<<<<< HEAD
              message_transf_C0t.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[12], dy[12], candidate[temp_z][1]};
              message_transf_C1t.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[13], dy[13], candidate[temp_z][1]};
              message_transf_C2t.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[14], dy[14], candidate[temp_z][1]};
              message_transf_C3t.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[15], dy[15], candidate[temp_z][1]};
              message_transf_C4t.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[16], dy[16], candidate[temp_z][1]};
              message_transf_C5t.data = {descriptor, candidate[temp_j][0], candidate[temp_j][2], candidate[temp_j][3], candidate[temp_j][4], candidate[temp_z][0], candidate[temp_z][2], candidate[temp_z][3], candidate[temp_z][4], dx[17], dy[17], candidate[temp_z][1]};
=======
>>>>>>> 42dfe2a1a3bbb8073505530247bd1321cc1f37dd
              publisher_transf_C0-> publish(message_transf_C0);
              publisher_transf_C1-> publish(message_transf_C1);
              publisher_transf_C2-> publish(message_transf_C2);
              publisher_transf_C3-> publish(message_transf_C3);
              publisher_transf_C4-> publish(message_transf_C4);
              publisher_transf_C5-> publish(message_transf_C5);
              publisher_transf_R0-> publish(message_transf_R0);
              publisher_transf_R1-> publish(message_transf_R1);
              publisher_transf_R2-> publish(message_transf_R2);
              publisher_transf_R3-> publish(message_transf_R3);
              publisher_transf_R4-> publish(message_transf_R4);
              publisher_transf_R5-> publish(message_transf_R5);
<<<<<<< HEAD
              publisher_transf_C0t-> publish(message_transf_C0t);
              publisher_transf_C1t-> publish(message_transf_C1t);
              publisher_transf_C2t-> publish(message_transf_C2t);
              publisher_transf_C3t-> publish(message_transf_C3t);
              publisher_transf_C4t-> publish(message_transf_C4t);
              publisher_transf_C5t-> publish(message_transf_C5t);
=======
>>>>>>> 42dfe2a1a3bbb8073505530247bd1321cc1f37dd

              loop_closure[id_loop_closure][0] = descriptor;
              loop_closure[id_loop_closure][1] = candidate[temp_j][0];
              loop_closure[id_loop_closure][2] = candidate[temp_j][2];
              loop_closure[id_loop_closure][3] = candidate[temp_j][3];
              loop_closure[id_loop_closure][4] = candidate[temp_j][4];
              loop_closure[id_loop_closure][5] = candidate[temp_z][0];
              loop_closure[id_loop_closure][6] = candidate[temp_z][2];
              loop_closure[id_loop_closure][7] = candidate[temp_z][3];
              loop_closure[id_loop_closure][8] = candidate[temp_z][4];
              loop_closure[id_loop_closure][9] = dx[0];
              loop_closure[id_loop_closure][10] = dy[0];
              loop_closure[id_loop_closure][11] = candidate[temp_j][1];

              id_loop_closure++;

              // A binary copy of the candidate table^2 to record the LCs creation, every candidate row (rows: Sender) keeps track of each other candidate row (columns: Receiver) interaction in the column
              candidate_history[temp_z][temp_j] = 1;

            }
          
          }

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
<<<<<<< HEAD
=======
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr subscription_blockchain_;
>>>>>>> 42dfe2a1a3bbb8073505530247bd1321cc1f37dd
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_cand_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_C0;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_C1;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_C2;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_C3;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_C4;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_C5;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_R0;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_R1;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_R2;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_R3;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_R4;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_R5;
<<<<<<< HEAD
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_C0t;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_C1t;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_C2t;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_C3t;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_C4t;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_transf_C5t;
=======
>>>>>>> 42dfe2a1a3bbb8073505530247bd1321cc1f37dd
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoopClosurePublisher>());
  rclcpp::shutdown();
  return 0;
}

