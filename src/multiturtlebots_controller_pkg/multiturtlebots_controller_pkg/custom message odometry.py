# Author: Angelo Moroncelli
# ROS Version: ROS 2 Foxy Fitzroy
 
# Python math library
import math
 
# ROS client library for Python
import rclpy
 
# Used to create nodes
from rclpy.node import Node
 
# Twist is linear and angular velocity
from geometry_msgs.msg import Twist 
 
# Position, orientation, linear velocity, angular velocity
from nav_msgs.msg import Odometry
 
# Handles laser distance scan to detect obstacles
from sensor_msgs.msg import LaserScan
 
# Used for laser scan
from rclpy.qos import qos_profile_sensor_data
 
# Enable use of std_msgs/Float64MultiArray message
from std_msgs.msg import Float64MultiArray 
 
# Scientific computing library for Python
import numpy as np

# Random library
from random import randrange

# Handle uint32 messages
from std_msgs.msg import uint32

# Transform between two keyframes
from geometry_msgs.msg import Transform

# Import custom message
from cslam_common_interfaces.msg import KeyframeOdom
 
class Noisy_Odometry (Node):
  """
  Class constructor to set up the node
  """
  def __init__(self):
 
    ############## INITIALIZE ROS PUBLISHERS AND SUBSCRIBERS ######
    super().__init__('Noisy_Odometry')
 
    # Create a subscriber for the odometry messages of every robot
    # The type of message is nav_msgs/Odometry (i.e. position and orientation of the robot)
    self.odom_subscriber = self.create_subscription(
                           Odometry,
                           '/bot1/odom',
                           self.odom_callback_1,
                           10)

    self.odom_subscriber = self.create_subscription(
                           Odometry,
                           '/bot2/odom',
                           self.odom_callback_2,
                           10)

    self.odom_subscriber = self.create_subscription(
                           Odometry,
                           '/bot3/odom',
                           self.odom_callback_3,
                           10)

    self.odom_subscriber = self.create_subscription(
                           Odometry,
                           '/bot4/odom',
                           self.odom_callback_4,
                           10)
 
    # Create a publisher
    # This node publishes random generated loop closures 
    # The type is a custom message: KeyframeOdom.msg
    self.publisher_odom = self.create_publisher(
                          KeyframeOdom, 
                          'cslam/keyframe_odom', 
                          1000)
 
  def odom_callback_1(self, msg):
    """
    Receive the odometry information and construct the custom message with the noise. 
    """                    
    
    id_custom_vector = [1, msg]
    self.publish_custom_odometry(id_custom_vector)

  def odom_callback_2(self, msg):                   
    
    id_custom_vector = [2, msg]
    self.publish_custom_odometry(id_custom_vector)

  def odom_callback_3(self, msg):
    
    id_custom_vector = [3, msg]
    self.publish_custom_odometry(id_custom_vector)

  def odom_callback_4(self, msg):                  
    
    id_custom_vector = [4, msg]
    self.publish_custom_odometry(id_custom_vector)

  def publish_custom_odometry(self, custom_vector):
    """
    Publish the custom message
    """
    msg = KeyframeOdom()
    msg.data = custom_vector
    self.publisher_odom.publish(msg)
 
def main(args=None):
    """
    Entry point for the program.
    """
    # Initialize rclpy library
    rclpy.init(args=args)
 
    # Create the node
    noisy_odometry = Noisy_Odometry()
 
    # Spin the node so the callback function is called.
    # Pull messages from any topics this node is subscribed to.
    # Publish any pending messages to the topics.
    rclpy.spin(noisy_odometry)
 
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    noisy_odometry.destroy_node()
     
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()