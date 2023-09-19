# Author: Angelo Moroncelli
# ROS Version: ROS 2 Foxy Fitzroy
 
############## IMPORT LIBRARIES #################

# Python math library
import math 
 
# ROS client library for Python
import rclpy 
 
# Enables pauses in the execution of code
from time import sleep 
 
# Used to create nodes
from rclpy.node import Node
 
# Enables the use of the string message type
from std_msgs.msg import String 
 
# Twist is linear and angular velocity
from geometry_msgs.msg import Twist     
                     
# Handles LaserScan messages to sense distance to obstacles (i.e. walls)        
from sensor_msgs.msg import LaserScan    
 
# Handle Pose messages
from geometry_msgs.msg import Pose 
 
# Handle float64 arrays
from std_msgs.msg import Float64MultiArray
                     
# Handles quality of service for LaserScan data
from rclpy.qos import qos_profile_sensor_data 
 
# Scientific computing library
import numpy as np 
 
# Random library
from random import randrange

class Controller_1(Node):
  """
  Create a Controller class, which is a subclass of the Node 
  class for ROS2.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    ##################### ROS SETUP ####################################################
    # Initiate the Node class's constructor and give it a name
    super().__init__('Controller_1')
 
    # Create a subscriber
    # This node subscribes to messages of type Float64MultiArray  
    # over a topic named: /bot1/state_est
    # The message represents the current estimated state:
    #   [x, y, yaw]
    # The callback function is called as soon as a message 
    # is received.
    # The maximum number of queued messages is 10.
    self.subscription = self.create_subscription(
                        Float64MultiArray,
                        '/bot1/state_est',
                        self.state_estimate_callback,
                        10)
    self.subscription  # prevent unused variable warning
 
    # Create a subscriber
    # This node subscribes to messages of type 
    # sensor_msgs/LaserScan     
    self.scan_subscriber = self.create_subscription(
                           LaserScan,
                           '/bot1/scan',
                           self.scan_callback,
                           qos_profile=qos_profile_sensor_data)
                            
    # Create a publisher
    # This node publishes the desired linear and angular velocity of the robot (in the
    # robot chassis coordinate frame) to the /bot1/cmd_vel topic. Using the diff_drive
    # plugin enables the robot model to read this /bot1/cmd_vel topic and execute
    # the motion accordingly.
    self.publisher_ = self.create_publisher(
                      Twist, 
                      '/bot1/cmd_vel', 
                      10)
 
    # Initialize the LaserScan sensor readings to some large value
    # Values are in meters.
    self.left_dist = 999999.9 # Left
    self.leftfront_dist = 999999.9 # Left-front
    self.front_dist = 999999.9 # Front
    self.rightfront_dist = 999999.9 # Right-front
    self.right_dist = 999999.9 # Right
 
    ################### ROBOT CONTROL PARAMETERS ##################

    # Counter for the turning times of the random walker when obstacles
    self.counter_r = 0
    self.counter_l = 0

    # Maximum forward speed of the robot in meters per second
    # Any faster than this and the robot risks falling over s=0.025.
    self.forward_speed = 0.1
 
    # Current position and orientation of the robot in the global 
    # reference frame
    self.current_x = 3.0
    self.current_y = 0.0
    self.current_yaw = 0.0
 
    ############# WALL FOLLOWING PARAMETERS #######################     

    self.turning_speed = 1.0  # Turn
 
    # Wall following distance threshold.
    # We want to try to keep within this distance from the wall.
    self.dist_thresh_wf = 0.50 # in meters  
 
  def state_estimate_callback(self, msg):
    """
    Extract the position and orientation data. 
    This callback is called each time
    a new message is received on the '/bot1/state_est' topic
    """
    # Update the current estimated state in the global reference frame
    curr_state = msg.data
    self.current_x = curr_state[0]
    self.current_y = curr_state[1]
    self.current_yaw = curr_state[2]
 
    # Command the robot to keep following the wall      
    self.follow_wall()
 
  def scan_callback(self, msg):
    """
    This method gets called every time a LaserScan message is 
    received on the '/bot1/laser/out' topic 
    """
    # Read the laser scan data that indicates distances
    # to obstacles (e.g. wall) in meters and extract
    # 5 distinct laser readings to work with.
    # Each reading is separated by 45 degrees.
    # Assumes 181 laser readings, separated by 1 degree. 
    # (e.g. -90 degrees to 90 degrees....0 to 180 degrees)
 
    #number_of_laser_beams = str(len(msg.ranges))       
    self.right_dist = msg.ranges[270]
    self.rightfront_dist = msg.ranges[315]
    self.front_dist = msg.ranges[0]
    self.leftfront_dist = msg.ranges[45]
    self.left_dist = msg.ranges[90]
             
  def follow_wall(self):
    """
    This method causes the robot to follow the boundary of a wall.
    """
    # Create a geometry_msgs/Twist message
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0        
 
    # Logic for following the wall
    # >d means no wall detected by that laser beam
    # <d means an wall was detected by that laser beam
    d = self.dist_thresh_wf
     
    if self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist > d and self.counter_r == 0 and self.counter_l == 0:
      self.wall_following_state = "go straight"
      msg.linear.x = self.forward_speed

    elif (self.front_dist < d or self.leftfront_dist < d) and self.counter_r == 0 and self.counter_l == 0:
      self.wall_following_state = "turn right"
      msg.angular.z = -self.turning_speed
      n = randrange(1, 200)
      self.counter_r = n

    elif self.leftfront_dist < d and self.counter_r == 0 and self.counter_l == 0:
      self.wall_following_state = "turn left"
      msg.angular.z = self.turning_speed
      n = randrange(1, 200)
      self.counter_l = n

    elif self.counter_r > 0:
      self.wall_following_state = "random right"
      msg.angular.z = -self.turning_speed
      self.counter_r = self.counter_r - 1

    elif self.counter_l > 0:
      self.wall_following_state = "random left"
      msg.angular.z = self.turning_speed
      self.counter_l = self.counter_l - 1

    else:
      msg.angular.z = 2.0
 
    # Send velocity command to the robot
    self.publisher_.publish(msg)    
 
def main(args=None):
 
    # Initialize rclpy library
    rclpy.init(args=args)
     
    # Create the node
    controller_1 = Controller_1()
 
    # Spin the node so the callback function is called
    # Pull messages from any topics this node is subscribed to
    # Publish any pending messages to the topics
    rclpy.spin(controller_1)
 
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_1.destroy_node()
     
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
