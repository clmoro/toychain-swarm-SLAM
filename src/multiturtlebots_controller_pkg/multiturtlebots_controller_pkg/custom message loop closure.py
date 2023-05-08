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

# Handle uint32 messages
from std_msgs.msg import Bool

# Transform between two keyframes
from geometry_msgs.msg import Transform
 
class Loop_Closures_Generator(Node):
  """
  Class constructor to set up the node
  """
  def __init__(self):
 
    ############## INITIALIZE ROS PUBLISHERS AND SUBSCRIBERS ######
    super().__init__('Loop_Closures_Generator')
 
    # Create a subscriber from the state_estimate messages of every robot
    self.subscription = self.create_subscription(
                        Float64MultiArray,
                        '/bot1/state_est',
                        self.state_estimate_callback_1,
                        10)

    self.subscription = self.create_subscription(
                        Float64MultiArray,
                        '/bot2/state_est',
                        self.state_estimate_callback_2,
                        10)

    self.subscription = self.create_subscription(
                        Float64MultiArray,
                        '/bot3/state_est',
                        self.state_estimate_callback_3,
                        10)

    self.subscription = self.create_subscription(
                        Float64MultiArray,
                        '/bot4/state_est',
                        self.state_estimate_callback_4,
                        10)

    # Create a publisher
    # This node publishes random generated loop closures 
    # The type is a custom message: InterRobotLoopClosure.msg
    self.publisher_loopclosure = self.create_publisher(
                                 InterRobotLoopClosure, 
                                 '/cslam/inter_robot_loop_closure', 
                                 1000)
    timer_period = 10  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
 
  def state_estimate_callback_1(self, msg):                    
    s1 = msg.data

  def state_estimate_callback_2(self, msg):
    s2 = msg.data

    self.publish_custom_odometry(id_custom_vector)

  def state_estimate_callback_3(self, msg):
    s3 = msg.data

  def state_estimate_callback_4(self, msg):
    s4 = msg.data

  def timer_callback(self, s1, s2, s3, s4):
    robot0_id = randrange(1, 4)
    robot1_id = randrange(1, 4)
    robot0_keyframe_id = robot0_id
    robot1_keyframe_id = robot1_id
    success = True
    transform = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    msg.data = [robot0_keyframe_id,robot0_id,robot1_keyframe_id,robot1_id,False,transform]
    
    if robot0_id == 1 and robot1_id == 2:
      calc_tranform(s1, s2)
    elif robot0_id == 1 and robot1_id == 3:
        calc_tranform(s1, s3)
    elif robot0_id == 1 and robot1_id == 4:
        calc_tranform(s1, s4)
    elif robot0_id == 2 and robot1_id == 3:
        calc_tranform(s2, s3)
    elif robot0_id == 2 and robot1_id == 4:
        calc_tranform(s2, s4)
    elif robot0_id == 3 and robot1_id == 4:
        calc_tranform(s3, s4)
    else:
        success = False

  def calc_tranform(self, e1, e2):
    dx = e1[0] - e2[0]
    dy = e1[1] - e2[1]
    dtheta = e1[2] - e2[2]
    w, x, y, z = self.quaternion_from_euler(0.0, 0.0, dtheta)
    msg.data = [dx, dy, 0.0, x, y, z, w]
    self.publisher_loopclosure.publish(msg)
    self.i += 1

  def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q
 
def main(args=None):
    """
    Entry point for the program.
    """
    # Initialize rclpy library
    rclpy.init(args=args)
 
    # Create the node
    loop_closure_generator = Loop_Closures_Generator()
 
    # Spin the node so the callback function is called.
    # Pull messages from any topics this node is subscribed to.
    # Publish any pending messages to the topics.
    rclpy.spin(loop_closure_generator)
 
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    loop_closure_generator.destroy_node()
     
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
