# Author: Angelo Moroncelli
# ROS Version: ROS 2 Foxy Fitzroy

import rclpy
from rclpy.node import Node
from math import *
import math
from nav_msgs.msg import Odometry
import tf_transformations
import tf2_ros
import numpy as np
from random import randrange

last_odom = Odometry()
last_odom.pose.pose.position.x = 100.0
last_odom.pose.pose.position.y = 100.0

#The cumulative noisy pose
pose = [0.0,0.0,0.0]

a1 = 150.0
a2 = 160.0
a3 = 1
a4 = 0.001

new_odom_frame = ""
odom_frame = ""

class OdomSubscriber(Node):

    def __init__(self):
        super().__init__('Odom_subscriber')
        self.subscription = self.create_subscription(Odometry, '/bot4/odom', self.add_noise_callback, 10)
          
        self.publisher_ = self.create_publisher(Odometry, '/bot4/noisy_odom', 10)

        self.subscription
        
    def add_noise_callback(self, msg):
            global last_odom
            global new_odom_frame
            global odom_frame
            global pose
            global a1
            global a2
            global a3
            global a4

            q = [msg.pose.pose.orientation.x,
                 msg.pose.pose.orientation.y,
                 msg.pose.pose.orientation.z,
                 msg.pose.pose.orientation.w]
                 
            (r, p, theta2) = tf_transformations.euler_from_quaternion(q)
            
            #At the initial time step the odometry is without noise, since we need a delta ...
            if(last_odom.pose.pose.position.x == 100.0 and last_odom.pose.pose.position.y == 100.0):
                pose[0] = msg.pose.pose.position.x
                pose[1] = msg.pose.pose.position.y
                pose[2] = theta2
                last_odom.pose.pose.position.x = msg.pose.pose.position.x
                last_odom.pose.pose.position.y = msg.pose.pose.position.y
                last_odom.pose.pose.position.z = 0.0
                last_odom.pose.pose.orientation.x = msg.pose.pose.orientation.x
                last_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y
                last_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z
                last_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w

            else:
                dx = msg.pose.pose.position.x - last_odom.pose.pose.position.x
                dy = msg.pose.pose.position.y - last_odom.pose.pose.position.y
                trans = sqrt(dx*dx + dy*dy)
                q = [last_odom.pose.pose.orientation.x,
                     last_odom.pose.pose.orientation.y,
                     last_odom.pose.pose.orientation.z,
                     last_odom.pose.pose.orientation.w]
                     
                (r, p, theta1) = tf_transformations.euler_from_quaternion(q)
                rot1 = atan2(dy, dx) - theta1
                rot2 = theta2-theta1-rot1
                
                sd_rot1 = a1*abs(rot1) + a2*trans
                sd_rot2 = a1*abs(rot2) + a2*trans
                sd_trans = a3*trans + a4*(abs(rot1) + abs(rot2))
                
                trans += np.random.normal(0, sd_trans*sd_trans)
                rot1 += np.random.normal(0, sd_rot1*sd_rot1)
                rot2 += np.random.normal(0, sd_rot2*sd_rot2)
                
                pose[0] += trans*math.cos(theta1+rot1)
                pose[1] += trans*math.sin(theta1+rot1)
                pose[2] +=  rot1 + rot2
                last_odom.pose.pose.position.x = msg.pose.pose.position.x
                last_odom.pose.pose.position.y = msg.pose.pose.position.y
                last_odom.pose.pose.position.z = 0.0
                last_odom.pose.pose.orientation.x = msg.pose.pose.orientation.x
                last_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y
                last_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z
                last_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w

            message = Odometry()
            ## adding position,yaw back 
            message.header = msg.header
            message.pose.pose.position.x = pose[0]
            message.pose.pose.position.y = pose[1]
            quaternion = tf_transformations.quaternion_from_euler(r,p,pose[2])
            #euler = tf_transformations.euler_from_quaternion(quaternion)
            message.pose.pose.orientation.x = quaternion[0]
            message.pose.pose.orientation.y = quaternion[1]
            message.pose.pose.orientation.z = quaternion[2]
            message.pose.pose.orientation.w = quaternion[3]

            self.publisher_.publish(message)


                             
def main(args=None):

    rclpy.init(args=args)
    node = rclpy.create_node('noisy_odometry')

    minimal_subscriber = OdomSubscriber()
    
    # alpha 1 is degree/degree
    # a1 = 0.15
    
    # alpha 2 is degree/m  
    # a2 = 10.0*pi/180.0
            
    # alpha 3 is m/meter
    # a3 = 0.15

    # alpha 4 is m/degree
    # a4 = 0.10

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


