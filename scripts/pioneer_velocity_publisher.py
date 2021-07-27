#!/usr/bin/env python

"""
.. module:: pioneer_velocity_publisher
    :platform: Unix
    :synopsis: Python module for publishing velocity commands to the wheels the pioneer robot

.. moduleauthor:: Omotoye Adekoya adekoyaomotoye@gmail.com 

This node takes in the cmd_vel velocity command from the go to point module and interprets it to the 
required velocity that should be sent to each of the wheels of the pioneer robot. 

Subscribes to:
    /cmd_vel velocity to move the robot to the desired robot positions

Publishes to: 
    /leftwheel_vel the required leftwheel velocity to achieve the cmd_vel command.
    /rightwheel_vel the required rightwheel velocity to achieve the cmd_vel command
    
"""

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

vel_l = Float32()
vel_r = Float32()
vel_l.data = 0.0
vel_r.data = 0.0
multiplier = 2.0


def handle_wheel_velocity(msg):
    """This is a callback function that takes in the cmd_vel command from the go to point
    node and the interprets it to the required velocity for each of the wheels of the pioneer robot

    Args:
        msg ([Twist]): Linear and angular velocity command from the go to point node
    """
    global vel_l, vel_r
    if (msg.linear.x > 0 or msg.linear.x < 0):
        vel_l.data = msg.linear.x * multiplier
        vel_r.data = msg.linear.x * multiplier
    elif(msg.angular.z > 0):
        vel_l.data = msg.angular.z
        vel_r.data = -(msg.angular.z)
    elif(msg.angular.z < 0):
        vel_l.data = -(msg.angular.z)
        vel_r.data = msg.angular.z
        
    
def main():
    rospy.init_node('pioneer_velocity_publisher', anonymous=True)
    pub_left = rospy.Publisher('/leftwheel_vel', Float32, queue_size=100)
    pub_right = rospy.Publisher('/rightwheel_vel', Float32, queue_size=100)
    rospy.Subscriber('/cmd_vel', Twist, handle_wheel_velocity)
    
    while not rospy.is_shutdown():
        pub_right.publish(vel_r)
        pub_left.publish(vel_l)
        


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass