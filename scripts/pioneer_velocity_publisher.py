#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

vel_l = Float32()
vel_r = Float32()
vel_l.data = 0.0
vel_r.data = 0.0
multiplier = 2.0


def handle_wheel_velocity(msg):
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