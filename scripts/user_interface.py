import rospy
import time
from rt2_assignment1.srv import Command

def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot: "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            time.sleep(1)
            x = int(input("\nPress 0 to stop the robot: "))
        else:
            print("The Goal will cancel now")
            ui_client("stop")
            time.sleep(1)
            x = int(input("\nPress 1 to start the robot: "))
            
if __name__ == '__main__':
    main()
