"""
.. module:: user_interface
    :platform: Unix
    :synopsis: Python module that acts as a user interface for getting user commands
.. moduleauthor:: Omotoye Adekoya adekoyaomotoye@gmail.com 
This node prompts the user to enter a command that is going to be sent to the state machine 
to perform a required task
Service:
    /user_interface sends the command to the state machine
    
"""

import rospy
import time
from rt2_assignment1.srv import Command

def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("Please wait, the robot is going to stop when the position will be reached")
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
