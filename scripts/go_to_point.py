#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import actionlib
import rt2_assignment1.msg
import math


class GoToPointAction(object):
    # create messages that are used to publish feedback/result
    _feedback = rt2_assignment1.msg.PositionFeedback()
    _result = rt2_assignment1.msg.PositionResult()

    # robot state variables
    position_ = Point()
    yaw_ = 0
    position_ = 0
    state_ = 0

    # parameters for control
    yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
    yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
    dist_precision_ = 0.1
    kp_a = -3.0
    kp_d = 0.2
    ub_a = 0.6
    lb_a = -0.5
    ub_d = 0.6

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, rt2_assignment1.msg.PositionAction, execute_cb=self.go_to_point, auto_start=False)

        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub_odom = rospy.Subscriber(
            '/odom', Odometry, self.clbk_odom)
        self._as.start()

    def clbk_odom(self, msg):
        # position
        self.position_ = msg.pose.pose.position

        # yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw_ = euler[2]

    def change_state(self, state):
        self.state_ = state
        print('State changed to [%s]' % self.state_)

    def normalize_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def fix_yaw(self, des_pos):
        desired_yaw = math.atan2(
            des_pos.y - self.position_.y, des_pos.x - self.position_.x)
        err_yaw = self.normalize_angle(desired_yaw - self.yaw_)
        rospy.loginfo(err_yaw)
        twist_msg = Twist()
        if math.fabs(err_yaw) > self.yaw_precision_2_:
            twist_msg.angular.z = self.kp_a*err_yaw
            if twist_msg.angular.z > self.ub_a:
                twist_msg.angular.z = self.ub_a
            elif twist_msg.angular.z < self.lb_a:
                twist_msg.angular.z = self.lb_a
        self.pub_.publish(twist_msg)
        # state change conditions
        if math.fabs(err_yaw) <= self.yaw_precision_2_:
            # print ('Yaw error: [%s]' % err_yaw)
            self.change_state(1)

    def go_straight_ahead(self, des_pos):
        desired_yaw = math.atan2(
            des_pos.y - self.position_.y, des_pos.x - self.position_.x)
        err_yaw = desired_yaw - self.yaw_
        err_pos = math.sqrt(pow(des_pos.y - self.position_.y, 2) +
                            pow(des_pos.x - self.position_.x, 2))
        err_yaw = self.normalize_angle(desired_yaw - self.yaw_)
        rospy.loginfo(err_yaw)

        if err_pos > self.dist_precision_:
            twist_msg = Twist()
            twist_msg.linear.x = 0.3
            if twist_msg.linear.x > self.ub_d:
                twist_msg.linear.x = self.ub_d

            twist_msg.angular.z = self.kp_a*err_yaw
            self.pub_.publish(twist_msg)
        else:  # state change conditions
            # print ('Position error: [%s]' % err_pos)
            self.change_state(2)

        # state change conditions
        if math.fabs(err_yaw) > self.yaw_precision_:
            # print ('Yaw error: [%s]' % err_yaw)
            self.change_state(0)

    def fix_final_yaw(self, des_yaw):
        err_yaw = self.normalize_angle(des_yaw - self.yaw_)
        rospy.loginfo(err_yaw)
        twist_msg = Twist()
        if math.fabs(err_yaw) > self.yaw_precision_2_:
            twist_msg.angular.z = self.kp_a*err_yaw
            if twist_msg.angular.z > self.ub_a:
                twist_msg.angular.z = self.ub_a
            elif twist_msg.angular.z < self.lb_a:
                twist_msg.angular.z = self.lb_a
        self.pub_.publish(twist_msg)
        # state change conditions
        if math.fabs(err_yaw) <= self.yaw_precision_2_:
            # print ('Yaw error: [%s]' % err_yaw)
            self.change_state(3)

    def done(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self.pub_.publish(twist_msg)

    def go_to_point(self, goal):
        desired_position = Point()
        desired_position.x = goal.x
        desired_position.y = goal.y
        des_yaw = goal.theta
        self.change_state(0)

        while True:

            # helper variables
            #r = rospy.Rate(1)

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                break

            if self.state_ == 0:
                self.fix_yaw(desired_position)
            elif self.state_ == 1:
                self.go_straight_ahead(desired_position)
            elif self.state_ == 2:
                self.fix_final_yaw(des_yaw)
            elif self.state_ == 3:
                self.done()
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self._result)
                break


if __name__ == '__main__':
    rospy.init_node('go_to_point')
    server = GoToPointAction(rospy.get_name())
    rospy.spin()
