#!/usr/bin/env python3  
import rospy
import math
import tf
from geometry_msgs.msg import Twist, PoseStamped

goal_pose = None
goal_msg = None
def goal_callback(msg):
    global goal_pose, goal_msg
    goal_pose = msg.pose.position
    goal_msg = msg
    #print(goal_msg.header.frame_id)

if __name__ == '__main__':
    rospy.init_node('simple_controller')
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)

    listener = tf.TransformListener()
    cmd_vel_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist,queue_size=1)

    while not rospy.is_shutdown():
        if not goal_pose:
            continue
        try:
            (robot_trans,robot_rot) = listener.lookupTransform(goal_msg.header.frame_id, '/base_link',  rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        robot_to_goal = (goal_pose.x - robot_trans[0], goal_pose.y - robot_trans[1])
        angle = math.atan2(robot_to_goal[1], robot_to_goal[0]) # -pi to pi
        orientation = tf.transformations.euler_from_quaternion(robot_rot)[2] # -pi to pi

        linear = 0
        angular = (angle - orientation) * 0.2 #-0.5 * angle
        if abs(angular) < 0.05:
            linear = 0.5

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        cmd_vel_publisher.publish(cmd)

        rospy.Rate(10.0).sleep()
