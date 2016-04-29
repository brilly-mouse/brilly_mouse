#!/usr/bin/env python
import rospy
from math import pi, cos, sin, atan2
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from tf import transformations
import numpy as np

# super sketchy

goal_theta = None
goal_x = None
goal_y = None
first = True
def update_velocity(odom):
    # get absolute angular position
    q = odom.pose.pose.orientation
    theta = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    position = odom.pose.pose.position

    global goal_theta
    global first
    global goal_x
    global goal_y
    if first:
        # goal_theta = theta + pi / 2
        goal_x = position.x + cos(theta) * 2
        goal_y = position.y + sin(theta) * 2
        first = False
        return

    angular_velocity = odom.twist.twist.angular.z

    cmd_vel = Twist()
    new_angular_velocity = 0
    new_linear_velocity = 0

    if goal_theta != None:
        offset = wrap_angle(goal_theta - theta)

        if abs(offset) < 0.01:
            goal_theta = None

        print(goal_theta, theta, offset)

        turn_dir = signum(offset)

        new_angular_velocity = angular_velocity + angular_accel * turn_dir;

        if signum(new_angular_velocity) == turn_dir:
            if abs(new_angular_velocity) < angular_jerk:
                new_angular_velocity = angular_jerk * turn_dir
            elif abs(new_angular_velocity) > target_angular_velocity:
                new_angular_velocity = target_angular_velocity * turn_dir
            cap = abs(offset * 8)
            if abs(new_angular_velocity) > cap:
                new_angular_velocity = max(angular_jerk, cap) * turn_dir
    elif goal_x != None and goal_y != None:
        desired_theta = atan2(goal_y - position.y, goal_x - position.x)
        angular_offset = wrap_angle(desired_theta - theta, 2)

        distance = dist(position.x, position.y, goal_x, goal_y)

        if distance > 0.05:
            new_angular_velocity = angular_offset * 5

        if distance < 0.02 and abs(odom.twist.twist.linear.x) < 0.02:
            goal_x = None
            goal_y = None
            goal_theta = None

        if abs(wrap_angle(desired_theta - theta)) > pi / 2:
            new_linear_velocity = -min(distance * 5, target_linear_velocity)
        else:
            new_linear_velocity = min(distance * 5, target_linear_velocity)
        print(distance, angular_offset)

    cmd_vel.linear.x = new_linear_velocity
    cmd_vel.angular.z = new_angular_velocity
    cmd_pub.publish(cmd_vel)

def dist(x1, y1, x2, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def cap_speed(velocity):
    if velocity > target_angular_velocity * signum(velocity):
        return target_angular_velocity * signum(velocity)
    return velocity

def wrap_angle(offset, x = 1):
    while offset > pi / x:
        offset -= 2 * pi / x
    while offset < -pi / x:
        offset += 2 * pi / x
    return offset

def signum(n):
    if n > 0:
        return 1
    if n < 0:
        return -1
    return 0

target_linear_velocity = 0.2
linear_accel = 0.04
linear_jerk = 0.04

target_angular_velocity = 10
angular_accel = 2
angular_jerk = 2

rospy.init_node('turn')
rospy.Subscriber("/wheel_odom", Odometry, update_velocity)
cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rospy.spin()
