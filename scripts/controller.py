#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pid import PID
from math import pi, atan2
from tf import transformations
from time import sleep

# sketchy mouse controller

class Controller(object):
    def __init__(self):
        self.left_wall_pid = PID(14, 2, 0.5)
        self.right_wall_pid = PID(14, 2, 0.5)
        self.left_wall_pid.setpoint = 0.055
        self.right_wall_pid.setpoint = 0.055

        self.goal_angle = None
        self.goal_distance = None

        self.start_position = None
        self.position = None
        self.start_angle = None
        self.angle = None

        self.left_dist = 0
        self.right_dist = 0
        self.front_dist = 0

        self.commands = []

        self.busy = False

        self.callback = None

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/wheel_odom", Odometry, self.update)
        rospy.Subscriber("/sensors/ir_left", Float32, self.ir_left)
        rospy.Subscriber("/sensors/ir_right", Float32, self.ir_right)
        rospy.Subscriber("/sensors/ir_front", Float32, self.ir_front)

    def ir_left(self, data):
        self.left_dist = data.data

    def ir_right(self, data):
        self.right_dist = data.data

    def ir_front(self, data):
        self.front_dist = data.data

    def update(self, odom):
        # populate current state variables
        pose = odom.pose.pose
        twist = odom.twist.twist
        q = pose.orientation
        self.angle = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.position = pose.position

        if not self.busy and self.commands:
            data = self.commands.pop(0)
            data[0]()
            self.callback = data[1]
            print("Starting action: " + data[2])
            self.busy = True

        # traverse if needed
        cmd_vel = Twist()
        linear_vel = 0
        angular_vel = 0
        if self.goal_distance != None:
            # heading correction

            if self.left_dist > 0.03 and self.left_dist < 0.08:
                angular_vel -= self.left_wall_pid.calc(self.left_dist)

            if self.right_dist > 0.03 and self.right_dist < 0.08:
                angular_vel += self.right_wall_pid.calc(self.right_dist)
            # vroom
            offset = abs(self.goal_distance) - dist(self.position.x, self.position.y, self.start_position.x, self.start_position.y)
            if abs(offset) < 0.01 and abs(twist.linear.x) < 0.01:
                self.goal_distance = None
            elif abs(offset) < 0.05:
                linear_vel = signum(offset) * max(0, 0.15 - abs(angular_vel) / 10)
            else:
                linear_vel = signum(offset) * max(0, 0.2 - abs(angular_vel) / 10)

            # absolute position correction via front IR
            if offset > 0.08 and offset < 0.18:
                error = self.front_dist - offset - 0.015
                if abs(error) < 0.06:
                    self.goal_distance += error

        if self.goal_angle != None:
            offset = wrap_angle(self.goal_angle - self.angle)
            # print(offset)
            if abs(offset) < 0.06 and abs(twist.angular.z) < 0.2 and signum(offset) == -signum(twist.angular.z):
                self.goal_angle = None
            elif abs(offset) < 0.05:
                angular_vel = signum(offset) * 1.5
            elif abs(offset) < 0.2:
                angular_vel = signum(offset) * 2.5
            else:
                angular_vel = signum(offset) * 5

        if self.busy and self.goal_angle == None and self.goal_distance == None:
            self.busy = False
            print('DONE')
            linear_vel = 0
            angular_vel = 0
            self.callback and self.callback()

        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd_vel)

    def turn(self, angle, cb=None):
        def f():
            self.goal_angle = wrap_angle(self.angle + angle)
        self.commands.append((f, cb, 'turn by ' + str(angle)))

    def turnTo(self, angle, cb=None):
        def f():
            self.goal_angle = wrap_angle(angle)
        self.commands.append((f, cb, 'turn to ' + str(angle)))

    def straight(self, dist, cb=None):
        def f():
            self.start_position = self.position
            self.start_angle = self.angle
            self.goal_distance = dist
        def c():
            theta = atan2(self.position.y - self.start_position.y, self.position.x - self.start_position.x)
            self.goal_angle = theta
            self.busy = True
            self.callback = cb
        self.commands.append((f, c, 'move ' + str(dist * 100) + 'cm'))

def dist(x1, y1, x2, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

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

if __name__ == '__main__':
    rospy.init_node('controller')
    c = Controller()
    sleep(2)
    c.straight(0.18 * 2)
    # c.straight(0.18)
    # c.turn(- pi / 2)
    # c.straight(0.18)
    rospy.spin()
