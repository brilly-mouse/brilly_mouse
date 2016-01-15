#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import Adafruit_BBIO.ADC as ADC

class IR():
    @staticmethod
    def setup():
        ADC.setup()

    @staticmethod
    def get_front():
        v = IR.get_value("P9_38")
        return round(2076 / (v * 736.56 - 11), 2) / 100

    @staticmethod
    def get_right():
        v = IR.get_value("P9_40")
        return round(1 / (v * 0.4464 + 0.011) - 1.8, 2) / 100

    @staticmethod
    def get_left():
        v = IR.get_value("P9_36")
        return round(1 / (v * 0.4464 + 0.011) - 1.8, 2) / 100

    @staticmethod
    def median(lst):
        quotient, remainder = divmod(len(lst), 2)
        if remainder:
            return sorted(lst)[quotient]
        return float(sum(sorted(lst)[quotient - 1:quotient + 1]) / 2)

    @staticmethod
    def get_value(pin):
        return IR.median([ADC.read(pin) for i in range(5)])

if __name__ == '__main__':
    rospy.init_node('infrared_publisher')

    left_pub = rospy.Publisher('/sensors/ir_left', Float32, queue_size=1)
    right_pub = rospy.Publisher('/sensors/ir_right', Float32, queue_size=1)
    front_pub = rospy.Publisher('/sensors/ir_front', Float32, queue_size=1)

    IR.setup()

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        left_pub.publish(Float32(IR.get_left()))
        right_pub.publish(Float32(IR.get_right()))
        front_pub.publish(Float32(IR.get_front()))
        r.sleep()
