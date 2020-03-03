#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float64

pub = rospy.Publisher('/karl/speed', Float64, queue_size=10)

old_x = 0
old_y = 0

speed = 0

def calc_speed(data):
    new_x = data.position.x
    new_y = data.position.y

    speed_x = math.fabs(new_x - old_x)
    speed_y = math.fabs(new_y - old_y)

    speed_vector = complex(speed_x, speed_y)

    speed = math.fabs(speed_vector)
    pub.publish(Float64(speed))


def speed_calc():
    rospy.init_node('speed_calc', anonymous=True)
    rospy.Subscriber('/slame_out_pose', Pose, calc_speed)
    # Initial movement.
    pub.publish(Float64(speed))
    rospy.spin()


if __name__ == '__main__':
    speed_calc()