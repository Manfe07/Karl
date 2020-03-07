#!/usr/bin/env python

import rospy, math, time
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64

pub = rospy.Publisher('/karl/speed', Float64, queue_size=10)

old_x = 0.0
old_y = 0.0

speed_array = [0, 0, 0, 0, 0]

speed = 0
tick = time.time()
old_tick = time.time()    

def calc_speed(data):
    global old_x, old_y, old_tick, speed_array

    tick = time.time()
    duration = tick - old_tick
    freq = 1 / duration
    old_tick = tick

    new_x = data.pose.position.x
    new_y = data.pose.position.y

    speed_x = math.fabs(new_x - old_x) * freq
    speed_y = math.fabs(new_y - old_y) * freq
    
    speed_vector = complex(speed_x, speed_y)

    for i in range(1,4):
        speed_array[i-1] = speed_array[i]
    
    speed = abs(speed_vector)
    speed_array[4] = speed
    
    speed_sum = 0.0

    for i in range(4):
        speed_sum = speed_sum + speed_array[i]

    speed_avg = speed_sum / 5

    pub.publish(Float64(speed_avg))
    print(speed_avg)
    print(speed)

    old_x = new_x
    old_y = new_y
    

def speed_calc():
    rospy.init_node('speed_calc', anonymous=True)
    rospy.Subscriber('/slam_out_pose', PoseStamped, calc_speed)
    # Initial movement.
    pub.publish(Float64(speed))
    rospy.spin()


if __name__ == '__main__':
    speed_calc()