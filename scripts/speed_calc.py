#!/usr/bin/env python

import rospy, math, time
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64

pub = rospy.Publisher('/karl/speed', Float64, queue_size=10)

old_x = 0.0
old_y = 0.0

speed_sum = 0.0
counter = 0
speed = 0
tick = time.time()
old_tick = time.time()    

def calc_speed(data):
    global old_x, old_y, old_tick, counter, speed_sum

    tick = time.time()
    duration = tick - old_tick
    freq = 1 / duration
    old_tick = tick

    new_x = data.pose.position.x
    new_y = data.pose.position.y

    speed_x = math.fabs(new_x - old_x) * freq
    speed_y = math.fabs(new_y - old_y) * freq
    
    speed_vector = complex(speed_x, speed_y)
 
    speed = abs(speed_vector)
    if (speed <= 0.02):
        speed = 0
    
    speed_sum += speed
    counter += 1

    if (counter >= 15):
        speed_avg = speed_sum / 15
        if (speed_avg <= 0.01):
            speed_avg = 0
        speed_sum = 0
        counter = 0
        
        pub.publish(Float64(speed_avg))
        #print(speed_avg)
    
    #print(speed)
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