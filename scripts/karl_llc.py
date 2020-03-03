#!/usr/bin/env python
import rospy
import time

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from PCA9685 import PCA9685

pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)

last_call = time.time()

class MotorDriver():
    def __init__(self):
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

    def MotorRun(self, motor, speed):
        if speed < 0:
            speed = abs(speed)
            direction = False
        else:
            direction = True

        if speed > 100:
            return
        if(motor == 0):
            pwm.setDutycycle(self.PWMA, speed)
            if(direction):
                pwm.setLevel(self.AIN1, 0)
                pwm.setLevel(self.AIN2, 1)
            else:
                pwm.setLevel(self.AIN1, 1)
                pwm.setLevel(self.AIN2, 0)
        else:
            pwm.setDutycycle(self.PWMB, speed)
            if(direction):
                pwm.setLevel(self.BIN1, 0)
                pwm.setLevel(self.BIN2, 1)
            else:
                pwm.setLevel(self.BIN1, 1)
                pwm.setLevel(self.BIN2, 0)
        last_call = time.time()

    def MotorStop(self, motor):
        if (motor == 0):
            pwm.setDutycycle(self.PWMA, 0)
        else:
            pwm.setDutycycle(self.PWMB, 0)


Motor = MotorDriver()

def motor_drive(message):
    speed = message.linear.x * 400
    steering = message.angular.z * 200
    
    speed_R = speed + (steering / 2)
    speed_L = speed - (steering / 2)

    Motor.MotorRun(0, speed_R)
    Motor.MotorRun(1, speed_L)


def stop_Motor():
    Motor.MotorStop(0)
    Motor.MotorStop(1)
    print("Motor Stopped cause Shutdown")

#def drive

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('karl_llc')

    rospy.Subscriber("cmd_vel", Twist, motor_drive)


    if((last_call + 1 ) <= time.time()):
        stop_Motor()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

rospy.on_shutdown(stop_Motor)