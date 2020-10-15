#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
import pigpio
import math
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(10, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
pi = pigpio.pi()
pi.set_mode(25,pigpio.OUTPUT)
pi.set_PWM_frequency(25,10000)
pi.set_PWM_range(25,1000)
pi.set_mode(8,pigpio.OUTPUT)
pi.set_PWM_frequency(8,10000)
pi.set_PWM_range(8,1000)
pi.set_mode(7,pigpio.OUTPUT)
pi.set_PWM_frequency(7,10000)
pi.set_PWM_range(7,1000)
print('DS4 is RobotController ,control now!!')
def motor(lh1,lh2,lh3,v1d,v2d,v3d):
    GPIO.output(8,lh1)
    GPIO.output(10,lh2)
    GPIO.output(12,lh3)
    pi.set_PWM_dutycycle(25,v1d)
    pi.set_PWM_dutycycle(8,v2d)
    pi.set_PWM_dutycycle(7,v3d)
    print ('low/high:',lh1,lh2,lh3,'duty:',v1d,v2d,v3d)

def yaw_L(L):
    Ld = L * 250
    GPIO.output(8,1)
    GPIO.output(10,1)
    GPIO.output(12,0)
    pi.set_PWM_dutycycle(25,Ld)
    pi.set_PWM_dutycycle(8,Ld)
    pi.set_PWM_dutycycle(7,Ld)
    print ('duty:',Ld)

def yaw_R(R):
    Rd = R * 250
    GPIO.output(8,0)
    GPIO.output(10,0)
    GPIO.output(12,1)
    pi.set_PWM_dutycycle(25,Rd)
    pi.set_PWM_dutycycle(8,Rd)
    pi.set_PWM_dutycycle(7,Rd)
    print ('duty:',Rd)

def joy_callback(joy_msg):
    lh1 = 0
    lh2 = 0
    lh3 = 0
    inv = math.sqrt(3)
    X = float(joy_msg.axes[0])
    Y = float(joy_msg.axes[1])
    L_raw = float(joy_msg.axes[3])
    R_raw = float(joy_msg.axes[4])
    L = abs(L_raw - 1)
    R = abs(R_raw - 1)
    print ('raw:',X,Y,L,R)
    v1 = X/2-inv*Y/2
    v2 = -X
    v3 = X/2+inv*Y/2
    print ('math:',v1,v2,v3)
    v1d = abs(v1) * 500 
    v2d = abs(v2) * 500 
    v3d = abs(v3) * 500 
    if v1 > 0:
	lh1 = 1
        v1d = v1d + L * 150
    elif v1 < 0:
        lh1 = 0
        v1d = v1d + R * 150
    if v2 > 0:
	lh2 = 1
        v2d = v2d + L * 150
    elif v2 < 0:
	lh2 =0
        v2d = v2d + R * 150
    if v3 > 0:
	lh3 = 0
        v3d = v3d + L * 150
    if v3 < 0:
	lh3 =1
        v3d = v3d + R * 150
    motor(lh1,lh2,lh3,v1d,v2d,v3d)
    if L > R and v1d == 0 and v2d == 0 and v3d == 0:
	yaw_L(L)
    elif L < R and v1d == 0 and v2d == 0 and v3d == 0:
	yaw_R(R)

rospy.init_node('joy_twist')
sub = rospy.Subscriber('joy', Joy, joy_callback, queue_size=1)
rospy.spin()
