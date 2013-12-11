#!/usr/bin/env python

#something something ARCTAN2

import rospy
import math

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

setAngle = 0
setSpeed = 0
actualAngle = 0
integral = 0
previous_error = 0
dt = 0.010 #miliseconds

def setAngleCallback(data):
    global setAngle
    setAngle = data.data.real
    
def setSpeedCallback(data):
    global setSpeed
    setSpeed = data.data.real

def callback(data):
    global actualAngle
    global mapOffset
    #Get Position
    position = data.pose.pose.position
    #calculate rotation
    quat = data.pose.pose.orientation
    q = [quat.x,quat.y,quat.z,quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    actualAngle = yaw
    
def doPID():
    global actualAngle
    global setAngle
    global setSpeed
    global previous_error
    global integral
    global dt
    
    Kp = 1.5
    Ki = 0
    Kd = 0.1
    
    print setAngle
    print actualAngle
    error = setAngle - actualAngle
    if error > math.pi:
        error -= 2*math.pi
    elif error < -math.pi:
        error += 2*math.pi
        
    print "error"
    print error
    integral = integral + error*dt
    derivative = (error - previous_error)/dt
    output = Kp*error + Ki*integral + Kd*derivative
    previous_error = error
    publishTwist(setSpeed, output)
    
def publishTwist(u,w):
    twist = Twist()
    twist.linear.x = u
    twist.angular.z = w
    velocPub.publish(twist)
    rospy.sleep(0.01)

if __name__ == '__main__':

    rospy.init_node('angle_PID')
    velocPub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)

    rospy.Subscriber("set_angle", Float64, setAngleCallback)
    rospy.Subscriber("set_speed", Float64, setSpeedCallback)
    rospy.Subscriber("shifted_Odom", Odometry, callback)


    while not rospy.is_shutdown():
        rospy.sleep(dt)
        doPID()