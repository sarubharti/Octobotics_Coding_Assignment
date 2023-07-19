#!/usr/bin/env python3
import rospy
from inverted_pendulum_sim.msg import ControlForce
from math import*

def sineF():
    pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=1)
    rospy.init_node('sineF', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    t=0
    #Fs=8000
    Fs=12000
    f=500
    sample=16
    while not rospy.is_shutdown():
        t=t+1
        F_val=20*sin((2*pi*f*t)/Fs)
        #F_val=5*sin(t/(2*pi))
        rospy.loginfo(F_val)
        pub.publish(F_val)
        rate.sleep()

if __name__ == '__main__':
    try:
        sineF()
    except rospy.ROSInterruptException:
        pass

