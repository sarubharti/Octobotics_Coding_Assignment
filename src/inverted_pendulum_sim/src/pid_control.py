#!/usr/bin/env python3
import rospy
# from std_msgs.msg import String
from inverted_pendulum_sim.msg import ControlForce
from inverted_pendulum_sim.msg import CurrentState
from math import*
import math
import numpy as np
from control.matlab import lqr
# def talker():
    # pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)
    # rospy.init_node('talker', anonymous=True)
    # rate = rospy.Rate(0.1) # 10hz
    # t=0
    # Fs=8000
    # f=500
    # sample=16
    # while not rospy.is_shutdown():
    #     # hello_str = "hello world %s" % rospy.get_time()
    #     t=t+1
    #     hello_str=1*sin(2*pi*f*t/Fs)
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass
def find_lqr_control_input(cart,pendulum,time_delta,error,previous_error,theta_dot,x_dot,g):
    # Using LQR to find control inputs
   
    # The A and B matrices are derived in the report
    A = np.matrix(  [
                    [0,1,0,0],
                    [0,0,g*pendulum.ball_mass/cart.mass,0],
                    [0,0,0,1],
                    [0,0,(cart.mass+pendulum.ball_mass)*g/(pendulum.length*cart.mass),0]
                    ])
   
    B = np.matrix(  [
                    [0],
                    [1/cart.mass],
                    [0],
                    [1/(pendulum.length*cart.mass)]
                    ])

    # The Q and R matrices are emperically tuned. It is described further in the report
    Q = np.matrix(  [
                    [10,0,0,0],
                    [0,1,0,0],
                    [0,0,10000,0],
                    [0,0,0,100]
                    ])

    R = np.matrix([500])

    # The K matrix is calculated using the lqr function from the controls library
    K, S, E = lqr(A, B, Q, R)
    np.matrix(K)

    x = np.matrix(  [
                    [np.squeeze(np.asarray(cart.x))],
                    [np.squeeze(np.asarray(x_dot))],
                    [np.squeeze(np.asarray(pendulum.theta))],
                    [np.squeeze(np.asarray(theta_dot))]
                    ])

    desired = np.matrix(    [
                    [300],
                    [0],
                    [0],
                    [0]
                    ])

    F = -(K*(x-desired))
    return np.squeeze(np.asarray(F))

class Cart:
    def __init__(self,x,mass):
        self.x = x  
        self.y = 1       # 0.6 was chosen for aesthetic reasons.
        self.mass = mass
        # self.color = (0,255,0)

class Pendulum:
    def __init__(self,length,theta,ball_mass):
        self.length = length
        self.theta = theta
        self.ball_mass = ball_mass      
        # self.color = (0,0,255)

def find_error(pendulum):
    # There's a seperate function for this because of the wrap-around problem
    # This function returns the error
    previous_error = (pendulum.theta % (2 * math.pi)) - 0
    if previous_error > math.pi:
        previous_error = previous_error - (2 * math.pi)
    return previous_error
def find_pid_control_input(cart,pendulum,time_delta,error,previous_error,integral,g):
    # Using PID to find control inputs

    # The gains were emperically tuned
    Kp = -250
    Kd = -200
    Ki = -20

    derivative = (error - previous_error) / time_delta
    integral += error * time_delta
    F = (Kp * error) + (Kd * derivative) + (Ki * integral)
    return F,integral

class LQ():
    def __init__(self):
        self.prev_error=[0]
        self.integrate=[0]
    def callback(self,data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)
        # rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(1) # 10hz

        theta=data.curr_theta-pi
        x=data.curr_x
        x_dot=data.curr_x_dot
        theta_dot=data.curr_theta_dot
        # print(data.header)
        mass_of_ball = 2.0
        mass_of_cart = 5.
        g = 9.81
        # errors, force, theta, times, x = [],[],[],[],[]
        # world_size = 1000
        cart = Cart(x,mass_of_cart)
        pendulum = Pendulum(3,theta,mass_of_ball)
        time_delta=5.0/100
        error = find_error(pendulum)
        previous_error=self.prev_error[-1]
        integral=self.integrate[-1]
       
        # F = find_lqr_control_input(cart,pendulum,time_delta,error,previous_error,theta_dot,x_dot,g)
        F,ig = find_pid_control_input(cart,pendulum,time_delta,error,previous_error,integral,g)
        self.integrate.append(ig)
        previous_error=self.prev_error.append(error)
       
        # t=11
        # hello_str=100*sin(2*pi*f*t/Fs)
        hello_str=F
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        # rate.sleep()
   
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    lq=LQ()

    rospy.Subscriber('/inverted_pendulum/current_state', CurrentState, lq.callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
