#!/usr/bin/env python3
import matplotlib.pyplot as plt
import rospy
from inverted_pendulum_sim.msg import ControlForce
from inverted_pendulum_sim.msg import CurrentState
import numpy as np
from matplotlib.animation import FuncAnimation


class Visualiser_position:
    def __init__(self):
        self.fig_px, self.ax_px = plt.subplots()
        self.ln_px, = plt.plot([], [], 'ro')
        self.x_data_px, self.y_data_px = [] , []
        self.fig_th, self.ax_th = plt.subplots()
        self.ln_th, = plt.plot([], [], 'ro')
        self.x_data_th, self.y_data_th = [] , []

    def plot_init_px(self):
        self.ax_px.set_xlim(0, 500)
        self.ax_px.set_ylim(-60, 60)
        self.ax_px.set_xlabel('time')
        self.ax_px.set_ylabel('cart acceleration')

        return self.ln_px

    def plot_init_th(self):
        self.ax_th.set_xlim(0, 500)
        self.ax_th.set_ylim(30, -30)
        self.ax_th.set_xlabel('time')
        self.ax_th.set_ylabel('cart velocity')

        return self.ln_th 

    def plot_th(self, msg):
        yaw_angle = msg.curr_x_dot
        self.y_data_th.append(yaw_angle)
        x_index = len(self.x_data_th)
        self.x_data_th.append(x_index+1)
        
    def plot_px(self, msg):
        yaw_angle = msg.curr_x_dot_dot
        # print(msg.curr_x)
        self.y_data_px.append(yaw_angle)
        x_index = len(self.x_data_px)
        self.x_data_px.append(x_index+1)
    
    def update_plot_px(self, frame):
        self.ln_px.set_data(self.x_data_px, self.y_data_px)
        return self.ln_px

    def update_plot_th(self, frame):
        self.ln_th.set_data(self.x_data_th, self.y_data_th)
        return self.ln_th


rospy.init_node('plotter')
#vis = Visualiser()

vis_pos = Visualiser_position()
#force = rospy.Subscriber('/inverted_pendulum/control_force', ControlForce, vis.odom_callback)

pos = rospy.Subscriber('/inverted_pendulum/current_state', CurrentState, vis_pos.plot_px)
posy = rospy.Subscriber('/inverted_pendulum/current_state', CurrentState, vis_pos.plot_th)

#ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
ani1 = FuncAnimation(vis_pos.fig_px, vis_pos.update_plot_px, init_func=vis_pos.plot_init_px)
ani2 = FuncAnimation(vis_pos.fig_th, vis_pos.update_plot_th, init_func=vis_pos.plot_init_th)
plt.show(block=True)


