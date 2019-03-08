#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Sep 30 22:52:41 2018

@author: baozhang
"""
import rospy
from turtlebot_ctrl.msg import TurtleBotState
from std_msgs.msg import Float32
def stop_all(msg):
    x_data = msg.x.data
    y_data = msg.y.data
    if msg.goal_reached.data == True: 
        print "Reach Goal State!"
        #stop whee program
    '''else:
        print msg.goal_reached.data'''
    
rospy.init_node('Turtlebot_State_Scriber')
sub = rospy.Subscriber('/turtlebot_state', TurtleBotState, stop_all)

rospy.spin()
