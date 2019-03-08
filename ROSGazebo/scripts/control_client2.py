#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Sep 30 23:11:12 2018

@author: baozhang
"""

import rospy
from turtlebot_ctrl.srv import TurtleBotControl, TurtleBotControlRequest
from geometry_msgs.msg import Point
import ast

rospy.init_node('service_control_client')
rospy.wait_for_service('/turtlebot_control')
control_service_client = rospy.ServiceProxy('/turtlebot_control', TurtleBotControl)
control_request_object = TurtleBotControlRequest()
print "in"
#get file


file_path = '/home/baozhang/Downloads/map1.txt'


f = open(file_path)
routs = []
temp = []
for line in f:
    # print(line)
    temp.append(line)
#
for i in range(len(temp)):
    routs.append(ast.literal_eval(temp[i]))

path = [routs[0][0],routs[0][1]]

while len(path)>0:
    
    cp = path.pop(0)
    '''x = Float64()
    x.data = cp[0]
    y = Float64()
    y.data = cp[1]
    z = Float64()
    z.data = cp[2]'''
    current_point = Point(float(cp[0]),float(cp[1]),float(0))
    control_request_object.point = current_point
    result = control_service_client(control_request_object)
    print result
print "No More Nodes" 
