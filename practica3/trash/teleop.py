#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('teleop')
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
while not rospy.is_shutdown():
    dato = input("Escribe un comando:(f:forward/l:left/r:right) ")
    cmd = Twist()
    if dato == 'f':
        cmd.linear.x = 0.25
    elif dato == 'l':
        cmd.angular.z = 0.75
        #cmd.linear.x = 0.25  
    elif dato == 'r':
        cmd.angular.z = -0.75
        #cmd.linear.x = 0.25
    else:
        cmd.linear.x = 0
        cmd.angular.z = 0        
    pub.publish(cmd)