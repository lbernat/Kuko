#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped

# Guarda la posición recibida en un fichero .txt
def callback(msg):
    print()
    print('-----------------------------')
    name = input('Nombre posicion: ')
    x = msg.point.x
    y = msg.point.y
    
    f = open("/home/lluis/catkin_ws/src/RMoviles/practica3/src/positions.txt","a")
    f.write(str(name) + "," + str(x) + "," + str(y) + "\n")
    f.close()

    print('Posicion guardada!')

rospy.init_node('save_points', anonymous=True)
rospy.Subscriber("/clicked_point", PointStamped, callback)
rospy.spin()