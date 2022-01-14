#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
import roslaunch

estado = -1

def callback_estado(msg):
    global estado
    estado = msg.data

rospy.init_node('map')
sub_estado = rospy.Subscriber('/estado', Int16, callback_estado)

# Loop Infinito
while True:

    # Espera hasta estar en la tarea de Mapeado
    while estado != 1:
        if estado == 0:
            break
    
    if estado == 0:
            break

    # Inicia el mapeado
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/lluis/catkin_ws/src/RMoviles/practica3/launch/mapeado.launch"])
    launch.start()

    # Espera hasta cambiar de tarea
    while estado == 1:
        if estado == 0:
            break

    # Finaliza el mapeado
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch2 = roslaunch.parent.ROSLaunchParent(uuid, ["/home/lluis/catkin_ws/src/RMoviles/practica3/launch/fin_mapeado.launch"])
    launch2.start()
    rospy.sleep(2)
    launch2.shutdown()

    launch.shutdown()

    if estado == 0:
            break