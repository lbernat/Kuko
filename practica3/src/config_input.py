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

    # Espera hasta estar en la tarea de Configuración
    while estado != 2:
        if estado == 0:
            break
    
    if estado == 0:
            break

    # Inicia el stack de navegación y el entorno RViz
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/lluis/catkin_ws/src/RMoviles/practica3/launch/config.launch"])
    launch.start()

    # Espera hasta cambiar de tarea
    while estado == 2:
        if estado == 0:
            break

    launch.shutdown()

    if estado == 0:
            break