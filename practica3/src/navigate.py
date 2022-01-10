#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Int16

estado = -1
move = False

#Uso de la acción move_base en ROS para moverse a un punto determinado
#En ROS una acción es como una petición de un "cliente" a un "servidor"
#En este caso este código es el cliente y el servidor es ROS
#(en concreto el nodo de ROS 'move_base')
class ClienteMoveBase:
    def __init__(self):
        #creamos un cliente ROS para la acción, necesitamos el nombre del nodo 
        #y la clase Python que implementan la acción
        #Para mover al robot, estos valores son "move_base" y MoveBaseAction
        self.client =  actionlib.SimpleActionClient('move_base',MoveBaseAction)
        #esperamos hasta que el nodo 'move_base' esté activo`
        self.client.wait_for_server()

    def moveTo(self, x, y):
        #un MoveBaseGoal es un punto objetivo al que nos queremos mover
        goal = MoveBaseGoal()
        #sistema de referencia que estamos usando
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x   
        goal.target_pose.pose.position.y = y
        #La orientación es un quaternion. Tenemos que fijar alguno de sus componentes
        goal.target_pose.pose.orientation.w = 1.0

        #enviamos el goal 
        self.client.send_goal(goal)
        #vamos a comprobar cada cierto tiempo si se ha cumplido el goal
        #get_state obtiene el resultado de la acción 
        state = self.client.get_state()
        #ACTIVE es que está en ejecución, PENDING que todavía no ha empezado
        while state==GoalStatus.ACTIVE or state==GoalStatus.PENDING:
            rospy.Rate(10)   #esto nos da la oportunidad de escuchar mensajes de ROS
            state = self.client.get_state()
        return self.client.get_result()

def callback_estado(msg):
    global move
    global estado
    estado = msg.data
    if msg.data == 3:
        move = True

rospy.init_node('navigate')
sub_estado = rospy.Subscriber('/estado', Int16, callback_estado)
cliente = ClienteMoveBase()

while True:

    if estado == 0:
        break

    if move == True:
        # Leer fichero
        positions = []
        with open("/home/lluis/catkin_ws/src/RMoviles/practica3/src/positions.txt","r") as archivo:
            for linea in archivo:
                info = linea.split(",")
                print(info)
                positions.append([info[0],float(info[1]),float(info[2])])
                print(linea)

        # Input
        i = 1
        print()
        print("############################")
        print("### POSICIONES GUARDADAS ###")
        for position in positions:
            print(str(i) + " -> " + position[0])
            i += 1
        print("############################")

        select = int(input('Elija una posicion: ')) - 1

        # Mover
        result = cliente.moveTo(positions[select][1],positions[select][2])
        print(result)
        if result:
            rospy.loginfo("Goal conseguido!")

        move = False