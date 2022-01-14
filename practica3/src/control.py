#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Int16
import math
import time

# Variables globales
media_vuelta = False
initTime     = 0
active = False
estado = False

# Función que determina la velocidad de avance
def val_avance(lectura_min):
    avance = 0.0
    if lectura_min < 0.3: 
        avance = 0.0
    elif lectura_min < 0.4: 
        avance = 0.025
    elif lectura_min < 0.5:
        avance = 0.05
    elif lectura_min < 0.6: 
        avance = 0.1
    elif lectura_min < 0.7: 
        avance = 0.15
    elif lectura_min < 0.8: 
        avance = 0.2
    elif lectura_min < 0.9:
        avance = 0.25
    elif lectura_min < 1:
        avance = 0.3
    elif lectura_min < 1.1: 
        avance = 0.35
    else:
        avance = 0.4
    return avance

# Cuando llega un mensaje al topic \scan
def callback(msg):

    # Variables globales
    global media_vuelta
    global initTime
    global active
    global estado

    if active and estado:
        # Varibels locales
        avance = 0.0
        giro = 0.0
        lectura_min = 1000.0
        i = 0
        izquierda = 0
        derecha = 0
        rangos = list(msg.ranges)
        # In real robot:
        #rangos.reverse()

        # Para cada lectura del Láser
        for lectura in rangos:
            # Comrpueba que no sea un valor nulo
            if not math.isnan(lectura): 
                # Comprueba si la lectura supera el umbral
                if lectura < 0.75:       
                    # Determina a qué lado pertenece la lectura
                    if msg.angle_min + msg.angle_increment*i <= 0:
                        izquierda += 1
                    else:
                        derecha += 1
                # Comprueba si es la lectura menor
                if lectura < lectura_min:
                    lectura_min = lectura
            i+=1

        ## AVANCE
        avance = val_avance(lectura_min)

        ## GIRO
        if (derecha > i/5 and izquierda > i/5) or media_vuelta: # Media vuelta
            print('media vuelta')
            avance = 0.0
            giro  = 0.7
            if media_vuelta == True and time.time() - initTime >= 4.5:
                media_vuelta = False
                initTime = 0
            elif initTime == 0:
                media_vuelta = True
                initTime = time.time()
        elif derecha > izquierda and not media_vuelta:          # Girar a la izquierda
            print('girar a la izquierda')
            giro = -0.7
        elif izquierda > derecha and not media_vuelta:          # Girar a la derecha
            print('girar a la derecha')
            giro = 0.7
        elif not media_vuelta:                                  # Seguir recto
            print('sigue recto')
            giro = 0.0
        
        # Mostrar información por terminal (interfaz)
        print('Avance: ' + str(avance))
        print('Giro: ' + str(giro))
        print('------------------------')

        # Enviar comandos al robot
        cmd = Twist()
        cmd.linear.x = avance
        cmd.angular.z = giro
        pub.publish(cmd)
        #rospy.sleep(1)

# Activar el modo auto
def callback_auto(msg):
    global active

    active = msg.data

# Recibir el estado del sistema
def callback_estado(msg):
    global estado

    if msg.data == 1:
        estado = True
    else:
        estado = False

rospy.init_node('control')
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
sub = rospy.Subscriber('/scan', LaserScan, callback)
sub_active = rospy.Subscriber('/auto', Bool, callback_auto)
sub_estado = rospy.Subscriber('/estado', Int16, callback_estado)
rospy.spin()