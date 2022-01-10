#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
import math
import time
import sys

if len(sys.argv) < 3 or len(sys.argv) > 3:
    print('\nERROR!')
    print('Usage: rosrun pkg_name control_position.py x y\n')
    sys.exit()

# Variables globales
media_vuelta = False
initTime     = 0
x = float(sys.argv[1])
y = float(sys.argv[2])
x_pos = 0
y_pos = 0
alpha = 0.0
theta = 0.0

# Función que determina la velocidad de avance
def val_avance(lectura_min):
    global x
    global y
    global x_pos
    global y_pos

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

    if abs(x-x_pos) < 0.25 and abs(y-y_pos) < 0.25:
        avance = 0.0

    return avance

def val_giro(alpha, theta):
    global x
    global y
    global x_pos
    global y_pos

    giro = 0.0

    if alpha - theta > 90:
        giro = 0.7
    elif alpha - theta > 50:
        giro = 0.5
    elif alpha - theta > 10:
        giro = 0.3
    elif alpha - theta > 0:
        giro = 0.1
    elif alpha - theta < 0:
        giro = -0.1
    elif alpha - theta < -10:
        giro = -0.3
    elif alpha - theta < -50:
        giro = -0.5
    elif alpha - theta < -90:
        giro = -0.7

    if abs(x-x_pos) < 0.25 and abs(y-y_pos) < 0.25:
        giro = 0.0

    return giro

# Cuando llega un mensaje al topic \odom
def callback_odom(msg):
    global x
    global y
    global x_pos
    global y_pos
    global alpha
    global theta

    # Obtener alpha
    x_pos = msg.pose.pose.position.x
    y_pos = msg.pose.pose.position.y
    z_pos = msg.pose.pose.position.z
    x_vector = x - x_pos
    y_vector = y - y_pos
    angle = math.degrees(math.acos(((x_vector*1)+(y_vector*0))/(math.sqrt(math.pow(x_vector,2)+math.pow(y_vector,2))*math.sqrt(math.pow(1,2)+math.pow(0,2)))))
    if y_pos > y:
        angle = 360 - angle

    alpha = angle

    # Obtener tetha
    x_or = msg.pose.pose.orientation.x # = 0.0
    y_or = msg.pose.pose.orientation.y # = 0.0
    z_or = msg.pose.pose.orientation.z
    w_or = msg.pose.pose.orientation.w

    theta = abs((math.acos(z_or)*180/math.pi)*2 - 360) -180

# Cuando llega un mensaje al topic \scan
def callback_scan(msg):

    # Variables globales
    global media_vuelta
    global initTime
    global alpha
    global theta

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
        giro = val_giro(alpha, theta)

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

rospy.init_node('control')
pub_reset_odometry = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
timer = time.time()
while time.time() - timer < 1:
    pub_reset_odometry.publish(Empty())
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
sub_scan = rospy.Subscriber('/scan', LaserScan, callback_scan)
sub_odom = rospy.Subscriber('/odom', Odometry, callback_odom)
rospy.spin()