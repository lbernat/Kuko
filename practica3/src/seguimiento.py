#!/usr/bin/env python
import cv2
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose
import numpy as np
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()

# Variables globales
HOME_AVANCE = 0.0
PROXIMIDAD = False
estado = -1
    # Señal de control
GIRO   = 0.0
AVANCE = 0.0

# Cuando llega un mensaje al topic \scan
def callback(msg):
    global PROXIMIDAD

    rangos = list(msg.ranges)

    PROXIMIDAD = False
    # Para cada lectura del Láser
    for lectura in rangos:
        # Comrpueba que no sea un valor nulo
        if not math.isnan(lectura): 
            # Comprueba si la lectura supera el umbral
            if lectura < 0.1:       
                PROXIMIDAD = True

def callback_estado(msg):
    global estado
    estado = msg.data

# Función para el cálculo de la 'bounding box'
def bounding_box_area(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_array = np.empty((0, 2), int)

    for landmark in landmarks:
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)

        landmark_point = [np.array((landmark_x, landmark_y))]

        landmark_array = np.append(landmark_array, landmark_point, axis=0)

    x, y, w, h = cv2.boundingRect(landmark_array)

    return [x,y,x+w,y+h]

def loop(msg):
    global AVANCE
    global GIRO
    global HOME_AVANCE
    global PROXIMIDAD
    global estado
    global bridge

    #cap = cv2.VideoCapture(0)
    with mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as pose:
        #while cap.isOpened():
        #    success, image = cap.read()
        #    if not success:
        #        print("Ignoring empty camera frame.")
        #        # If loading a video, use 'break' instead of 'continue'.
        #        continue

        # Convierte la imagen recibida para pdoer procesarla
        image = bridge.imgmsg_to_cv2(msg, msg.encoding)
        
        # Si la tarea actual es la de Cartero
        if estado == 4:
            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Realiza la detección
            results = pose.process(image)

            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # Dibuja líneas 'interfaz' de la 'zona cero'
            image = cv2.line(image, (int(640*0.4), 0), (int(640*0.4), 480), (255, 255, 255), 3)
            image = cv2.line(image, (int(640*0.6), 0), (int(640*0.6), 480), (255, 255, 255), 3)

            # Si se ha detectado a una persona
            if results.pose_landmarks:

                # Obtiene su posición
                x_centroid = ((results.pose_landmarks.landmark[23].x + results.pose_landmarks.landmark[24].x) / 2)

                # En función de su posición define el giro del robot
                if x_centroid < 0.45 and PROXIMIDAD == False:
                    GIRO = 0.4
                elif x_centroid > 0.55 and PROXIMIDAD == False:
                    GIRO = -0.4
                else:
                    GIRO = 0.0

                # Obtiene la bounding box del torso
                landmarks = []
                landmarks.append(results.pose_landmarks.landmark[11])
                landmarks.append(results.pose_landmarks.landmark[12])
                landmarks.append(results.pose_landmarks.landmark[23])
                landmarks.append(results.pose_landmarks.landmark[24])

                x_,y_,x_w,y_h = bounding_box_area(image,landmarks)
                area = (x_w-x_) * (y_h-y_)

                # En función del área de la bounding box define el avance del robot
                if area > HOME_AVANCE + 0.05:
                    AVANCE = 0.2
                elif area < HOME_AVANCE - 0.05:
                    AVANCE = -0.2
                else:
                    AVANCE = 0.0 

                # Enviar comandos al robot
                cmd = Twist()
                cmd.linear.x = AVANCE
                cmd.angular.z = GIRO
                pub.publish(cmd)

                # Muestra donde se ha realizaod la detección de la posición
                image = cv2.line(image, (int(640*x_centroid), 0), (int(640*x_centroid), 480), (0, 0, 255), 7)
                    
                x_min = 1000
                x_max = 0
                y_min = 1000
                y_max = 0
                for landmark in landmarks:
                    if landmark.x < x_min:
                        x_min = landmark.x
                    if landmark.x > x_max:
                        x_max = landmark.x
                    if landmark.y < y_min:
                        y_min = landmark.y
                    if landmark.y > y_max:
                        y_max = landmark.y

                # Muestra la bounding box del torso
                image = cv2.rectangle(image,(int(640*x_min),int(480*y_min)),(int(640*x_max),int(480*y_max)),(0,255,0),3)

                # Establece el origen para el avance
                if cv2.waitKey(5) & 0xFF == ord('s'):
                    HOME_AVANCE = area
            # Muestra la imagen en una ventana
            cv2.imshow('Seguimiento', cv2.flip(image, 1))
            if cv2.waitKey(5) & 0xFF == 27:
                pass

rospy.init_node('control')
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
sub = rospy.Subscriber('/scan', LaserScan, callback)
sub_image = rospy.Subscriber('/camera/rgb/image_raw', Image, loop)
sub_estado = rospy.Subscriber('/estado', Int16, callback_estado)
rospy.spin()