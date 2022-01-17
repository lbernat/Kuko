#!/usr/bin/env python3
 
# Author: Automatic Addison https://automaticaddison.com
# Description: An example of a basic finite state machine for a turnstile at 
#   a stadium or metro station.
 
# Import the necessary libraries
import rospy # Python client library
from smach import State, StateMachine # State machine library
import smach_ros # Extensions for SMACH library to integrate it with ROS
from time import sleep # Handle time
import math
import numpy as np
from numpy import empty
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from gazebo_msgs.srv import DeleteModel # For deleting models from the environment
from std_msgs.msg import Int16

# Define state VAGABUNDEO
class Vagabundeo(State):
    def __init__(self):
        State.__init__(self, outcomes=['no_change','vag_to_trash'])
        self.estado = -1
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)
        self.pubMove = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.sub_estado = rospy.Subscriber('/estado', Int16, self.callback_estado)
        self.sub_image = rospy.Subscriber("/camera/rgb/image_raw",Image,self.image_callback)
        self.scan=None
    # Inside this block, you can execute any code you want

    def image_callback(self, msg):
        #rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)

    def callback_estado(self, msg):
        self.estado = msg.data

    def mean(self,vector, center):
        meanVector = 0.0 # Variable para realizar la media
        
        # Bucle for para calcular la suma de todos los valores, 11 en total
        for i in range(-5, 5):
            meanVector = meanVector + vector[center+i]
        
        meanVector = meanVector/11 # Se divide el total para obtener la media de los 11 valores
        
        return meanVector # Se retorna el valor obtenido

    def algorythm_90(self,scanData):
        ##############################
        #  Declaracion de variables  #
        ##############################
        
        distances = list(scanData.ranges)	# Vector con las distancias
        #distances.reverse()
        #print(distances)
        cmd = Twist() # Mensaje tipo Twist a publicar en el topic
        #print(cmd)
        tam = len(distances) # Tamanio total del vector de distancias
        mid = int(tam/2) # Valor intermedio

        i = 0
        while(i < tam):
            if np.isnan(distances[i]): # Se comprueba si el numero es nan (Not 
                                                                        #  A 
                                                                        #  Number)
                distances[i] = scanData.range_max # Si es nan, se iguala el valor al maximo medible
            i = i+1
        
        i = 4 # Iterador del bucle. Empieza en 4 porque al tener un rango igual o menor a 90, tomamos todos los datos, 
                # asi que para hacer una media con los 5 datos anteriores no se puede si estamos entre los 4 datos primeros
        
        rotating = False # Booleano para saber si tiene que rotar o no

        
        while(i < tam-4 and not rotating):
            #print(i)
            
                
            #print(mean(distances, i))
            if self.mean(distances, i) < 0.6: # Se comprueba si la media de distancias enfrente del robot es menor que 1
                #print(distances[i])
                if i <= mid-(0.05*tam): # Si la posicion actual esta por la zona derecha (menor o igual que la posicion media) se gira a la izquierda
                    cmd.angular.z = 0.25
                    
                else: # En cualquier otro caso se gira a la derecha
                    cmd.angular.z = -0.25
                rotating = True
            i = i+1
        #print(i)
        if not rotating: # Si no esta rotando, sigue hacia delante
            cmd.linear.x = 0.25
        
        #print(cmd)
        
        return cmd

    def callback(self,scanData):
        self.scan=scanData
        

    def execute(self,userdata):
        sleep(1)
        rospy.loginfo('Executing state Vagabundeo')
        while 1:
            if self.estado == 5:
                scanData=self.scan
                    
                # When a state finishes, an outcome is returned. An outcome is a 
                # user-defined string that describes how a state finishes.
                # The transition to the next state is based on this outcome
                
                #rate = rospy.Rate(10) # El codigo se ejecutara a 10Hz  			# Se ha comentado esta línea
                minAngle = scanData.angle_min*180/math.pi # Angulo minimo 
                maxAngle = scanData.angle_max*180/math.pi # Angulo maximo
                totalAngle = abs(minAngle) + abs(maxAngle) # Angulo total medible
                
                cmd = Twist()
                cmd = self.algorythm_90(scanData)
                # Se publica el mensaje y se duerme el programa durante 0.1s
            
                self.pubMove.publish(cmd)
                
                if self.image is not None:
                    frame = self.image
                    #frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
                    hight,width,_ = frame.shape
                    #HSV = cv2.cvtColor(frame,cv2.COLOR_RGB2HSV)
                    HSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
                    x,y,w,h=1000,1000,1000,1000
                    # 2- define the range of red
                    lower=np.array([10, 0, 0])
                    upper=np.array([255, 255,255])

                    #check if the HSV of the frame is lower or upper red
                    Red_mask = cv2.inRange(HSV,lower, upper)
                    result = cv2.bitwise_and(frame, frame, mask = Red_mask)

                    # Draw rectangular bounded line on the detected red area
                    contours, hierarchy = cv2.findContours(Red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    area=0
                    for pic,contour in enumerate(contours):
                        area = cv2.contourArea(contour)
                        if(area > 500): #to remove the noise
                            return 'vag_to_trash'


    
# Define state TRASH
class Trash(State):
    def __init__(self):
        State.__init__(self, outcomes=['no_change','trash_to_vag'])
        # Params
        self.estado = -1
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # # Publishers
        # self.pub_vag= rospy.Publisher('/paper', Bool,queue_size=5)
        # self.paper=Bool()
        # self.paper.data=False
        # Subscribers
        self.sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        self.sub_estado = rospy.Subscriber('/estado', Int16, self.callback_estado)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)

    def callback(self, msg):
        #rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)
    
    def callback_estado(self, msg):
        self.estado = msg.data
    
    def execute(self,userdata):
        sleep(1)
        rospy.loginfo('Executing state Trash')
        while 1:
            if self.image is not None and self.estado == 5:
                frame = self.image
                #frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
                hight,width,_ = frame.shape
                #HSV = cv2.cvtColor(frame,cv2.COLOR_RGB2HSV)
                HSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
                x,y,w,h=1000,1000,1000,1000
                # 2- define the range of red
                lower=np.array([10, 0, 0])
                upper=np.array([255, 255,255])

                #check if the HSV of the frame is lower or upper red
                Red_mask = cv2.inRange(HSV,lower, upper)
                result = cv2.bitwise_and(frame, frame, mask = Red_mask)

                # Draw rectangular bounded line on the detected red area
                contours, hierarchy = cv2.findContours(Red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                area=0
                for pic,contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    if(area > 500): #to remove the noise
                        # Constructing the size of boxes to be drawn around the detected red area
                        x,y,w,h = cv2.boundingRect(contour)
                        frame = cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
                        giro=(width/2-x-w)/width
                        avance=(hight-y-h)/hight
                        cmd = Twist()
                        cmd.linear.x = avance*0.5
                        cmd.angular.z = giro*0.5
                        self.pub.publish(cmd)
                        if abs(avance) <= 10**-1 and abs(giro) <= 10**-1:
                            print("Trash colected")
                            del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel) # Handle to model spawner
                            del_model_prox("coke_can") # Remove from Gazebo
                            return 'trash_to_vag'
                    
    # Main method
def main():

    # Initialize the node
    rospy.init_node('state_machine')
    
    # Create a SMACH state machine container
    sm = StateMachine(outcomes=['succeeded','failed'])
        
    # Open the state machine container. A state machine container holds a number of states.
    with sm:
        
        # Add states to the container, and specify the transitions between states
        # For example, if the outcome of state LOCKED is 'coin', then we transition to state UNLOCKED.
        StateMachine.add('Vagabundeo', Vagabundeo(), transitions={'vag_to_trash':'Trash','no_change':'Vagabundeo'})
        StateMachine.add('Trash', Trash(), transitions={'trash_to_vag':'Vagabundeo','no_change':'Trash'})
    
    # View our state transitions using ROS by creating and starting the instrospection server
    # sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    # sis.start()
    
    # Execute the state machine 
    outcome = sm.execute()
 
if __name__ == '__main__':
    main()
