import rospy
import math
import numpy as np
import cv2 as cv
from numpy import empty
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from matplotlib import pyplot as plt
from cv_bridge import CvBridge
from std_msgs.msg import Int16

# Importar al archivo config y los pesos de la red
net = cv.dnn.readNetFromDarknet("./alarm/yolov3.cfg", "./alarm/yolov3.weights")

# Clases a detectar
classes = ['person']

estado = -1

def callback_estado(msg):
    global estado
    estado = msg.data

def callback(ros_img):
    global estado

    if estado == 6:
        
        ##############################
        #  Declaracion de variables  #
        ##############################
        
        person_detected = False
        cmd = Twist()
        giro = 0.0
        #cap = cv.VideoCapture(0) # Descomentar estas dos líneas y cap.release() más abajo y comentar las 2 siguientes si 
        #ret, img = cap.read()    # que quiere probar con la webcam conectada al ordenador en vez de con la cámara del robot
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(ros_img, desired_encoding='passthrough')


        img = cv.resize(img,(1280,720))
        hight,width,_ = img.shape

        ###############################################################
        #                                                             #
        # Código relacionado con detectar la clase en la red neuronal #
        #                                                             #
        ###############################################################
        blob = cv.dnn.blobFromImage(img, 1/255,(416,416),(0,0,0),swapRB = True,crop= False)

        net.setInput(blob)

        output_layers_name = net.getUnconnectedOutLayersNames()

        layerOutputs = net.forward(output_layers_name)

        boxes =[]
        confidences = []
        class_ids = []

        for output in layerOutputs:
            for detection in output:
                score = detection[5:]
                class_id = np.argmax(score)
                confidence = score[class_id]
                if confidence > 0.7:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * hight)
                    w = int(detection[2] * width)
                    h = int(detection[3]* hight)
                    x = int(center_x - w/2)
                    y = int(center_y - h/2)
                    boxes.append([x,y,w,h])
                    confidences.append((float(confidence)))
                    class_ids.append(class_id)


        indexes = cv.dnn.NMSBoxes(boxes,confidences,.5,.4)

        boxes =[]
        confidences = []
        class_ids = []

        for output in layerOutputs:
            for detection in output:
                score = detection[5:]
                class_id = np.argmax(score)
                confidence = score[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * hight)
                    w = int(detection[2] * width)
                    h = int(detection[3]* hight)

                    x = int(center_x - w/2)
                    y = int(center_y - h/2)



                    boxes.append([x,y,w,h])
                    confidences.append((float(confidence)))
                    class_ids.append(class_id)

        indexes = cv.dnn.NMSBoxes(boxes,confidences,.8,.4)
        font = cv.FONT_HERSHEY_PLAIN
        colors = np.random.uniform(0,255,size =(len(boxes),3))
        if  len(indexes)>0:
            for i in indexes.flatten():
                x,y,w,h = boxes[i]
                if class_ids[i] == 0: # Si hay personas detectadas, el robot se pondrá a girar y se imprimirá por pantalla ALERTA INTRUSO
                    person_detected = True
                    
                    label = str(classes[class_ids[i]])
                    confidence = str(round(confidences[i],2)*100)
                    color = colors[i]
                    
                    cv.rectangle(img,(x,y),(x+w,y+h),color,2)
                    cv.putText(img,label + " " + confidence + "%", (x,y+400),font,2,color,2)
                    
                    giro = 10.0
                    print('ALERTA INTRUSO')
                    print('ALERTA INTRUSO')
                    print('ALERTA INTRUSO')

        cmd.angular.z = giro
        img = cv.resize(img, (640, 480))

        cv.imshow('ROS Camera',img)
        cv.waitKey(1)
        #cap.release() 
        pubMove.publish(cmd)
        pubRoam.publish(person_detected)
    


##################
#				 #	
#  Funcion main  #
#				 #
##################

if __name__ == "__main__":

    #########################################################
	#  Declaracion de nodos, suscripciones y publicaciones  #
	#########################################################

    rospy.init_node('people_detection')

    pubMove = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
    pubRoam = rospy.Publisher('/person_detected', Bool, queue_size=0)
    sub = rospy.Subscriber('/camera/rgb/image_raw', Image, callback)
    sub_estado = rospy.Subscriber('/estado', Int16, callback_estado)
    
    rospy.spin()