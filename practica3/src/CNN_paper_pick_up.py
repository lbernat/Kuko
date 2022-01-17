import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from gazebo_msgs.srv import DeleteModel # For deleting models from the environment
from std_msgs.msg import Int16

class Nodo(object):
    def __init__(self):
        # Params
        self.estado = -1
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)
        # Publishers
        self.pub_vag= rospy.Publisher('/paper', Bool,queue_size=5)
        self.paper=Bool()
        self.paper.data=False
        # Subscribers
        self.sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        self.sub_estado = rospy.Subscriber('/estado', Int16, self.callback_estado)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.net = cv2.dnn.readNetFromDarknet("./trash/yolov3_custom.cfg",r"./trash/yolov3_custom_final.weights")
        self.classes = ['can','paper']
    
    def callback(self, msg):
        #rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)

    def callback_estado(self, msg):
        self.estado = msg.data

    def start(self):
        while True: 
            if self.image is not None and self.estado == 5:
                img = self.image
                self.paper.data=False
                img = cv2.resize(img,(720,480))
                hight,width,_ = img.shape
                blob = cv2.dnn.blobFromImage(img, 1/255,(416,416),(0,0,0),swapRB = True,crop= False)

                self.net.setInput(blob)

                output_layers_name = self.net.getUnconnectedOutLayersNames()

                layerOutputs = self.net.forward(output_layers_name)

                boxes =[]
                confidences = []
                class_ids = []

                for output in layerOutputs:
                    for detection in output:
                        score = detection[5:]
                        class_id = np.argmax(score)
                        confidence = score[class_id]
                        if confidence > 0.9:
                            center_x = int(detection[0] * width)
                            center_y = int(detection[1] * hight)
                            w = int(detection[2] * width)
                            h = int(detection[3]* hight)
                            x = int(center_x - w/2)
                            y = int(center_y - h/2)
                            boxes.append([x,y,w,h])
                            confidences.append((float(confidence)))
                            class_ids.append(class_id)

                indexes = cv2.dnn.NMSBoxes(boxes,confidences,.9,.7)
                font = cv2.FONT_HERSHEY_PLAIN
                if  len(indexes)>0:
                    for i in indexes.flatten():
                        x,y,w,h = boxes[i]
                        label = str(self.classes[class_ids[i]])
                        confidence = str(round(confidences[i],2))
                        color = (0,255,0)
                        cv2.rectangle(img,(x,y),(x+w,y+h),color,2)
                        cv2.putText(img,label + " " + confidence, (x,y+400),font,2,color,2)

                        giro=(width/2-x-w)/width
                        avance=(hight-y-h)/hight
                        
                        cmd = Twist()
                        cmd.linear.x = avance*0.5
                        cmd.angular.z = giro*0.5
                        if abs(avance) <= 10**-1 and abs(giro) <= 10**-1:
                            print("Paper colected")
                            self.paper.data=False
                            cmd.linear.x = 0.0
                            cmd.angular.z = 5.0
                        self.pub.publish(cmd)    

                self.pub_vag.publish(self.paper)         
                cv2.imshow('Trash tracking',img)
                if cv2.waitKey(1) == ord('q'):
                    self.cap.release()
                    cv2.destroyAllWindows()
                    break

if __name__ == '__main__':
    rospy.init_node("trash_tracking", anonymous=True)
    my_node = Nodo()
    my_node.start()
