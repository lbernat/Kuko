import rospy
import math
import numpy as np
from numpy import empty
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

estado = -1

def callback_estado(msg):
    global estado
    estado = msg.data

# Variables globales para indicar si se han detectado personas o papeles 
person_detected = True
paper_detected = True

###############################################################################################
#																							  #	
#  Funcion para realizar una media de los 5 valores por delante y detras de la medida tomada  #
#																							  #
###############################################################################################
def mean(vector, center):
	meanVector = 0.0 # Variable para realizar la media
	
	# Bucle for para calcular la suma de todos los valores, 11 en total
	for i in range(-5, 5):
		meanVector = meanVector + vector[center+i]
	
	meanVector = meanVector/11 # Se divide el total para obtener la media de los 11 valores
	
	return meanVector # Se retorna el valor obtenido


############################################################################################
#																						   #	
#  Funcion con el algoritmo de evitacion de obstaculos para un rango de medida entre 0-90  #
#																						   #
############################################################################################

def algorythm_90(scanData):
	
	##############################
	#  Declaracion de variables  #
	##############################
	
	distances = list(scanData.ranges) # Vector con las distancias
	#distances.reverse() # Descomentar para el robot real
	cmd = Twist() # Mensaje tipo Twist a publicar en el topic
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
		if mean(distances, i) < 0.6: # Se comprueba si la media de distancias enfrente del robot es menor que 1
			if i <= mid-(0.05*tam): # Si la posicion actual esta por la zona derecha (menor o igual que la posicion media) se gira a la izquierda
				cmd.angular.z = 0.25
				
			else: # En cualquier otro caso se gira a la derecha
				cmd.angular.z = -0.25
			rotating = True
		i = i+1
	if not rotating: # Si no esta rotando, sigue hacia delante
		cmd.linear.x = 0.25
	
	return cmd


###############################################################################################
#																							  #	
#  Funcion con el algoritmo de evitacion de obstaculos para un rango de medida entre 180-270  #
#																							  #
###############################################################################################

def algorythm_180_270(scanData):
	
	##############################
	#  Declaracion de variables  #
	##############################
	
	distances = list(scanData.ranges) # Vector con las distancias
	#distances.reverse() # Descomentar para el robot real
	cmd = Twist() # Mensaje tipo Twist a publicar en el topic
	
	tenPercent = int(len(distances)*0.1) # Entero que representa el 10% del numero de medidas totales (si medidas totales = 100, tenPercent = 10)
	twentyPercent = tenPercent*2 # Lo mismo que el anterior pero es el 20%
	
	initRight = tenPercent # Medida que recoge los valores de la derecha. El primero esta en la posicion tenPercent
	initMid = tenPercent*4 # Medida que recoge los valores del centro. El primero esta en la posicion 4 veces tenPercent
	initLeft = tenPercent*7 # Medida que recoge los valores de la izquierda. El primero esta en la posicion 7 veces tenPercent
	
	i = 0 # Iterador del bucle
	rotating = False # Booleano para saber si tiene que rotar o no
	
	
	#####################
	#  Bucle principal  #
	#####################
	
	while(i < twentyPercent and not rotating): # Mientras i sea menor que twentyPercent y no tenga que rotar
		if np.isnan(distances[i]): # Se comprueba si el numero es nan (Not 
																	#  A 
																	#  Number)
			distances[i] = scanData.range_max # Si es nan, se iguala el valor al maximo medible
			
		if mean(distances, initMid+i) < 0.4: # Se comprueba si la media de distancias enfrente del robot es menor que 1
			if mean(distances, initRight+i) >= 0.4 and mean(distances,initLeft+i) >= 0.4: # Si no hay nada en la izquierda y derecha a menos de 1 unidad 
																					  # se revisa si hay algo en los dos lados entre 1 y el rango maximo
				if mean(distances, initRight+i) > mean(distances, initLeft+i):
					cmd.angular.z = -0.5 # Si hay algun obstaculo a la izquierda, se gira a la derecha 
					
				elif mean(distances, initRight+i) < mean(distances, initLeft+i):
					cmd.angular.z = 0.5 # Si hay algun obstaculo a la derecha, se gira a la izquierda
					
				else: # Si no hay nada en 5 unidades, se revisa en que zona del centro es la medida
					if initMid+i <= initMid+tenPercent:
						cmd.angular.z = 0.5 # Si la medida esta en una posicion inferior o igual al medio, 0 radianes, se gira a la izquierda
						
					else:
						cmd.angular.z = -0.5 # Si la medida esta en una posicion superior al medio, se gira a la derecha
						
				rotating = True
				
			elif mean(distances, initRight+i) >= 0.4 and mean(distances, initLeft+i) < 0.4: # Si hay un obstaculo a la izquierda y no a la derecha se gira a la derecha
				cmd.angular.z = -0.5
				rotating = True
				
			elif mean(distances, initRight+i) < 0.4 and mean(distances, initLeft+i) >= 0.4: # Si hay un obstaculo a la derecha y no a la izquierda, se gira a la izquierda
				cmd.angular.z = 0.5
				rotating = True
		# Los siguientes conndicionales son para los caminos estrechos
		elif mean(distances, initRight+i) < 0.3: # Si hay un obstaculo a menos de 0.3 unidades en la derecha, se gira a la izquierda
			cmd.angular.z = 0.5
			rotating = True
			
		elif mean(distances, initLeft+i) < 0.3: # Si hay un obstaculo a menos de 0.3 unidades en la izquierda, se gira a la derecha
			cmd.angular.z = -0.5
			rotating = True
			
		i = i+1
		
	# Si no esta rotando, sigue hacia delante
	if not rotating:
		cmd.linear.x = 0.5
		
	return cmd
	

def roam(data):
	global person_detected
	person_detected = data.data

def detected_callback(data):
	global paper_detected
	paper_detected=data.data


#############################################################################################
#																		  					#
#  Esta funcion es la que se ejecutara cada vez que se reciba un mensaje en el nodo creado  #
#																		  					#
#############################################################################################

def callback(scanData):

	##############################
	#  Declaracion de variables  #
	##############################

	global person_detected
	global paper_detected
	global estado
	if (not person_detected and estado == 6) or (not paper_detected and estado == 5):
		minAngle = scanData.angle_min*180/math.pi # Angulo minimo 
		maxAngle = scanData.angle_max*180/math.pi # Angulo maximo
		totalAngle = abs(minAngle) + abs(maxAngle) # Angulo total medible
		
		cmd = Twist()
		
		if totalAngle < 100: # Si el angulo medible es menor que 100(100 en vez de 90 por posibles errores en la conversion), se llama a un algoritmo para estas medidas
			cmd = algorythm_90(scanData)
		elif totalAngle > 175 and totalAngle <= 275: # Si el angulo medible es mayor que 175 y menor que 275(175 y 275 en vez de 180 y 270 respectivamente por
													# posibles errores en la conversion), se llama a otro algoritmo diferente 
			cmd = algorythm_180_270(scanData)
			
		
		# Se publica el mensaje 
		pubMove.publish(cmd)


##################
#				 #	
#  Funcion main  #
#				 #
##################

if __name__ == "__main__":
	
	#########################################################
	#  Declaracion de nodos, suscripciones y publicaciones  #
	#########################################################
	
	rospy.init_node('Roamming')

	pubMove = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
	subRoam = rospy.Subscriber('/person_detected', Bool, roam)
	subVag = rospy.Subscriber('/paper', Bool, detected_callback)
	sub = rospy.Subscriber('/scan', LaserScan, callback)
	sub_estado = rospy.Subscriber('/estado', Int16, callback_estado)

	rospy.spin()
