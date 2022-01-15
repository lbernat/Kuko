# Kuko - El Robot Oficinista

Kuko es un robot oficinista, encargado de ofrecer asistencia en un entorno laboral para facilitar el trabajo de los empleados. Para ello, este robot dispone de una serie de tareas que pueden ser  útiles en una oficina.

### Pre-requisitos 

MediaPipe:
```
pip install mediapipe
```

TensorFlow:
```
pip install tensorflow
```

ROS Noeitc: seguir las [intrucciones de instalación](http://wiki.ros.org/noetic#Installation).

### Instalación 

```
cd catkin_ws/src

git clone https://github.com/lbernat/Kuko.git

cd ..

catkin_make

source devel/setup.bash
```

## Ejecutando las pruebas

1. Lanzar turtlebot2
```
# En simulación Gazebo
source $HOME/tb2_ws/devel_isolated/setup.bash
roslaunch turtlebot_gazebo turtlebot_world.launch

# En turtlebot real
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_bringup hokuyo_ust10lx.launch
roslaunch astra_launch astra.launch
```

2. Lanzar el software de Kuko
```
roslaunch all.launch
```

En el siguiente vídeo se muestra la ejecución del software:

https://user-images.githubusercontent.com/31955512/149568775-aae16804-b771-4ac4-88bf-2ddbb0a0a75e.mp4

## Construido con 

* [MediaPipe Pose](https://google.github.io/mediapipe/solutions/pose.html) - Detección de personas
* [TensorFlow](https://www.tensorflow.org/) - Detección de basura
* [Azure Theme](https://github.com/rdbende/Azure-ttk-theme) - Apariencia interfaz

## Autores 

* **Lluís Bernat Iborra** 
* **Tamai Ramírez Gordillo** 
* **Israel Hernández Ramírez** 
