# Kuko - El Robot Oficinista

Kuko es un robot oficinista, encargado de ofrecer asistencia en un entorno laboral para facilitar el trabajo de los empleados. Para ello, este robot dispone de una serie de tareas que pueden ser  útiles en una oficina.

### Pre-requisitos 

MediaPipe:
```
pip install mediapipe
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

[![Alt text](https://img.youtube.com/vi/gE65OlMEMvw/0.jpg)](https://www.youtube.com/watch?v=gE65OlMEMvw)

## Construido con 

* [MediaPipe Pose](https://google.github.io/mediapipe/solutions/pose.html) - Detección de personas
* [Azure Theme](https://github.com/rdbende/Azure-ttk-theme) - Apariencia interfaz



https://user-images.githubusercontent.com/31955512/149568775-aae16804-b771-4ac4-88bf-2ddbb0a0a75e.mp4



## Autores 

* **Lluís Bernat Iborra** 
* **Tamai Ramírez Gordillo** 
* **Israel** 
