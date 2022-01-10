#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
import tkinter as tk
from tkinter import messagebox, ttk

pub = rospy.Publisher('estado', Int16, queue_size=10)
rospy.init_node('menu', anonymous=True)

root = tk.Tk()
root.config(width=300, height=310)
root.title("Kuko")

# Establecer tema para la interfaz
root.tk.call("source", "/home/lluis/catkin_ws/src/RMoviles/practica3/src/azure_theme/azure.tcl")
root.tk.call("set_theme", "light")

frame = tk.LabelFrame(root, text='ESTADO ACTUAL', width=100, height=50)
frame.place(x=160, y=20)

text = tk.StringVar()
text.set("reposo")
label2 = tk.Label(frame, textvariable=text)
label2.pack()

# ESTADO 1
def mapeado(event=None):
    text.set("mapeado")
    pub.publish(1)

# ESTADO 2
def config(event=None):
    text.set("config")
    pub.publish(2)

# ESTADO 3
def cartero(event=None):
    text.set("cartero")
    pub.publish(3)

# ESTADO 4
def seguimiento(event=None):
    text.set("seguimiento")
    pub.publish(4)

# ESTADO 5
def seguimiento2(event=None):
    text.set("seguimiento")
    pub.publish(5)

boton_reposo = ttk.Button(text="APAGAR", style="Accent.TButton", command=root.destroy)
boton_reposo.place(x=170, y=260)

boton_mapeado = ttk.Button(text="Mapeado", command=mapeado)
boton_mapeado.bind("<Return>", mapeado)
boton_mapeado.place(x=30, y=20)

boton_cartero = ttk.Button(text="Config.", command=config)
boton_cartero.bind("<Return>", config)
boton_cartero.place(x=30, y=80)

boton_alarma = ttk.Button(text="Cartero", command=cartero)
boton_alarma.bind("<Return>", cartero)
boton_alarma.place(x=30, y=140)

boton_basura = ttk.Button(text="Seguimiento", command=seguimiento)
boton_basura.bind("<Return>", seguimiento)
boton_basura.place(x=30, y=200)

boton_seguimiento = ttk.Button(text="Mapeado", command=seguimiento2)
boton_seguimiento.bind("<Return>", seguimiento2)
boton_seguimiento.place(x=30, y=260)

root.mainloop()