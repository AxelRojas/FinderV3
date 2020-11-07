#!/usr/bin/env python
# coding=utf-8

import rospy
#from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import Matriz_de_distancias as md
import time


class Nodo:
    def __init__(self,twist):
        self.init_node=rospy.init_node("Exploration_and_Mapping", anonymous=True)
        self.laser=[]
        self.twist= twist
        self.dato=0
        self.mapa=0
        self.objetivos=[]
        self.pos_x=0
        self.pos_y=0
        self.posicion=0
        self.nuevo_grafo=0
        self.distancias=0
        self.min_x_anterior=400
        self.min_y_anterior=400
        self.max_x_anterior=10
        self.max_y_anterior=10
        
        
        

    def posicion_robot_callback(self,data):
        #Obtendre la posicion del laser a partir de la posicion de base_footprint
        self.pos_x=data.pose.position.x
        self.pos_y=data.pose.position.y
    
    #Se obtiene un arreglo bidimensional que se transforma en un arreglo bidimensional, en donde las columnas corresponden a las coordenadas en x en el mapa de rviz
    #Mientras que las filas corresponden a las coordenadas en y. Una vez obtenido el mapa, se procede a alterar los valores de la matriz, con el fin de prepararlo tanto para la busqueda de objetivos, como para la evaluacion de distancias
    #Se cambian los valores conocidos (originalmente 0) a valores de 1
    #Mientras que los valores de espacio desconocido (valor de -1) se colocan en 0
    #Una vez se tiene esto, se procede a la busqueda de objetivos guiandonos con la posicion del elemento de la matriz, con ello determinamos el numero de nodos alrededor del actual que tenemos que buscar (se debe pulir ya que algunos puntos se repiten, debido a que no se gueardan los valores que ya se compararon, y ademas son demasiados puntos los que se analizan)
    #Una vez obtenido los puntos, se envian al metodo que los graficara, los puntos estan en coordenadas (x,y) y los valores de los puntos se encuentran invertidos (por lo mismo que se comento con los sistemas de referencia en rviz)    
    #Los puntos necesitan multiplicarse por el factor de escala que es 0.05 aprocimadamente (ya que es la conversion necesaria para pasar de puntos de la matriz a puntos en el mapa)
    #Posteriormente se les resta a estos puntos 10.27 aproximadamente, esto debido a que el origen se encuentra justo en el medio del mapa, y para situarnos en el 0 de la matriz, necesitamos recorrernos -10.27 en ambos ejes (ese valor se obtiene de datos del mapa, y se d¿hace un pequeño ajuste ya que los puntos se ven un poco desfasados)
    #Se publican a una frecuencia de 15 por segundo y en principio despues de esto vendria el filtrado de puntos objetivos
    def map_callback(self,dato):
        #Estoy reacomodando el arreglo para poder manejarlo como una matriz, basandome en la altura y ancho
        self.dato=dato
        #-------------------------------- Estoy preparando una matriz, a partir de todos los puntos que me entrega rviz como arreglo, el elemento 0,0 se encuentra u origen se encuentra en la esquina superior izq, desde ahi, rviz empieza el arreglo que obtenemos en /map
        #Sin embargo, el sistema de referencia del mapa se encuentra justo en el medio, por lo cual debemos de hacer un ajuste para movernos en esta matriz, seria a cada punto restarle la posicion original obtenida del topico /map en origin/posicion y multiplicar los indices por el factor de escala de 0.05
        self.mapa=np.array(self.dato.data).reshape((self.dato.info.height, self.dato.info.width))#[(self.dato.info.height/2)+self.pos_x-40:(self.dato.info.height/2)+self.pos_x+40,(self.dato.info.width/2)-self.pos_y-40:(self.dato.info.width/2)-self.pos_y+40]
        #Las filas de la matriz corresponden a las coordenadas y
        #Las columnas de la matriz corresponden a las coordenas en x
        self.mapa.flags.writeable = True
        self.mapa[self.mapa==0]=1
        self.mapa[self.mapa==-1]=0
        self.objetivos=md.Busqueda_Objetivos(self.mapa)
        list(set(self.objetivos))#Elimina los elementos repetidos en mis puntos objetivos
        #md.visualizacion_objetivos(self.objetivos,self.dato)
        #self.objetivos=md.Filtrado_de_objetivos(self)
        if len(self.objetivos)!=0:
            md.visualizacion_objetivos(self.objetivos,self.dato)
        
        #self.nuevo_grafo=md.convertir_matriz(self.mapa)
        #self.objetivos=md.calcularDistancias(self)
        #md.visualizacion_objetivos(self.objetivos,self.dato)
        #print(distancias_min_cada_objetivo)
         


          
        #------------------------------------------------------------------
        
    
    
if __name__ == "__main__":
    nodo=Nodo(Twist())
    #rospy.Subscriber('/slam_out_pose',PoseStamped,nodo.posicion_robot_callback,queue_size=1)
    rospy.Subscriber('/map',OccupancyGrid,nodo.map_callback,queue_size=100)
    
    rospy.spin()

    
      
        
    
