#!/usr/bin/env python
# coding=utf-8
import rospy
import numpy as np
import networkx as nx
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def calcularDistancias(self):
    caminos=[]
    #Tengo que hacer un ajuste, ya que necesito saber la posicion del laser para poder buscar la distancia minima, por lo cual le sumo a la distancia de base_link 0.45 en x para saber la posicion y le sumo 0.1 a todos para asegurar que se encuentre dentro de mi matriz, es una celda despues de donde esta el laser
    abscisas=[]
    ordenadas=[]
    for (x,y) in self.objetivos:
        abscisas.append(x)
        ordenadas.append(y)
    a=np.where(ordenadas>=int(214+(self.pos_y/self.dato.info.resolution)))[0]
    coordenadas=(abscisas[a[0]],ordenadas[0])
    nx.shortest_path(self.nuevo_grafo,source=coordenadas)
    for i in self.objetivos:
       caminos.append(nx.shortest_path(self.nuevo_grafo,source=coordenadas,target=i))
    return caminos


def visualizacion_objetivos(objetivos,mapa):
    
    pub =rospy.Publisher('/visualization_marker',Marker, queue_size=50)
    puntos=Marker(ns="puntos_objetivo",type=Marker.POINTS,action=Marker.ADD,lifetime=rospy.Duration(),id=0)
    puntos.header.stamp=rospy.Time()
    puntos.header.frame_id="/map"
    puntos.pose.orientation.w=1.0
    #Esta es la posicion de la marca
    puntos.pose.position.x=mapa.info.origin.position.x+0.025 #La referencia se encuentra en la esquina inferior derecha se se ve el robot avanzando hacia enfrente, en el punto más alejado verde en una esquina, y se encuentra a -10.275 positivos en ambos ejes
    puntos.pose.position.y=mapa.info.origin.position.y+0.025
    puntos.pose.position.z=0
    #Puntos
    puntos.scale.x=0.04
    puntos.scale.y=0.04
    #Los puntos seran rojos
    puntos.color.g=1.0
    puntos.color.a=1.0
    
    """o=Point()
    o.x=0
    o.y=0
    o.z=0
    puntos.points.append(o)"""
    #Son las posiciones de los puntos
    
    for (x,y) in objetivos:
        p=Point()
        p.x=y*(mapa.info.resolution)#Se hizo una regla de tres o se multiplico por la resolución del mapa para ajustar los valores de la matriz a los valores de 
        p.y=x*(mapa.info.resolution)
        p.z=0
        puntos.points.append(p)
    
    
    rate = rospy.Rate(20)#defino que los datos se publicaran 15/s
    pub.publish(puntos)
    rate.sleep()
        
        
def convertir_matriz(grafo):
    """#Entra un grafo de 40x40 que es subgrafo que envuelve al robot en un area cuadrada, en la cual se hara la busqueda de objetivos.
    #Este grafo fue modificado colocando como 1 los valores conocidos, esto porque se consideran grafos conectados, y a los demas valores se les ha colocado un 0.
    #Segun la imagen enviada, la matriz debe de ser transformada a una matriz que relaciones puntos en el mapa, ya que en la matriz original se relacionaban coordenadas x= Filas, y=Columnas
    #Obtengo la forma del grafo ya que el numero de columnas y filas de la nueva matriz esta en funcion de las filas y columnas de la matriz original.
    nuevo_grafo=np.zeros((c*c,l*l))#Este sera el grafo que se utilizara para el reacomodo de la matriz
    #Una vez se tiene la matriz se procedera a recorrer todas las filas (i) para cada columna, esto con la intension de reacomodar la matriz segun el algoritmo pensado.
    #Se encuentran las variables d, a, iz, b que representan los posibles casos en los cuales puede entrar el elemento que buscamos sus vecinos.
    #Al iniciar se cae en la primera condicion de cuando i=j=0, esto quiere decir que solamente existen 2 posibles vecinos, a la derecha (d) y debajo (b), las formulas se obtuvieron de acuerdo a la forma de convertir un punto especifico a el valor de la nueva matriz
    #Posteriormente tenemos a k, que representa la expresion con la cual podemos obtener el valor que tiene un punto especifico en las nuevas filas de mi matriz, como ejemplo el elemento (0,0) seria en la nueva matriz el elemento 0 de las filas.
    #El ciclo termina una vez se ha recorrido toda la matriz y remapeado la misma, cayendo en todos los casos, y se puede verificar con una matriz de 3*3 como se muestra en la imagen anexada.
    #Para saber si el "nodo" esta conectado, se evalua el producto de 2 elemento del grafo original: supongamos que estamos en (0,0), este elemento tiene 2 posibles conexiones, por lo cual se multiplicara el valor de (0,0)*(0,1) y (0,0)*(1,0) si el producto da 1, esto quiere decir que estaban conectados (ya que ambos son espacio conocido), en caso contrario sera 0"""
    c, l=np.shape(grafo)
    #Se crea un grafo no dirigido, esto debido a que la forma de crear el grafo hace que mi grafo caiga dentro de esta clasificación
    #Cuando creo una arista, entre dos puntos, automaticamente creo los nodos (en este caso puntos) y estos no se sobreescriben.
    nuevo_grafo=nx.Graph()
    for j in range(l):
        for i in range(c): 
            """d=(i*c)+(j+1)
            a=((i-1)*c)+j
            iz=(i*c)+(j-1)
            b=((i+1)*c)+j
            k=i+(j*l)"""
            if i==0 and j==0:
                #nuevo_grafo[k,d]=grafo[i,j]*grafo[i,j+1]
                #nuevo_grafo[k,b]=grafo[i,j]*grafo[i+1,j]
                if (grafo[i,j]*grafo[i,j+1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j+1),weight=grafo[i,j]*grafo[i,j+1])

                if (grafo[i,j]*grafo[i+1,j])==1 :
                    nuevo_grafo.add_edge((i,j),(i+1,j),weight=grafo[i,j]*grafo[i+1,j])

            elif i==0 and j>0 and j!=l-1:
                #nuevo_grafo[k,iz]=grafo[i,j]*grafo[i,j-1]
                #nuevo_grafo[k,d]=grafo[i,j]*grafo[i,j+1]
                #nuevo_grafo[k,b]=grafo[i,j]*grafo[i+1,j]
                if (grafo[i,j]*grafo[i,j+1])==1 :
                    nuevo_grafo.add_edge((i,j),(i,j+1),weight=grafo[i,j]*grafo[i,j+1])

                if (grafo[i,j]*grafo[i+1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i+1,j),weight=grafo[i,j]*grafo[i+1,j])

                if (grafo[i,j]*grafo[i,j-1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j-1),weight=grafo[i,j]*grafo[i,j-1])

            elif i==0 and j==l-1:
                #nuevo_grafo[k,iz]=grafo[i,j]*grafo[i,j-1]
                #nuevo_grafo[k,b]=grafo[i,j]*grafo[i+1,j]
                if (grafo[i,j]*grafo[i,j-1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j-1),weight=grafo[i,j]*grafo[i,j-1])

                if (grafo[i,j]*grafo[i+1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i+1,j),weight=grafo[i,j]*grafo[i+1,j])

            elif i>0 and i !=c-1 and j==0:
                #nuevo_grafo[k,a]=grafo[i,j]*grafo[i-1,j]
                #nuevo_grafo[k,d]=grafo[i,j]*grafo[i,j+1]
                #nuevo_grafo[k,b]=grafo[i,j]*grafo[i+1,j]
                if (grafo[i,j]*grafo[i-1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i-1,j),weight=grafo[i,j]*grafo[i-1])

                if (grafo[i,j]*grafo[i+1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i+1,j),weight=grafo[i,j]*grafo[i+1,j])

                if (grafo[i,j]*grafo[i,j+1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j+1),weight=grafo[i,j]*grafo[i,j+1])
                
            elif i>0 and i!=c-1 and j>0 and j!=l-1:
                #nuevo_grafo[k,a]=grafo[i,j]*grafo[i-1,j]
                #nuevo_grafo[k,d]=grafo[i,j]*grafo[i,j+1]
                #nuevo_grafo[k,b]=grafo[i,j]*grafo[i+1,j]
                #nuevo_grafo[k,iz]=grafo[i,j]*grafo[i,j-1]
                if (grafo[i,j]*grafo[i,j+1])==1 :
                    nuevo_grafo.add_edge((i,j),(i,j+1),weight=grafo[i,j]*grafo[i,j+1])

                if (grafo[i,j]*grafo[i+1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i+1,j),weight=grafo[i,j]*grafo[i+1,j])

                if (grafo[i,j]*grafo[i,j-1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j-1),weight=grafo[i,j]*grafo[i,j-1])

                if (grafo[i,j]*grafo[i-1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i-1,j),weight=grafo[i,j]*grafo[i-1,j])

            elif i>0 and i!=c-1 and j==l-1:
                #nuevo_grafo[k,a]=grafo[i,j]*grafo[i-1,j]
                #nuevo_grafo[k,iz]=grafo[i,j]*grafo[i,j-1]
                #nuevo_grafo[k,b]=grafo[i,j]*grafo[i+1,j]
                if (grafo[i,j]*grafo[i,j-1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j-1),weight=grafo[i,j]*grafo[i,j-1])

                if (grafo[i,j]*grafo[i-1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i-1,j),weight=grafo[i,j]*grafo[i-1,j])

                if (grafo[i,j]*grafo[i+1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i+1,j),weight=grafo[i,j]*grafo[i+1,j])
                
            elif i==c-1 and j==0:
                #nuevo_grafo[k,d]=grafo[i,j]*grafo[i,j+1]
                #nuevo_grafo[k,a]=grafo[i,j]*grafo[i-1,j]
                if (grafo[i,j]*grafo[i,j+1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j+1),weight=grafo[i,j]*grafo[i,j+1])

                if (grafo[i,j]*grafo[i-1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i-1,j),weight=grafo[i,j]*grafo[i-1,j])

            elif i==c-1 and j>0 and j!=l-1:
                #nuevo_grafo[k,a]=grafo[i,j]*grafo[i-1,j]
                #nuevo_grafo[k,d]=grafo[i,j]*grafo[i,j+1]
                #nuevo_grafo[k,iz]=grafo[i,j]*grafo[i,j-1]
                if (grafo[i,j]*grafo[i,j-1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j-1),weight=grafo[i,j]*grafo[i,j-1])

                if (grafo[i,j]*grafo[i-1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i-1,j),weight=grafo[i,j]*grafo[i-1,j])

                if (grafo[i,j]*grafo[i,j+1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j+1),weight=grafo[i,j]*grafo[i,j+1])

            elif i==c-1 and j==l-1:
                #nuevo_grafo[k,iz]=grafo[i,j]*grafo[i,j-1]
                #nuevo_grafo[k,a]=grafo[i,j]*grafo[i-1,j]
                if (grafo[i,j]*grafo[i,j-1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j-1),weight=grafo[i,j]*grafo[i,j-1])

                if (grafo[i,j]*grafo[i-1,j])==1 :
                    nuevo_grafo.add_edge((i,j),(i-1,j),weight=grafo[i,j]*grafo[i-1,j])
                

    return nuevo_grafo


                
def Busqueda_Objetivos(grafo):

    c, l=np.shape(grafo)
    nodos_objetivo=[]
    
    for j in range(l):
        for i in range(c): 
            
            if i==0 and j==0:
               
               if (grafo[i,j]+grafo[i,j+1])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j+1))
                

               if (grafo[i,j]+grafo[i+1,j])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i+1,j))

            elif i==0 and j>0 and j!=l-1:
               

                if (grafo[i,j]+grafo[i,j+1])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j+1))

                if (grafo[i,j]+grafo[i+1,j])==1:

                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i+1,j))

                if (grafo[i,j]+grafo[i,j-1])==1:

                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

            elif i==0 and j==l-1:
                
                if (grafo[i,j]+grafo[i,j-1])==1:

                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

                if (grafo[i,j]+grafo[i+1,j])==1:

                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i+1,j))

            elif i>0 and i !=c-1 and j==0:
                
                if (grafo[i,j]+grafo[i-1,j])==1:

                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))

                    else:
                        nodos_objetivo.append((i-1,j))

                if (grafo[i,j]+grafo[i+1,j])==1:

                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i+1,j))

                if (grafo[i,j]+grafo[i,j+1])==1:

                   if grafo[i,j]==1:

                        nodos_objetivo.append((i,j))
                   else:
                        nodos_objetivo.append((i,j+1))
                
            elif i>0 and i!=c-1 and j>0 and j!=l-1:
                
                if (grafo[i,j]+grafo[i,j+1])==1:

                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j+1))

                if (grafo[i,j]+grafo[i+1,j])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i+1,j))

                if (grafo[i,j]+grafo[i,j-1])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

                if (grafo[i,j]+grafo[i-1,j])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i-1,j))

            elif i>0 and i!=c-1 and j==l-1:
                
                if (grafo[i,j]+grafo[i,j-1])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

                if (grafo[i,j]+grafo[i-1,j])==1:
                   
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i-1,j))

                if (grafo[i,j]+grafo[i+1,j])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i+1,j))
                
            elif i==c-1 and j==0:
                
                if (grafo[i,j]+grafo[i,j+1])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j+1))

                if (grafo[i,j]+grafo[i-1,j])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i-1,j))

            elif i==c-1 and j>0 and j!=l-1:
                
                if (grafo[i,j]+grafo[i,j-1])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

                if (grafo[i,j]+grafo[i-1,j])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i-1,j))

                if (grafo[i,j]+grafo[i,j+1])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j+1))

            elif i==c-1 and j==l-1:
               
                if (grafo[i,j]+grafo[i,j-1])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

                if (grafo[i,j]+grafo[i-1,j])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i-1,j))
                

    return nodos_objetivo

def Filtrado_de_objetivos(self):
    abscisas=[]
    ordenadas=[]
    for (x,y) in self.objetivos:
        abscisas.append(x)
        ordenadas.append(y)
    
    
    objetivos_izq=[]
    objetivos_arr=[]
    objetivos_aba=[]
    objetivos_der=[]
    puntos=[]
    #print(sorted(ordenadas))
    a=np.where(abscisas==np.min(abscisas))[0]
    c=np.where(ordenadas==np.min(ordenadas))[0]
    
    
    #(X_min,Y_max) Izquierda
    for i in a:
        objetivos_izq.append(ordenadas[i])
    b=np.where(objetivos_izq==np.max(objetivos_izq))[0]
    objetivos_izq=[(abscisas[a[0]],objetivos_izq[b[0]])]
    #(X_min,Y_min) Arriba
    for i in a:
        objetivos_arr.append(ordenadas[i])
    b=np.where(objetivos_arr==np.min(objetivos_arr))[0]
    objetivos_arr=[(abscisas[a[0]],objetivos_arr[b[0]])]
    
    #(X_min,Y_max) Abajo
    for i in a:
        objetivos_aba.append(ordenadas[i])
    b=np.where(objetivos_aba==np.max(objetivos_aba))[0]
    objetivos_aba=[(abscisas[a[0]],objetivos_aba[b[0]])]
    
    #(X_max,Y_min) Derecha
    for i in c:
        objetivos_der.append(abscisas[i])
    b=np.where(objetivos_der==np.max(objetivos_der))[0]
    objetivos_der=[(objetivos_der[b[0]],ordenadas[c[0]])]
        
    puntos=[]
    #Izquierda
    if objetivos_izq[0][0]<self.min_x_anterior or objetivos_izq[0][1]<=self.max_y_anterior:
        
        self.min_x_anterior=objetivos_izq[0][0]
        self.max_y_anterior=objetivos_arr[0][1]
        
        if self.mapa[objetivos_izq[0][0]+12,objetivos_izq[0][1]-8]==1 and self.mapa[objetivos_izq[0][0]-8,objetivos_izq[0][1]-8]==0 : 
            
            
            puntos.append((objetivos_izq[0][0]+14,objetivos_izq[0][1]-14))
    
    #Abajo    
    elif objetivos_aba[0][1]>self.max_y_anterior:
        self.max_y_anterior=objetivos_aba[0][1]
        
        if self.mapa[objetivos_aba[0][0]+8,objetivos_aba[0][1]+8]==0 and self.mapa[objetivos_aba[0][0]+8,objetivos_aba[0][1]-12]==1:
            puntos.append((objetivos_aba[0][0]+14,objetivos_aba[0][1]-14))
    
    #Arriba
    elif objetivos_arr[0][1]<self.min_y_anterior:
        #Si entra en el caso de Y _min  
        self.min_y_anterior=objetivos_arr[0][1]
        
        if self.mapa[objetivos_arr[0][0]+8,objetivos_arr[0][1]+12]==1 and self.mapa[objetivos_arr[0][0]+8,objetivos_arr[0][1]-8]==0 : 
            puntos.append((objetivos_arr[0][0]+14,objetivos_arr[0][1]+14))
       
            
    #Derecha 
    elif objetivos_der[0][0]>self.max_x_anterior or objetivos_der[0][1]<=self.min_y_anterior:
        #Si entra en el caso de Y _min  
        self.max_x_anterior=objetivos_der[0][1]
        self.min_y_anterior=objetivos_arr[0][1]
        
        if self.mapa[objetivos_der[0][0]-12,objetivos_der[0][1]+8]==1 and self.mapa[objetivos_der[0][0]+8,objetivos_der[0][1]+8]==0 : 
            
            
            puntos.append((objetivos_der[0][0]-14,objetivos_der[0][1]+14))
        
       
        
    
    
    return puntos
    