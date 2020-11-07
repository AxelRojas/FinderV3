#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

rospy.init_node('Prueba_figura')
pub = rospy.Publisher('/visualization_marker',Marker, queue_size=1)

while not rospy.is_shutdown():
    
    puntos=Marker()
    puntos.header.frame_id="/map"
    puntos.header.stamp=rospy.Time()
    puntos.ns="puntos y lineas"
    puntos.id= 0;
    puntos.action=Marker.ADD
    puntos.pose.orientation.w=1.0
    puntos.type = Marker.POINTS
    #Puntos
    puntos.scale.x=0.2
    puntos.scale.y=0.2
    
   
    #Los puntos seran rojos
    puntos.color.r=1.0
    puntos.color.a=1.0
    
    for i in range(100):
        y=i
        z=0
        p=Point()
        p.x=0
        p.y=y
        p.z=z
        puntos.points.append(p)
        
    
    
    pub.publish(puntos)