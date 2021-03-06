#!/usr/bin/env python3

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64
import sys, select, os
from time import sleep
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios


#--------------------La funcion angs nos permite devolver un mensaje con el valor de posicion que tiene el flipper seleccionado--------------------
def angs(target_linear_vel):
    return "currently:\tvalue Flippers %s\t " % (target_linear_vel)

#--------------------La funcion velocidad nos permite devolcer el valor lineal y angular que tiene la base--------------------
def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

#--------------------La funcion makeSimpleProfile nos permite mantentener un control entre los avances y limites permitidos de velocidades o posiciones ya sea de los flippers o la base-------------------------
def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

#---------------------La funcion constrain evita que evadamos el valor maximo o minimo configurado para los valores de velocidad y posicion------------------------
def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

#--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la velocidad, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros --------------------
def checkLinearLimitVelocity(vel):
    
    vel = constrain(vel, -FINDER_MAX_LIN_VEL, FINDER_MAX_LIN_VEL)
    

    return vel

#--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros --------------------
def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -FINDER_MAX_ANG_VEL, FINDER_MAX_ANG_VEL)

    return vel

#--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------
def checkAngularLimitFlipper(vel):
    vel = constrain(vel, -FLIPPER_MAX_VALUE, FLIPPER_MAX_VALUE)

    return vel
#--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------
def checkAngularLimitFlipper5(vel):
    vel = constrain(vel, 0, SHOULDER_MAX_VALUE)

    return vel

#--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------    
def checkAngularLimitFlipper6(vel):
    vel = constrain(vel, ELBOW_MIN_VALUE, 0)

    return vel
#--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------
def checkAngularLimitFlipper8(vel):
    vel = constrain(vel, -PITCH_ROTATION_MAX_VALUE, PITCH_ROTATION_MAX_VALUE)

    return vel

#--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------
def checkAngularLimitFlipper10(vel):
    vel = constrain(vel, 0.4, GRIPPER_ROTATION_MAX_VALUE)

    return vel

def main():
    #--------------------Mensaje donde se describen os comandos de movimiento--------------------
    msg = """
    Control Your FINDER and Your Flippers!
    ---------------------------

    Moving around:   
        w
    a   s   d
        x   

    Right_front_flipper:   
        o 

        l 

    Left_front_flipper:   
        r 

        f

    Right_back_flipper:   
        i

        k

    Left_back_flipper:   
        t 

        g

    Arm_base_rotation:   
         
    1       3
     
    Arm_Shoulder_rotation:   
        2 

        0 

    Arm_Elbow_rotation:   
        8 

        5   

    roll rotation:   
         
    4       6

    Pitch_rotation:   
        -

        +   

    Roll rotation2:   
         
    7       9

    Gripper rotation:   
        u     

        j


    w/x : increase/decrease linear velocity 
    a/d : increase/decrease angular velocity
    r/f, o/l, i/k, t/g : up / down
    1/3 :rotate arm base sentido de giro <-- / -->
    2/0, 5/8, -/+ :rotation shoulder sentido de giro ^ /sentido de giro v
    4/6, 7/9 :rotation roll sentido de giro <-- / -->
    u/j :gripper rotation sentido de giro ^ /sentido de giro v

    s : force stop
    space key: to see de comands

    CTRL-C to quit
    """

    e = """
    Communications Failed
    """
    
    
    #--------------------Variable que controla la alerta del mensaje--------------------
    status=0
    status0=0
    status1=0
    status2=0
    status3=0
    status4=0
    status5=0
    status6=0
    status7=0
    status8=0
    status9=0
    status10=0
    #--------------------Variables de desplazamiento de la base--------------------
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    #--------------------Variables para controlar las juntas--------------------
    control_angular_flipper0 = 0.0
    control_angular_flipper1 = 0.0
    control_angular_flipper2 = 0.0
    control_angular_flipper3 = 0.0
    control_angular_flipper4 = 0.0
    control_angular_flipper5= 0.0
    control_angular_flipper6= 0.0
    control_angular_flipper7= 0.0
    control_angular_flipper8= 0.0
    control_angular_flipper9= 1.6
    control_angular_flipper10= 1.6
    #--------------------Varaibles para definir los pasos--------------------
    LIN_VEL_STEP_SIZE=0.1
    ANG_VEL_STEP_SIZE=0.2
    ANG_FLIPPER_STEP_SIZE=0.1
    #--------------------Variables para guardar las velocidades de las juntas--------------------
    target_angular_flipper0=0.0
    target_angular_flipper1 =0.0
    target_angular_flipper2=0.0
    target_angular_flipper3=0.0
    target_angular_flipper4=0.0
    target_angular_flipper5=0.0
    target_angular_flipper6=0.0
    target_angular_flipper7=0.0
    target_angular_flipper8=0.0
    target_angular_flipper9=1.6
    target_angular_flipper10=1.6
    pos=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.6,1.6]

    rospy.init_node('finder_teleop_key')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub1=rospy.Publisher('/flipper1_out', Float64, queue_size=1)
    pub2=rospy.Publisher('/flipper2_out', Float64, queue_size=1)
    pub3=rospy.Publisher('/flipper3_out', Float64, queue_size=1)
    pub4=rospy.Publisher('/flipper4_out', Float64, queue_size=1)
    pub5=rospy.Publisher('/base_rotation_out',Float32,queue_size=1)
    pub6=rospy.Publisher('/shoulder_rotation_out',Float32,queue_size=1)
    pub7=rospy.Publisher('/elbow_rotation_out',Float32,queue_size=1)
    pub8=rospy.Publisher('/roll_rotation_out',Float32,queue_size=1)
    pub9=rospy.Publisher('/pitch_rotation_out',Float32,queue_size=1)
    pub10=rospy.Publisher('/roll_rotation_2_out',Float32,queue_size=1)
    pub11=rospy.Publisher('/gripper_rotation_out',Float32,queue_size=1)

    #--------------------La funcio getkey nos permite obtener la letra presionada despues de hacer una comprobacion del sistema--------------------

    def getKey():
        if os.name == 'nt':
          return msvcrt.getch()

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    
    def callback(data):

        nonlocal target_angular_flipper10
        nonlocal target_angular_flipper9
        nonlocal target_angular_flipper8
        nonlocal target_angular_flipper7
        nonlocal target_angular_flipper6  
        nonlocal target_angular_flipper5
        nonlocal target_angular_flipper4
        nonlocal target_angular_flipper3
        nonlocal target_angular_flipper2
        nonlocal target_angular_flipper1
        nonlocal target_angular_flipper0
        nonlocal control_angular_flipper10
        nonlocal control_angular_flipper9
        nonlocal control_angular_flipper8
        nonlocal control_angular_flipper7
        nonlocal control_angular_flipper6
        nonlocal control_angular_flipper5
        nonlocal control_angular_flipper4
        nonlocal control_angular_flipper3
        nonlocal control_angular_flipper2
        nonlocal control_angular_flipper1
        nonlocal control_angular_flipper0
        nonlocal pos
        pos=data.position
        pos=list(pos)
        target_angular_flipper0=pos[0]
        target_angular_flipper1=pos[1]
        target_angular_flipper2=pos[2]
        target_angular_flipper3=pos[3]
        target_angular_flipper4=pos[4]
        target_angular_flipper5=pos[5]
        target_angular_flipper6=pos[6]
        target_angular_flipper7=pos[7]
        target_angular_flipper8=pos[8]
        target_angular_flipper9=pos[9]
        target_angular_flipper10=pos[10]
        control_angular_flipper0=pos[0]
        control_angular_flipper1=pos[1]
        control_angular_flipper2=pos[2]
        control_angular_flipper3=pos[3]
        control_angular_flipper4=pos[4]
        control_angular_flipper5=pos[5]
        control_angular_flipper6=pos[6]
        control_angular_flipper7=pos[7]
        control_angular_flipper8=pos[8]
        control_angular_flipper9=pos[9]
        control_angular_flipper10=pos[10]

    #--------------------Aqui es donde se declaran todas las operaciones al presionar una de las teclas dentro del menu --------------------
    def funcion():
        try:
            nonlocal target_angular_flipper10
            nonlocal target_angular_flipper9
            nonlocal target_angular_flipper8
            nonlocal target_angular_flipper7
            nonlocal target_angular_flipper6  
            nonlocal target_angular_flipper5
            nonlocal target_angular_flipper4
            nonlocal target_angular_flipper3
            nonlocal target_angular_flipper2
            nonlocal target_angular_flipper1
            nonlocal target_angular_flipper0
            global target_angular_vel
            global target_linear_vel
            nonlocal control_angular_vel
            nonlocal control_linear_vel
            nonlocal control_angular_flipper10
            nonlocal control_angular_flipper9
            nonlocal control_angular_flipper8
            nonlocal control_angular_flipper7
            nonlocal control_angular_flipper6
            nonlocal control_angular_flipper5
            nonlocal control_angular_flipper4
            nonlocal control_angular_flipper3
            nonlocal control_angular_flipper2
            nonlocal control_angular_flipper1
            nonlocal control_angular_flipper0
            nonlocal status, status0, status1, status2, status3, status4, status5, status6, status7, status8, status9, status10
            nonlocal msg, pub, pub1, pub2, pub3, pub4, pub5, pub6, pub7, pub8, pub9, pub10, pub11, pos
            global   FLIPPER_MAX_VALUE, FINDER_MAX_ANG_VEL, FINDER_MAX_LIN_VEL ,ANG_FLIPPER_STEP_SIZE,SHOULDER_MAX_VALUE, ELBOW_MIN_VALUE, PITCH_ROTATION_MAX_VALUE, GRIPPER_ROTATION_MAX_VALUE, LIN_VEL_STEP_SIZE, ANG_VEL_STEP_SIZE, ANG_FLIPPER_STEP_SIZE
            print(msg)
            while(1):


                key = getKey()
                if key == 'w' :
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                elif key == 'x' :
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                elif key == 'a' :
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                elif key == 'd' :
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                #--------------------Right front flipper--------------------
                elif key == 'o' :
                    target_angular_flipper0= checkAngularLimitFlipper(target_angular_flipper0 + ANG_FLIPPER_STEP_SIZE)
                    status0 = status0 + 1
                    print("Right_front_flipper "+angs(target_angular_flipper0))
                    control_angular_flipper0 = makeSimpleProfile(control_angular_flipper0, target_angular_flipper0, (ANG_FLIPPER_STEP_SIZE))
                    pos[0]=control_angular_flipper0
                    
                elif key == 'l' :
                    target_angular_flipper0= checkAngularLimitFlipper(target_angular_flipper0 - ANG_FLIPPER_STEP_SIZE)
                    status0 = status0 + 1
                    print("Right_front_flipper "+angs(target_angular_flipper0))
                    control_angular_flipper0 = makeSimpleProfile(control_angular_flipper0, target_angular_flipper0, (ANG_FLIPPER_STEP_SIZE))
                    pos[0]=control_angular_flipper0

                #--------------------Left front flipper--------------------
                elif key == 'r' :
                    target_angular_flipper1= checkAngularLimitFlipper(target_angular_flipper1 + ANG_FLIPPER_STEP_SIZE)
                    status1 = status1 + 1
                    print("Left_front_flipper "+angs(target_angular_flipper1))
                    control_angular_flipper1 = makeSimpleProfile(control_angular_flipper1, target_angular_flipper1, (ANG_FLIPPER_STEP_SIZE))
                    pos[1]=control_angular_flipper1

                elif key == 'f' :
                    target_angular_flipper1= checkAngularLimitFlipper(target_angular_flipper1 - ANG_FLIPPER_STEP_SIZE)
                    status1 = status1 + 1
                    print("Left_front_flipper "+angs(target_angular_flipper1))
                    control_angular_flipper1 = makeSimpleProfile(control_angular_flipper1, target_angular_flipper1, (ANG_FLIPPER_STEP_SIZE))
                    pos[1]=control_angular_flipper1

                #--------------------Right back flipper--------------------
                elif key == 'i' :
                    target_angular_flipper2= checkAngularLimitFlipper(target_angular_flipper2 + ANG_FLIPPER_STEP_SIZE)
                    status2 = status2 + 1
                    print("Right_back_flipper "+angs(target_angular_flipper2))
                    control_angular_flipper2 = makeSimpleProfile(control_angular_flipper2, target_angular_flipper2, (ANG_FLIPPER_STEP_SIZE))
                    pos[2]=control_angular_flipper2
                elif key == 'k' :
                    target_angular_flipper2= checkAngularLimitFlipper(target_angular_flipper2 - ANG_FLIPPER_STEP_SIZE)
                    status2 = status2 + 1
                    print("Right_back_flipper "+angs(target_angular_flipper2))
                    control_angular_flipper2 = makeSimpleProfile(control_angular_flipper2, target_angular_flipper2, (ANG_FLIPPER_STEP_SIZE))
                    pos[2]=control_angular_flipper2
                #--------------------Left back flipper--------------------
                elif key == 't' :
                    target_angular_flipper3= checkAngularLimitFlipper(target_angular_flipper3 + ANG_FLIPPER_STEP_SIZE)
                    status3 = status3 + 1
                    print("Left_back_flipper "+angs(target_angular_flipper3))
                    control_angular_flipper3 = makeSimpleProfile(control_angular_flipper3, target_angular_flipper3, (ANG_FLIPPER_STEP_SIZE))
                    pos[3]=control_angular_flipper3

                elif key == 'g' :
                    target_angular_flipper3= checkAngularLimitFlipper(target_angular_flipper3 - ANG_FLIPPER_STEP_SIZE)
                    status3 = status3 + 1
                    print("Left_back_flipper "+angs(target_angular_flipper3))
                    control_angular_flipper3 = makeSimpleProfile(control_angular_flipper3, target_angular_flipper3, (ANG_FLIPPER_STEP_SIZE))
                    pos[3]=control_angular_flipper3
                #--------------------Arm base rotation--------------------
                elif key == '1' :
                    target_angular_flipper4= checkAngularLimitFlipper(target_angular_flipper4 + ANG_FLIPPER_STEP_SIZE)
                    status4 = status4 + 1
                    print("Arm_base_rotation "+angs(target_angular_flipper4))
                    control_angular_flipper4 = makeSimpleProfile(control_angular_flipper4, target_angular_flipper4, (ANG_FLIPPER_STEP_SIZE))
                    pos[4]=control_angular_flipper4

                elif key == '3' :
                    target_angular_flipper4= checkAngularLimitFlipper(target_angular_flipper4 - ANG_FLIPPER_STEP_SIZE)
                    status4 = status4 + 1
                    print("Arm_base_rotation "+angs(target_angular_flipper4))
                    control_angular_flipper4 = makeSimpleProfile(control_angular_flipper4, target_angular_flipper4, (ANG_FLIPPER_STEP_SIZE))
                    pos[4]=control_angular_flipper4
                #--------------------Arm Shoulder rotation--------------------
                elif key == '2' :
                    target_angular_flipper5= checkAngularLimitFlipper5(target_angular_flipper5 + ANG_FLIPPER_STEP_SIZE)
                    status5 = status5 + 1
                    print("Arm_Shoulder_rotation "+angs(target_angular_flipper5))
                    control_angular_flipper5 = makeSimpleProfile(control_angular_flipper5, target_angular_flipper5, (ANG_FLIPPER_STEP_SIZE))
                    pos[5]=control_angular_flipper5

                elif key == '0' :
                    target_angular_flipper5= checkAngularLimitFlipper5(target_angular_flipper5 - ANG_FLIPPER_STEP_SIZE)
                    status5 = status5 + 1
                    print("Arm_Shoulder_rotation "+angs(target_angular_flipper5))
                    control_angular_flipper5 = makeSimpleProfile(control_angular_flipper5, target_angular_flipper5, (ANG_FLIPPER_STEP_SIZE))
                    pos[5]=control_angular_flipper5
                #--------------------Arm Elbow rotation--------------------
                elif key == '8' :
                    target_angular_flipper6= checkAngularLimitFlipper6(target_angular_flipper6 - ANG_FLIPPER_STEP_SIZE)
                    status6 = status6 + 1
                    print("Arm_Elbow_rotation "+angs(target_angular_flipper6))
                    control_angular_flipper6 = makeSimpleProfile(control_angular_flipper6, target_angular_flipper6, (ANG_FLIPPER_STEP_SIZE))
                    pos[6]=control_angular_flipper6
                elif key == '5' :
                    target_angular_flipper6= checkAngularLimitFlipper6(target_angular_flipper6 + ANG_FLIPPER_STEP_SIZE)
                    status6 = status6 + 1
                    print("Arm_Elbow_rotation "+angs(target_angular_flipper6))
                    control_angular_flipper6 = makeSimpleProfile(control_angular_flipper6, target_angular_flipper6, (ANG_FLIPPER_STEP_SIZE))
                    pos[6]=control_angular_flipper6
                #--------------------Roll rotation--------------------
                elif key == '6' :
                    target_angular_flipper7= checkAngularLimitFlipper(target_angular_flipper7 - ANG_FLIPPER_STEP_SIZE)
                    status7 = status7 + 1
                    print("Roll_rotation "+angs(target_angular_flipper7))
                    control_angular_flipper7 = makeSimpleProfile(control_angular_flipper7, target_angular_flipper7, (ANG_FLIPPER_STEP_SIZE))
                    pos[7]=control_angular_flipper7
                elif key == '4' :
                    target_angular_flipper7= checkAngularLimitFlipper(target_angular_flipper7 + ANG_FLIPPER_STEP_SIZE)
                    status7 = status7 + 1
                    print("Roll_rotation "+angs(target_angular_flipper7))
                    control_angular_flipper7 = makeSimpleProfile(control_angular_flipper7, target_angular_flipper7, (ANG_FLIPPER_STEP_SIZE))
                    pos[7]=control_angular_flipper7
                #--------------------Pitch rotation--------------------
                elif key == '-' :
                    target_angular_flipper8= checkAngularLimitFlipper8(target_angular_flipper8 + ANG_FLIPPER_STEP_SIZE)
                    status8 = status8 + 1
                    print("Pitch_rotation "+angs(target_angular_flipper8))
                    control_angular_flipper8 = makeSimpleProfile(control_angular_flipper8, target_angular_flipper8, (ANG_FLIPPER_STEP_SIZE))
                    pos[8]=control_angular_flipper8
                elif key == '+' :
                    target_angular_flipper8= checkAngularLimitFlipper8(target_angular_flipper8 - ANG_FLIPPER_STEP_SIZE)
                    status8 = status8 + 1
                    print("Pitch_rotation "+angs(target_angular_flipper8))
                    control_angular_flipper8 = makeSimpleProfile(control_angular_flipper8, target_angular_flipper8, (ANG_FLIPPER_STEP_SIZE))
                    pos[8]=control_angular_flipper8
                #--------------------Roll rotation2--------------------
                elif key == '9' :
                    target_angular_flipper9= checkAngularLimitFlipper(target_angular_flipper9 - ANG_FLIPPER_STEP_SIZE)
                    status9 = status9 + 1
                    print("Roll_rotation2 "+angs(target_angular_flipper9))
                    control_angular_flipper9 = makeSimpleProfile(control_angular_flipper9, target_angular_flipper9, (ANG_FLIPPER_STEP_SIZE))
                    pos[9]=control_angular_flipper9
                elif key == '7' :
                    target_angular_flipper9= checkAngularLimitFlipper(target_angular_flipper9 + ANG_FLIPPER_STEP_SIZE)
                    status9 = status9 + 1
                    print("Roll_rotation2 "+angs(target_angular_flipper9))
                    control_angular_flipper9 = makeSimpleProfile(control_angular_flipper9, target_angular_flipper9, (ANG_FLIPPER_STEP_SIZE))
                    pos[9]=control_angular_flipper9
                #--------------------Gripper_rotation--------------------
                elif key == 'u' :
                    target_angular_flipper10= checkAngularLimitFlipper10(target_angular_flipper10 - ANG_FLIPPER_STEP_SIZE)
                    status10 = status10 + 1
                    print("Gripper_rotation "+angs(target_angular_flipper10))
                    control_angular_flipper10 = makeSimpleProfile(control_angular_flipper10, target_angular_flipper10, (ANG_FLIPPER_STEP_SIZE))
                    pos[10]=control_angular_flipper10
                elif key == 'j' :
                    target_angular_flipper10= checkAngularLimitFlipper10(target_angular_flipper10 + ANG_FLIPPER_STEP_SIZE)
                    status10 = status10 + 1
                    print("Gripper_rotation "+angs(target_angular_flipper10))
                    control_angular_flipper10 = makeSimpleProfile(control_angular_flipper10, target_angular_flipper10, (ANG_FLIPPER_STEP_SIZE))
                    pos[10]=control_angular_flipper10
                #--------------------Stop all movements --------------------   
                elif key == 's' :
                    target_linear_vel   = 0.0
                    control_linear_vel  = 0.0
                    target_angular_vel  = 0.0
                    control_angular_vel = 0.0
                    print(vels(target_linear_vel, target_angular_vel))
                    print(msg)
                #--------------------Show commands--------------------    
                elif key == ' ':

                    print(msg)

                else:
                    if (key == '\x03'):
                        break
                
                #--------------------Se lleva la cuenta de las veces que se presionan las teclas y cuando debe de ser mandado el mensaje de comandos--------------------
                if status == 5 :
                    print(msg)
                    status = 0
                elif status0==31:
                    print(msg)
                    status0 = 0
                elif status1==31:
                    print(msg)
                    status1 = 0
                elif status2==31:
                    print(msg)
                    status2 = 0
                elif status3==31:
                    print(msg)
                    status3 = 0
                elif status4==31:
                    print(msg)
                    status4 = 0
                elif status5==9:
                    print(msg)
                    status5 = 0
                elif status6==23:
                    print(msg)
                    status6 = 0
                elif status7==31:
                    print(msg)
                    status7 = 0
                elif status8==10:
                    print(msg)
                    status8 = 0
                elif status9==31:
                    print(msg)
                    status9 = 0
                elif status10==16:
                    print(msg)
                    status10 = 0

                #--------------------Se asignan los valores lineales y angulares de movimiento de la base--------------------    
                twist = Twist()

                control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
                control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                pub.publish(twist)     
                pub1.publish(pos[0])
                pub2.publish(pos[1])
                pub3.publish(pos[2])
                pub4.publish(pos[3])
                pub5.publish(pos[4])
                pub6.publish(pos[5])
                pub7.publish(pos[6])
                pub8.publish(pos[7])
                pub9.publish(pos[8])
                pub10.publish(pos[9])
                pub11.publish(pos[10])
               

        #--------------------Manejo de excepciones--------------------
        #except:
        #    print(e)
        
        #--------------------Se indica que el robot deje de moverse hasta que reciba otro comando de velocidad--------------------
        finally:
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    rospy.Subscriber("/joint_states",JointState,callback)
    sleep(1)
    funcion()
    

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    #--------------------Variables para definir el tope de comandos o valores--------------------
    LIN_VEL_STEP_SIZE=0.3
    ANG_VEL_STEP_SIZE=0.2
    ANG_FLIPPER_STEP_SIZE=0.1
    FINDER_MAX_LIN_VEL = 0.3
    FINDER_MAX_ANG_VEL = 0.2
    FLIPPER_MAX_VALUE= 3.14
    SHOULDER_MAX_VALUE=0.9
    ELBOW_MIN_VALUE=-2.3
    ROLL_ROTATION_MAX_VALUE= 3.14
    PITCH_ROTATION_MAX_VALUE=1.5
    ROLL_ROTATION2_MAX_VALUE = 3.14
    GRIPPER_ROTATION_MAX_VALUE=1.62
    target_linear_vel= 0.0
    target_angular_vel= 0.0
    main()
