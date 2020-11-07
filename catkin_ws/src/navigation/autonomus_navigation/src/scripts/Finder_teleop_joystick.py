#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import Joy
from time import sleep


class Nodo():
    #--------------------La funcion angs nos permite devolver un mensaje con el valor de posicion que tiene el flipper seleccionado--------------------
    def angs(self,input2):
        self.input2=input2
        return "currently:\tvalue Flippers %s\t " % (self.input2)

    #--------------------La funcion velocidad nos permite devolcer el valor lineal y angular que tiene la base--------------------
    def vels(self):
        return "currently:\tlinear vel %s\t angular vel %s " % (self.target_linear_vel,self.target_angular_vel)

    #--------------------La funcion makeSimpleProfile nos permite mantentener un control entre los avances y limites permitidos de velocidades o posiciones ya sea de los flippers o la base-------------------------
    def makeSimpleProfile(self,output,input,slop):
        self.output=output
        self.input=input
        self.slop=slop
        if self.input > self.output:
            self.output = min( self.input, self.output + self.slop )
        elif input < self.output:
            self.output = max( self.input, self.output - self.slop )
        else:
            self.output = self.input

        return self.output

    #---------------------La funcion constrain evita que evadamos el valor maximo o minimo configurado para los valores de velocidad y posicion------------------------
    def constrain(self,input,low,high):
        self.input=input
        self.low=low
        self.high=high
        if self.input < self.low:
          self.input = self.low
        elif self.input > self.high:
          self.input = self.high
        else:
          self.input = self.input

        return self.input

    #--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la velocidad, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros --------------------
    def checkLinearLimitVelocity(self,vel):
        self.vel=vel
        self.vel = self.constrain(self.vel, -self.FINDER_MAX_LIN_VEL, self.FINDER_MAX_LIN_VEL)
        

        return self.vel

    #--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros --------------------
    def checkAngularLimitVelocity(self,vel):
        self.vel=vel
        self.vel = self.constrain(self.vel, -self.FINDER_MAX_ANG_VEL, self.FINDER_MAX_ANG_VEL)

        return self.vel

    #--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------
    def checkAngularLimitFlipper(self,vel):
        self.vel=vel
        self.vel = self.constrain(self.vel, -self.FLIPPER_MAX_VALUE, self.FLIPPER_MAX_VALUE)

        return self.vel
    #--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------
    def checkAngularLimitFlipper5(self,vel):
        self.vel=vel
        self.vel = self.constrain(self.vel, 0, self.SHOULDER_MAX_VALUE)

        return self.vel

    #--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------    
    def checkAngularLimitFlipper6(self,vel):
        self.vel=vel
        self.vel = self.constrain(self.vel, self.ELBOW_MIN_VALUE, 0)

        return self.vel

    #--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------    
    def checkAngularLimitFlipper7(self,vel):
        self.vel=vel
        self.vel = self.constrain(self.vel, -self.ROLL_ROTATION_MAX_VALUE, self.ROLL_ROTATION_MAX_VALUE)

        return self.vel

    #--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------
    def checkAngularLimitFlipper8(self,vel):
        self.vel=vel
        self.vel = self.constrain(self.vel, -self.PITCH_ROTATION_MAX_VALUE, self.PITCH_ROTATION_MAX_VALUE)
        return self.vel

    #--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------
    def checkAngularLimitFlipper9(self,vel):
        self.vel=vel
        self.vel = self.constrain(self.vel, -self.ROLL_ROTATION2_MAX_VALUE, self.ROLL_ROTATION2_MAX_VALUE)

        return self.vel

    #--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------
    def checkAngularLimitFlipper10(self,vel):
        self.vel=vel
        self.vel = self.constrain(self.vel, 0.4, self.GRIPPER_ROTATION_MAX_VALUE)
        return self.vel

    def callback(self,data):
            
        self.pos=data.position
        self.pos=list(self.pos)
        self.target_angular_flipper0=self.pos[0]
        self.target_angular_flipper1=self.pos[1]
        self.target_angular_flipper2=self.pos[2]
        self.target_angular_flipper3=self.pos[3]
        self.target_angular_flipper4=self.pos[4]
        self.target_angular_flipper5=self.pos[5]
        self.target_angular_flipper6=self.pos[6]
        self.target_angular_flipper7=self.pos[7]
        self.target_angular_flipper8=self.pos[8]
        self.target_angular_flipper9=self.pos[9]
        self.target_angular_flipper10=self.pos[10]
        self.control_angular_flipper0=self.pos[0]
        self.control_angular_flipper1=self.pos[1]
        self.control_angular_flipper2=self.pos[2]
        self.control_angular_flipper3=self.pos[3]
        self.control_angular_flipper4=self.pos[4]
        self.control_angular_flipper5=self.pos[5]
        self.control_angular_flipper6=self.pos[6]
        self.control_angular_flipper7=self.pos[7]
        self.control_angular_flipper8=self.pos[8]
        self.control_angular_flipper9=self.pos[9]
        self.control_angular_flipper10=self.pos[10]



        
    def joystick_callback(self,data):
        self.pos=list(self.pos)
        self.palancas=data.axes
        self.botones=data.buttons
        #------------------Move base of finder ------------
        if self.botones[4]==0 and self.botones[5]==0:

            if self.palancas[1]>0.4 :
                self.target_linear_vel = self.checkLinearLimitVelocity((self.target_linear_vel+self.palancas[1])*0.2) #------Palanca izquierda-----
                print(self.vels())
            if self.palancas[1]<-0.4  :
                self.target_linear_vel = self.checkLinearLimitVelocity((self.target_linear_vel+self.palancas[1])*0.2)
                print(self.vels())
                
            if self.palancas[3]>0.4 :
                self.target_angular_vel = self.checkAngularLimitVelocity((self.target_angular_vel+self.palancas[3])*0.2) #----- Palanca Derecha-----
                print(self.vels())
                
            if self.palancas[3]<-0.4 :
                self.target_angular_vel = self.checkAngularLimitVelocity((self.target_angular_vel+self.palancas[3])*0.2)
                print(self.vels())
            
            #--------------------Stop all movements --------------------   
            #--------------------Show commands--------------------    
            if self.botones[7]==1:
                print(self.vels())
                self.target_linear_vel   = 0.0
                self.control_linear_vel  = 0.0
                self.target_angular_vel  = 0.0
                self.control_angular_vel = 0.0
                print(self.vels())
                print(self.msg)

        if self.botones[4]==0 and self.botones[5]==1:

            self.target_linear_vel   = 0.0
            self.control_linear_vel  = 0.0
            self.target_angular_vel  = 0.0
            self.control_angular_vel = 0.0
            
        #--------------------Right front flipper--------------------
            if self.palancas[4]>0.4:
                self.target_angular_flipper0= self.checkAngularLimitFlipper(self.target_angular_flipper0+self.ANG_FLIPPER_STEP_SIZE)
                #status0 = status0 + 1
                self.input=self.target_angular_flipper0
                print("Right_front_flipper "+self.angs(self.target_angular_flipper0))
                self.control_angular_flipper0 = self.makeSimpleProfile(self.control_angular_flipper0, self.target_angular_flipper0, self.ANG_FLIPPER_STEP_SIZE)
                self.pos[0]=self.control_angular_flipper0
                
            if self.palancas[4]<-0.4:
                self.target_angular_flipper0= self.checkAngularLimitFlipper(self.target_angular_flipper0-self.ANG_FLIPPER_STEP_SIZE)
                #status0 = status0 + 1
                print("Right_front_flipper "+self.angs(self.target_angular_flipper0))
                self.control_angular_flipper0 = self.makeSimpleProfile(self.control_angular_flipper0, self.target_angular_flipper0, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[0]=self.control_angular_flipper0

            #--------------------Right back flipper--------------------
            if self.palancas[1]>0.4 :
                self.target_angular_flipper2= self.checkAngularLimitFlipper(self.target_angular_flipper2+self.ANG_FLIPPER_STEP_SIZE)
                #status2 = status2 + 1
                print("Right_back_flipper "+self.angs(self.target_angular_flipper2))
                self.control_angular_flipper2 = self.makeSimpleProfile(self.control_angular_flipper2, self.target_angular_flipper2, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[2]=self.control_angular_flipper2
            if self.palancas[1]<-0.4:
                self.target_angular_flipper2= self.checkAngularLimitFlipper(self.target_angular_flipper2-self.ANG_FLIPPER_STEP_SIZE)
                #status2 = status2 + 1
                print("Right_back_flipper "+self.angs(self.target_angular_flipper2))
                self.control_angular_flipper2 = self.makeSimpleProfile(self.control_angular_flipper2, self.target_angular_flipper2, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[2]=self.control_angular_flipper2

        if self.botones[4]==1 and self.botones[5]==0:
            self.target_linear_vel   = 0.0
            self.control_linear_vel  = 0.0
            self.target_angular_vel  = 0.0
            self.control_angular_vel = 0.0
           
            #--------------------Left front flipper--------------------
            if self.palancas[4]>0.4 :
                self.target_angular_flipper1= self.checkAngularLimitFlipper(self.target_angular_flipper1+self.ANG_FLIPPER_STEP_SIZE)
                #status1 = status1 + 1
                print("Left_front_flipper "+self.angs(self.target_angular_flipper1))
                self.control_angular_flipper1 = self.makeSimpleProfile(self.control_angular_flipper1, self.target_angular_flipper1, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[1]=self.control_angular_flipper1

            if self.palancas[4]<-0.4 :
                self.target_angular_flipper1= self.checkAngularLimitFlipper(self.target_angular_flipper1-self.ANG_FLIPPER_STEP_SIZE)
                #status1 = status1 + 1
                print("Left_front_flipper "+self.angs(self.target_angular_flipper1))
                self.control_angular_flipper1 = self.makeSimpleProfile(self.control_angular_flipper1, self.target_angular_flipper1, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[1]=self.control_angular_flipper1


            #--------------------Left back flipper--------------------
            if self.palancas[1]>0.4  :
                self.target_angular_flipper3= self.checkAngularLimitFlipper(self.target_angular_flipper3+self.ANG_FLIPPER_STEP_SIZE)
                #status3 = status3 + 1
                print("Left_back_flipper "+self.angs(self.target_angular_flipper3))
                self.control_angular_flipper3 = self.makeSimpleProfile(self.control_angular_flipper3, self.target_angular_flipper3, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[3]=self.control_angular_flipper3

            if self.palancas[1]<-0.4 :
                self.target_angular_flipper3= self.checkAngularLimitFlipper(self.target_angular_flipper3-self.ANG_FLIPPER_STEP_SIZE)
                #status3 = status3 + 1
                print("Left_back_flipper "+self.angs(self.target_angular_flipper3))
                self.control_angular_flipper3 = self.makeSimpleProfile(self.control_angular_flipper3, self.target_angular_flipper3, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[3]=self.control_angular_flipper3

            #-----Auxiliar arm-----
        if self.palancas[2]<0 and self.palancas[5]>=0:
            self.target_linear_vel   = 0.0
            self.control_linear_vel  = 0.0
            self.target_angular_vel  = 0.0
            self.control_angular_vel = 0.0

            #--------------------Arm base rotation--------------------
            if self.botones[1]==0 and self.botones[2]==1:
                self.target_angular_flipper4= self.checkAngularLimitFlipper(self.target_angular_flipper4 - self.ANG_FLIPPER_STEP_SIZE)
                #status4 = status4 + 1
                print("Arm_base_rotation "+self.angs(self.target_angular_flipper4))
                self.control_angular_flipper4 = self.makeSimpleProfile(self.control_angular_flipper4, self.target_angular_flipper4, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[4]=self.control_angular_flipper4
                
            if self.botones[1]==1 and self.botones[2]==0:
                self.target_angular_flipper4= self.checkAngularLimitFlipper(self.target_angular_flipper4 + self.ANG_FLIPPER_STEP_SIZE)
                #status4 = status4 + 1
                print("Arm_base_rotation "+self.angs(self.target_angular_flipper4))
                self.control_angular_flipper4 = self.makeSimpleProfile(self.control_angular_flipper4, self.target_angular_flipper4, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[4]=self.control_angular_flipper4

            #--------------------Arm Shoulder rotation--------------------
            
            if self.botones[3]==1 and self.botones[0]==0:
            
                self.target_angular_flipper5= self.checkAngularLimitFlipper5(self.target_angular_flipper5 + self.ANG_FLIPPER_STEP_SIZE)
                #status5 = status5 + 1
                print("Arm_Shoulder_rotation "+self.angs(self.target_angular_flipper5))
                self.control_angular_flipper5 = self.makeSimpleProfile(self.control_angular_flipper5, self.target_angular_flipper5, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[5]=self.control_angular_flipper5

            if self.botones[3]==0 and self.botones[0]==1:
                self.target_angular_flipper5= self.checkAngularLimitFlipper5(self.target_angular_flipper5 - self.ANG_FLIPPER_STEP_SIZE)
                #status5 = status5 + 1
                print("Arm_Shoulder_rotation "+self.angs(self.target_angular_flipper5))
                self.control_angular_flipper5 = self.makeSimpleProfile(self.control_angular_flipper5, self.target_angular_flipper5, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[5]=self.control_angular_flipper5

        if self.palancas[2]>=0 and self.palancas[5]<0:
            self.target_linear_vel   = 0.0
            self.control_linear_vel  = 0.0
            self.target_angular_vel  = 0.0
            self.control_angular_vel = 0.0
            #--------------------Arm Elbow rotation--------------------
            if self.botones[3]==1 and self.botones[0]==0:
                self.target_angular_flipper6= self.checkAngularLimitFlipper6(self.target_angular_flipper6 - self.ANG_FLIPPER_STEP_SIZE)
                #status6 = status6 + 1
                print("Arm_Elbow_rotation "+self.angs(self.target_angular_flipper6))
                self.control_angular_flipper6 = self.makeSimpleProfile(self.control_angular_flipper6, self.target_angular_flipper6, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[6]=self.control_angular_flipper6
            if self.botones[3]==0 and self.botones[0]==1:
                self.target_angular_flipper6= self.checkAngularLimitFlipper6(self.target_angular_flipper6 + self.ANG_FLIPPER_STEP_SIZE)
                #status6 = status6 + 1
                print("Arm_Elbow_rotation "+self.angs(self.target_angular_flipper6))
                self.control_angular_flipper6 = self.makeSimpleProfile(self.control_angular_flipper6, self.target_angular_flipper6, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[6]=self.control_angular_flipper6
            #--------------------Roll rotation--------------------
            if self.botones[2]==1 and self.botones[1]==0:
                self.target_angular_flipper7= self.checkAngularLimitFlipper(self.target_angular_flipper7 + self.ANG_FLIPPER_STEP_SIZE)
                #status7 = status7 + 1
                print("Roll_rotation "+self.angs(self.target_angular_flipper7))
                self.control_angular_flipper7 = self.makeSimpleProfile(self.control_angular_flipper7, self.target_angular_flipper7, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[7]=self.control_angular_flipper7

            if self.botones[2]==0 and self.botones[1]==1:
                self.target_angular_flipper7= self.checkAngularLimitFlipper(self.target_angular_flipper7 - self.ANG_FLIPPER_STEP_SIZE)
                #status7 = status7 + 1
                print("Roll_rotation "+self.angs(self.target_angular_flipper7))
                self.control_angular_flipper7 = self.makeSimpleProfile(self.control_angular_flipper7, self.target_angular_flipper7, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[7]=self.control_angular_flipper7

            #--------------------Pitch rotation--------------------
            if self.palancas[1]>0.4:
                self.target_angular_flipper8= self.checkAngularLimitFlipper8(self.target_angular_flipper8 + self.ANG_FLIPPER_STEP_SIZE)
                #status8 = status8 + 1
                print("Pitch_rotation "+self.angs(self.target_angular_flipper8))
                self.control_angular_flipper8 = self.makeSimpleProfile(self.control_angular_flipper8, self.target_angular_flipper8, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[8]=self.control_angular_flipper8

            if self.palancas[1]<-0.4:
                self.target_angular_flipper8= self.checkAngularLimitFlipper8(self.target_angular_flipper8 - self.ANG_FLIPPER_STEP_SIZE)
                #status8 = status8 + 1
                print("Pitch_rotation "+self.angs(self.target_angular_flipper8))
                self.control_angular_flipper8 = self.makeSimpleProfile(self.control_angular_flipper8, self.target_angular_flipper8, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[8]=self.control_angular_flipper8

        #--------------------Roll rotation2--------------------

            if self.palancas[3]>0.4:                
                self.target_angular_flipper9= self.checkAngularLimitFlipper(self.target_angular_flipper9 + self.ANG_FLIPPER_STEP_SIZE)
                #status9 = status9 + 1
                print("Roll_rotation2 "+self.angs(self.target_angular_flipper9))
                self.control_angular_flipper9 = self.makeSimpleProfile(self.control_angular_flipper9, self.target_angular_flipper9, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[9]=self.control_angular_flipper9
            if self.palancas[3]<-0.4: 
                self.target_angular_flipper9= self.checkAngularLimitFlipper(self.target_angular_flipper9 - self.ANG_FLIPPER_STEP_SIZE)
                #status9 = status9 + 1
                print("Roll_rotation2 "+self.angs(self.target_angular_flipper9))
                self.control_angular_flipper9 = self.makeSimpleProfile(self.control_angular_flipper9, self.target_angular_flipper9, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[9]=self.control_angular_flipper9

        #--------------------Gripper_rotation--------------------
        if self.palancas[2]<0 and self.palancas[5]<0:
            self.target_linear_vel   = 0.0
            self.control_linear_vel  = 0.0
            self.target_angular_vel  = 0.0
            self.control_angular_vel = 0.0

            if self.palancas[1]>0.4:
                
                self.target_angular_flipper10= self.checkAngularLimitFlipper10(self.target_angular_flipper10 - self.ANG_FLIPPER_STEP_SIZE)
                #status10 = status10 + 1
                print("Gripper_rotation "+self.angs(self.target_angular_flipper10))
                self.control_angular_flipper10 = self.makeSimpleProfile(self.control_angular_flipper10, self.target_angular_flipper10, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[10]=self.control_angular_flipper10

            if self.palancas[1]<-0.4:
                
                self.target_angular_flipper10= self.checkAngularLimitFlipper10(self.target_angular_flipper10 + self.ANG_FLIPPER_STEP_SIZE)
                #status10 = status10 + 1
                print("Gripper_rotation "+self.angs(self.target_angular_flipper10))
                self.control_angular_flipper10 = self.makeSimpleProfile(self.control_angular_flipper10, self.target_angular_flipper10, (self.ANG_FLIPPER_STEP_SIZE))
                self.pos[10]=self.control_angular_flipper10





        twist = Twist()
        self.control_linear_vel = self.makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (self.LIN_VEL_STEP_SIZE/2.0))
        twist.linear.x = self.control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
        self.control_angular_vel = self.makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (self.ANG_VEL_STEP_SIZE/2.0))
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.control_angular_vel
        self.pub.publish(twist)  
        self.pub1.publish(self.pos[0])
        self.pub2.publish(self.pos[1])
        self.pub3.publish(self.pos[2])
        self.pub4.publish(self.pos[3])
        self.pub5.publish(self.pos[4])
        self.pub6.publish(self.pos[5])
        self.pub7.publish(self.pos[6])
        self.pub8.publish(self.pos[7])
        self.pub9.publish(self.pos[8])
        self.pub10.publish(self.pos[9])
        self.pub11.publish(self.pos[10])
        
            
           

        #--------------------Manejo de excepciones--------------------


        #--------------------Se indica que el robot deje de moverse hasta que reciba otro comando de velocidad--------------------
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        self.pub.publish(twist)



    #--------------------Mensaje donde se describen os comandos de movimiento--------------------
    #--------------------Variables para definir el tope de comandos o valores--------------------

    def __init__(self):
        self.msg = """
        Control Your FINDER and Your Flippers!
        ---------------------------

        Moving around:   

            LS AND RS
               

        Left_back_flipper:   
            
            RB + RS 

        Left_front_flipper:   
            
            LB + RS

        Right_front_flipper:   
            
            RB + LS

        Right_back_flipper:   
            
            LB + LS
            

        Arm_base_rotation:   
               
         Press LT + X ---> Turn Left

         Press LT + B ---> Turn Right

        Arm_Shoulder_rotation:   
            
         Press LT + A  ---> Up

         Press LT + Y  ---> Down    

        Arm_Elbow_rotation:   
             
         Press RT + Y  ---> Up

         Press RT + A  ---> Down

        Roll rotation:   
             
         Press RT + X   ---> Turn Left

         Press RT + B   ---> Turn Right

        Pitch_rotation:   
            
         Press RT + LS UP  ---> Up

         Press RT + LS DOWN --> Down  

        Roll rotation2:   
             
         Press RT + RS LEFT----> Turn Left 

         Press RT + RS RIGHT ----> Turn Right 

        Gripper rotation:   
                
            RT + LT + LS UP ---> Open

            RT + LT + LS DOWN  ---> Close    


        Start button : force stop

        CTRL-C to quit
        """
        
        #--------------------Varaibles para definir los pasos--------------------
        self.LIN_VEL_STEP_SIZE=0.1
        self.ANG_VEL_STEP_SIZE=0.2
        self.ANG_FLIPPER_STEP_SIZE=0.01
        self.FINDER_MAX_LIN_VEL = 0.3
        self.FINDER_MAX_ANG_VEL = 0.2
        self.FLIPPER_MAX_VALUE= 3.14
        self.SHOULDER_MAX_VALUE=0.9
        self.ELBOW_MIN_VALUE=-2.3
        self.ROLL_ROTATION_MAX_VALUE= 3.14
        self.PITCH_ROTATION_MAX_VALUE=1.5
        self.ROLL_ROTATION2_MAX_VALUE = 3.14
        self.GRIPPER_ROTATION_MAX_VALUE=1.62
        self.target_linear_vel= 0.0
        self.target_angular_vel= 0.0
        
        self.target_angular_flipper0=0.0
        self.target_angular_flipper1 =0.0
        self.target_angular_flipper2=0.0
        self.target_angular_flipper3=0.0
        self.target_angular_flipper4=0.02
        self.target_angular_flipper5=0.01
        self.target_angular_flipper6=0.01
        self.target_angular_flipper7=0.01
        self.target_angular_flipper8=0.01
        self.target_angular_flipper9=0.01
        self.target_angular_flipper10=0.01
        #--------------------Variables de desplazamiento de la base--------------------

        self.control_linear_vel  = 0.0
        self.control_angular_vel = 0.0
        #--------------------Variables para controlar las juntas--------------------
        self.control_angular_flipper0 = 0.0
        self.control_angular_flipper1 = 0.0
        self.control_angular_flipper2 = 0.0
        self.control_angular_flipper3 = 0.0
        self.control_angular_flipper4 = 0.0
        self.control_angular_flipper5= 0.0
        self.control_angular_flipper6= 0.0
        self.control_angular_flipper7= 0.0
        self.control_angular_flipper8= 0.0
        self.control_angular_flipper9= 1.6
        self.control_angular_flipper10= 1.6
        self.palancas=[]
        self.botones=[]
        self.vel=0
        self.input=0
        self.output=0
        self.slop=0
        self.input2=0
        

        #--------------------Variables para guardar las velocidades de las juntas--------------------
        """"""
        
        self.pos=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.6,1.6]
        

        self.init_node=rospy.init_node('Finder_teleop_joystick')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
        self.pub1=rospy.Publisher('/flipper1_out', Float64, queue_size=1)
        self.pub2=rospy.Publisher('/flipper2_out', Float64, queue_size=1)
        self.pub3=rospy.Publisher('/flipper3_out', Float64, queue_size=1)
        self.pub4=rospy.Publisher('/flipper4_out', Float64, queue_size=1)
        self.pub5=rospy.Publisher('/base_rotation_out',Float32,queue_size=1)
        self.pub6=rospy.Publisher('/shoulder_rotation_out',Float32,queue_size=1)
        self.pub7=rospy.Publisher('/elbow_rotation_out',Float32,queue_size=1)
        self.pub8=rospy.Publisher('/roll_rotation_out',Float32,queue_size=1)
        self.pub9=rospy.Publisher('/pitch_rotation_out',Float32,queue_size=1)
        self.pub10=rospy.Publisher('/roll_rotation_2_out',Float32,queue_size=1)
        self.pub11=rospy.Publisher('/gripper_rotation_out',Float32,queue_size=1)

        #--------------------Variable que controla la alerta del mensaje--------------------
        """status=0
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
        status10=0"""
        

        #--------------------Aqui es donde se declaran todas las operaciones al presionar una de las teclas dentro del menu --------------------

        print(self.msg)
        
        

        
    #--------------------Se asignan los valores lineales y angulares de movimiento de la base--------------------    



if __name__=="__main__":
    
    nodo=Nodo()
    rospy.Subscriber("/joint_states",JointState,nodo.callback)
    sleep(1)
    rospy.Subscriber("/joy",Joy,nodo.joystick_callback)
    rospy.spin()

